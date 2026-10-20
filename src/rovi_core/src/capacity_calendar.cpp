#include "rovi_core/capacity_calendar.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <numeric>
#include <optional>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

#include "rovi_core/units.hpp"

namespace rovi::core {

CapacityCalendar::CapacityCalendar(TimePoint origin, Duration slot_length,
                                   std::vector<Bytes> capacity, std::vector<Bytes> reserved)
    : origin_{origin},
      slot_length_{slot_length},
      capacity_{std::move(capacity)},
      reserved_{std::move(reserved)} {}

std::optional<CapacityCalendar> CapacityCalendar::create(TimePoint origin, Duration slot_length,
                                                         std::vector<Bytes> capacity_per_slot) {
  if (slot_length.ns() <= 0 || capacity_per_slot.empty()) {
    return std::nullopt;
  }
  if (std::ranges::any_of(capacity_per_slot, [](Bytes capacity) { return capacity.value() < 0; })) {
    return std::nullopt;
  }
  std::int64_t horizon_ns{};
  if (__builtin_mul_overflow(capacity_per_slot.size(), slot_length.ns(), &horizon_ns) ||
      !checked_add(origin, Duration::from_ns(horizon_ns))) {
    return std::nullopt;
  }
  std::vector<Bytes> no_reservations(capacity_per_slot.size());
  return CapacityCalendar{origin, slot_length, std::move(capacity_per_slot),
                          std::move(no_reservations)};
}

std::optional<Transfer> CapacityCalendar::plan_transfer(TimePoint earliest_start,
                                                        Bytes demand) const {
  return plan(earliest_start, demand, {});
}

std::optional<Transfer> CapacityCalendar::plan_transfer(TimePoint earliest_start, Bytes demand,
                                                        const CapacityCalendar& limit) const {
  const std::array<const CapacityCalendar*, 1> limits{&limit};
  return plan_transfer(earliest_start, demand, limits);
}

std::optional<Transfer> CapacityCalendar::plan_transfer(
    TimePoint earliest_start, Bytes demand, std::span<const CapacityCalendar* const> limits) const {
  if (std::ranges::any_of(limits, [this](const CapacityCalendar* limit) {
        return limit == nullptr || !same_geometry(*limit);
      })) {
    return std::nullopt;
  }
  return plan(earliest_start, demand, limits);
}

std::optional<Transfer> CapacityCalendar::plan(
    TimePoint earliest_start, Bytes demand, std::span<const CapacityCalendar* const> limits) const {
  if (demand.value() <= 0) {
    return std::nullopt;
  }
  std::vector<ServiceSlice> slices;
  std::int64_t outstanding = demand.value();
  for (std::size_t slot = first_slot_at_or_after(earliest_start); slot < capacity_.size(); ++slot) {
    const std::int64_t available =
        std::accumulate(limits.begin(), limits.end(), residual(slot),
                        [slot](std::int64_t least, const CapacityCalendar* limit) {
                          return std::min(least, limit->residual(slot));
                        });
    if (available <= 0) {
      continue;
    }
    const std::int64_t granted = std::min(available, outstanding);
    slices.push_back(
        ServiceSlice{.slot = SlotIndex{static_cast<std::int64_t>(slot)}, .bytes = Bytes{granted}});
    outstanding -= granted;
    if (outstanding == 0) {
      return Transfer{.slices = std::move(slices), .completion = boundary_at(slot + 1)};
    }
  }
  return std::nullopt;
}

std::optional<CapacityCalendar> CapacityCalendar::reserve(const Transfer& transfer) const {
  if (transfer.slices.empty()) {
    return std::nullopt;
  }
  std::vector<Bytes> next_reserved = reserved_;
  std::size_t next_allowed_slot = 0;
  for (const ServiceSlice& slice : transfer.slices) {
    const std::optional<std::size_t> index = index_of(slice.slot);
    if (!index || *index < next_allowed_slot || slice.bytes.value() <= 0) {
      return std::nullopt;
    }
    const std::int64_t already_reserved = reserved_.at(*index).value();
    if (slice.bytes.value() > capacity_.at(*index).value() - already_reserved) {
      return std::nullopt;
    }
    next_reserved.at(*index) = Bytes{already_reserved + slice.bytes.value()};
    next_allowed_slot = *index + 1;
  }
  if (transfer.completion != boundary_at(next_allowed_slot)) {
    return std::nullopt;
  }
  return CapacityCalendar{origin_, slot_length_, capacity_, std::move(next_reserved)};
}

std::optional<CapacityCalendar> CapacityCalendar::advance(
    SlotIndex first_kept, const std::vector<Bytes>& appended_capacity) const {
  if (first_kept.value() < 0 || first_kept.value() > std::ssize(capacity_)) {
    return std::nullopt;
  }
  std::vector<Bytes> kept_capacity{std::next(capacity_.begin(), first_kept.value()),
                                   capacity_.end()};
  std::vector<Bytes> kept_reserved{std::next(reserved_.begin(), first_kept.value()),
                                   reserved_.end()};
  kept_capacity.insert(kept_capacity.end(), appended_capacity.begin(), appended_capacity.end());
  kept_reserved.resize(kept_capacity.size());
  std::optional<CapacityCalendar> shifted =
      create(boundary_at(static_cast<std::size_t>(first_kept.value())), slot_length_,
             std::move(kept_capacity));
  if (shifted) {
    shifted->reserved_ = std::move(kept_reserved);
  }
  return shifted;
}

Bytes CapacityCalendar::capacity(SlotIndex slot) const {
  const std::optional<std::size_t> index = index_of(slot);
  if (!index) {
    throw std::out_of_range{"slot is outside the capacity calendar horizon"};
  }
  return capacity_.at(*index);
}

std::int64_t CapacityCalendar::slot_count() const { return std::ssize(capacity_); }

Duration CapacityCalendar::slot_length() const { return slot_length_; }

TimePoint CapacityCalendar::horizon_end() const { return boundary_at(capacity_.size()); }

bool CapacityCalendar::same_geometry(const CapacityCalendar& other) const {
  return origin_ == other.origin_ && slot_length_ == other.slot_length_ &&
         capacity_.size() == other.capacity_.size();
}

Bytes CapacityCalendar::reserved(SlotIndex slot) const {
  const std::optional<std::size_t> index = index_of(slot);
  if (!index) {
    throw std::out_of_range{"slot is outside the capacity calendar horizon"};
  }
  return reserved_.at(*index);
}

std::optional<TimePoint> CapacityCalendar::slot_start(SlotIndex slot) const {
  const std::optional<std::size_t> index = index_of(slot);
  if (!index) {
    return std::nullopt;
  }
  return boundary_at(*index);
}

std::int64_t CapacityCalendar::residual(std::size_t slot) const {
  return capacity_.at(slot).value() - reserved_.at(slot).value();
}

std::optional<std::size_t> CapacityCalendar::index_of(SlotIndex slot) const {
  if (slot.value() < 0 || slot.value() >= std::ssize(capacity_)) {
    return std::nullopt;
  }
  return static_cast<std::size_t>(slot.value());
}

TimePoint CapacityCalendar::boundary_at(std::size_t boundary) const {
  return TimePoint::from_ns(origin_.ns() +
                            (static_cast<std::int64_t>(boundary) * slot_length_.ns()));
}

std::size_t CapacityCalendar::first_slot_at_or_after(TimePoint time) const {
  if (time <= origin_) {
    return 0;
  }
  if (time > boundary_at(capacity_.size())) {
    return capacity_.size();
  }
  const std::int64_t offset = time.ns() - origin_.ns();
  const std::int64_t whole_slots = offset / slot_length_.ns();
  const bool on_boundary = offset % slot_length_.ns() == 0;
  return static_cast<std::size_t>(on_boundary ? whole_slots : whole_slots + 1);
}

}  // namespace rovi::core
