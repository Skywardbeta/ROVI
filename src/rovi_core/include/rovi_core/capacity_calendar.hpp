// Fixed-slot calendar of usable and reserved network-service bytes for one directed resource.
#ifndef ROVI_CORE_CAPACITY_CALENDAR_HPP_
#define ROVI_CORE_CAPACITY_CALENDAR_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "rovi_core/units.hpp"

namespace rovi::core {

struct ServiceSlice {
  SlotIndex slot;
  Bytes bytes;

  bool operator==(const ServiceSlice&) const = default;
};

struct Transfer {
  std::vector<ServiceSlice> slices;
  TimePoint completion;

  bool operator==(const Transfer&) const = default;
};

class CapacityCalendar {
 public:
  [[nodiscard]] static std::optional<CapacityCalendar> create(TimePoint origin,
                                                              Duration slot_length,
                                                              std::vector<Bytes> capacity_per_slot);

  [[nodiscard]] std::optional<Transfer> plan_transfer(TimePoint earliest_start, Bytes demand) const;
  [[nodiscard]] std::optional<Transfer> plan_transfer(TimePoint earliest_start, Bytes demand,
                                                      const CapacityCalendar& limit) const;
  // Plans within the residual of every limit, which must share this calendar's geometry.
  [[nodiscard]] std::optional<Transfer> plan_transfer(
      TimePoint earliest_start, Bytes demand,
      std::span<const CapacityCalendar* const> limits) const;
  [[nodiscard]] std::optional<CapacityCalendar> reserve(const Transfer& transfer) const;
  [[nodiscard]] std::optional<CapacityCalendar> advance(
      SlotIndex first_kept, const std::vector<Bytes>& appended_capacity) const;
  [[nodiscard]] Bytes capacity(SlotIndex slot) const;
  [[nodiscard]] Bytes reserved(SlotIndex slot) const;
  [[nodiscard]] std::optional<TimePoint> slot_start(SlotIndex slot) const;
  [[nodiscard]] std::int64_t slot_count() const;
  [[nodiscard]] Duration slot_length() const;
  [[nodiscard]] TimePoint horizon_end() const;
  [[nodiscard]] bool same_geometry(const CapacityCalendar& other) const;

  bool operator==(const CapacityCalendar&) const = default;

 private:
  CapacityCalendar(TimePoint origin, Duration slot_length, std::vector<Bytes> capacity,
                   std::vector<Bytes> reserved);

  [[nodiscard]] std::optional<Transfer> plan(TimePoint earliest_start, Bytes demand,
                                             std::span<const CapacityCalendar* const> limits) const;
  [[nodiscard]] std::int64_t residual(std::size_t slot) const;
  [[nodiscard]] std::optional<std::size_t> index_of(SlotIndex slot) const;
  [[nodiscard]] TimePoint boundary_at(std::size_t boundary) const;
  [[nodiscard]] std::size_t first_slot_at_or_after(TimePoint time) const;

  TimePoint origin_;
  Duration slot_length_;
  std::vector<Bytes> capacity_;
  std::vector<Bytes> reserved_;
};

}  // namespace rovi::core

#endif  // ROVI_CORE_CAPACITY_CALENDAR_HPP_
