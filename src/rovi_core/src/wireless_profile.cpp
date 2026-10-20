#include "rovi_core/wireless_profile.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "rovi_core/capacity_calendar.hpp"
#include "rovi_core/units.hpp"

namespace rovi::core {
namespace {

constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000;
constexpr std::int64_t kWholePermille = 1'000;
constexpr std::int64_t kBitsPerByte = 8;

}  // namespace

std::optional<Bytes> bytes_per_slot(BitsPerSecond rate, Duration slot_length, Permille reduction) {
  if (rate.value() < 0 || slot_length.ns() <= 0 || reduction.value() < 0 ||
      reduction.value() > kWholePermille) {
    return std::nullopt;
  }
  std::int64_t bit_nanoseconds{};
  if (__builtin_mul_overflow(rate.value(), slot_length.ns(), &bit_nanoseconds)) {
    return std::nullopt;
  }
  const std::int64_t bits = bit_nanoseconds / kNanosecondsPerSecond;
  const std::int64_t usable_bits = bits * (kWholePermille - reduction.value()) / kWholePermille;
  return Bytes{usable_bits / kBitsPerByte};
}

std::optional<std::vector<Bytes>> service_capacity(Duration slot_length,
                                                   const std::vector<ServiceSegment>& segments,
                                                   std::int64_t first_slot,
                                                   std::int64_t slot_count) {
  std::int64_t end_slot{};
  if (slot_count <= 0 || first_slot < 0 ||
      __builtin_add_overflow(first_slot, slot_count, &end_slot) || segments.empty() ||
      segments.front().first_slot != 0) {
    return std::nullopt;
  }
  std::vector<Bytes> segment_capacity;
  segment_capacity.reserve(segments.size());
  for (std::size_t index = 0; index < segments.size(); ++index) {
    const ServiceSegment& segment = segments.at(index);
    const std::optional<Bytes> per_slot =
        bytes_per_slot(segment.rate, slot_length, segment.reduction);
    if (!per_slot || (index > 0 && segment.first_slot <= segments.at(index - 1).first_slot)) {
      return std::nullopt;
    }
    segment_capacity.push_back(*per_slot);
  }
  std::vector<Bytes> capacity;
  capacity.reserve(static_cast<std::size_t>(slot_count));
  std::size_t current = 0;
  for (std::int64_t slot = first_slot; slot < end_slot; ++slot) {
    while (current + 1 < segments.size() && segments.at(current + 1).first_slot <= slot) {
      ++current;
    }
    capacity.push_back(segment_capacity.at(current));
  }
  return capacity;
}

std::optional<CapacityCalendar> build_service_calendar(
    TimePoint origin, Duration slot_length, std::int64_t slot_count,
    const std::vector<ServiceSegment>& segments) {
  if (segments.empty() || segments.back().first_slot >= slot_count) {
    return std::nullopt;
  }
  std::optional<std::vector<Bytes>> capacity =
      service_capacity(slot_length, segments, 0, slot_count);
  if (!capacity) {
    return std::nullopt;
  }
  return CapacityCalendar::create(origin, slot_length, std::move(*capacity));
}

}  // namespace rovi::core
