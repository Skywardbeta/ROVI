// Logical wireless profiles that turn integer rates, loss overhead, and outages into capacity.
#ifndef ROVI_CORE_WIRELESS_PROFILE_HPP_
#define ROVI_CORE_WIRELESS_PROFILE_HPP_

#include <cstdint>
#include <optional>
#include <vector>

#include "rovi_core/capacity_calendar.hpp"
#include "rovi_core/units.hpp"

namespace rovi::core {

struct ServiceSegment {
  std::int64_t first_slot{0};
  BitsPerSecond rate{};
  Permille reduction{};

  bool operator==(const ServiceSegment&) const = default;
};

[[nodiscard]] std::optional<Bytes> bytes_per_slot(BitsPerSecond rate, Duration slot_length,
                                                  Permille reduction);

[[nodiscard]] std::optional<std::vector<Bytes>> service_capacity(
    Duration slot_length, const std::vector<ServiceSegment>& segments, std::int64_t first_slot,
    std::int64_t slot_count);

[[nodiscard]] std::optional<CapacityCalendar> build_service_calendar(
    TimePoint origin, Duration slot_length, std::int64_t slot_count,
    const std::vector<ServiceSegment>& segments);

}  // namespace rovi::core

#endif  // ROVI_CORE_WIRELESS_PROFILE_HPP_
