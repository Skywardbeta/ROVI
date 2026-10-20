// Rolling-horizon window that slides network-service calendars and compute lanes over time.
#ifndef ROVI_CORE_WINDOW_HPP_
#define ROVI_CORE_WINDOW_HPP_

#include <cstdint>
#include <optional>
#include <vector>

#include "rovi_core/admission.hpp"
#include "rovi_core/units.hpp"
#include "rovi_core/wireless_profile.hpp"

namespace rovi::core {

struct ServiceProfile {
  Duration slot_length;
  std::vector<ServiceSegment> uplink;
  std::vector<ServiceSegment> downlink;
  std::optional<std::vector<ServiceSegment>> medium{};
  Duration uplink_delay{};
  Duration downlink_delay{};
  Duration dispatch_lead{};
  ServiceSource source{ServiceSource::kLogical};

  bool operator==(const ServiceProfile&) const = default;
};

struct WorkerSpec {
  WorkerId id;
  std::vector<ExecutionProfile> profiles;

  bool operator==(const WorkerSpec&) const = default;
};

[[nodiscard]] std::optional<ResourceSnapshot> open_window(const ServiceProfile& profile,
                                                          const std::vector<WorkerSpec>& workers,
                                                          TimePoint epoch, std::int64_t slot_count);

[[nodiscard]] std::optional<ResourceSnapshot> advance_window(const ResourceSnapshot& current,
                                                             const ServiceProfile& profile,
                                                             TimePoint epoch, TimePoint now);

}  // namespace rovi::core

#endif  // ROVI_CORE_WINDOW_HPP_
