// Validated scheduler configuration parsed from flat integer parameter arrays.
#ifndef ROVI_SCHEDULER_CONFIG_HPP_
#define ROVI_SCHEDULER_CONFIG_HPP_

#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

#include "rovi_core/scheduler.hpp"
#include "rovi_core/units.hpp"
#include "rovi_core/window.hpp"
#include "rovi_core/wireless_profile.hpp"

namespace rovi::scheduler {

inline constexpr std::int64_t kMaximumHorizonSlots = 100'000;
inline constexpr std::int64_t kMaximumPendingJobs = 4'096;

struct SegmentArrays {
  std::vector<std::int64_t> first_slots;
  std::vector<std::int64_t> rates_bps;
  std::vector<std::int64_t> reductions_permille;
};

struct WorkerArrays {
  std::vector<std::int64_t> ids;
  std::vector<std::int64_t> model_classes;
  std::vector<std::int64_t> durations_ns;
};

struct SchedulerConfig {
  core::Policy policy{core::Policy::kLeastSlack};
  core::Duration cycle_period{};
  core::Duration release_tolerance{};
  std::int64_t horizon_slots{0};
  std::int64_t max_pending_jobs{0};
  core::ServiceProfile profile{};
  std::vector<core::WorkerSpec> workers{};
};

[[nodiscard]] std::optional<core::Policy> policy_from(std::string_view name);

[[nodiscard]] std::optional<std::vector<core::ServiceSegment>> segments_from(
    const SegmentArrays& arrays);

[[nodiscard]] std::optional<std::vector<core::WorkerSpec>> workers_from(const WorkerArrays& arrays);

[[nodiscard]] bool is_valid(const SchedulerConfig& config);

[[nodiscard]] std::uint64_t config_digest(const SchedulerConfig& config);

}  // namespace rovi::scheduler

#endif  // ROVI_SCHEDULER_CONFIG_HPP_
