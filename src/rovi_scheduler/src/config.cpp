#include "rovi_scheduler/config.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

#include "rovi_core/admission.hpp"
#include "rovi_core/digest.hpp"
#include "rovi_core/scheduler.hpp"
#include "rovi_core/units.hpp"
#include "rovi_core/window.hpp"
#include "rovi_core/wireless_profile.hpp"

namespace rovi::scheduler {
namespace {

void add_segments(core::Fnv1aDigest& digest, const std::vector<core::ServiceSegment>& segments) {
  digest.add(segments.size());
  for (const core::ServiceSegment& segment : segments) {
    digest.add(segment.first_slot);
    digest.add(segment.rate.value());
    digest.add(std::int64_t{segment.reduction.value()});
  }
}

}  // namespace

std::optional<core::Policy> policy_from(std::string_view name) {
  if (name == "least_slack") {
    return core::Policy::kLeastSlack;
  }
  if (name == "earliest_deadline") {
    return core::Policy::kEarliestDeadline;
  }
  if (name == "first_in_first_out") {
    return core::Policy::kFirstInFirstOut;
  }
  return std::nullopt;
}

std::optional<std::vector<core::ServiceSegment>> segments_from(const SegmentArrays& arrays) {
  const std::size_t count = arrays.first_slots.size();
  if (count == 0 || arrays.rates_bps.size() != count ||
      arrays.reductions_permille.size() != count) {
    return std::nullopt;
  }
  std::vector<core::ServiceSegment> segments;
  segments.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    const std::int64_t reduction = arrays.reductions_permille.at(index);
    if (!std::in_range<std::int32_t>(reduction)) {
      return std::nullopt;
    }
    segments.push_back(
        core::ServiceSegment{.first_slot = arrays.first_slots.at(index),
                             .rate = core::BitsPerSecond{arrays.rates_bps.at(index)},
                             .reduction = core::Permille{static_cast<std::int32_t>(reduction)}});
  }
  return segments;
}

std::optional<std::vector<core::WorkerSpec>> workers_from(const WorkerArrays& arrays) {
  const std::size_t count = arrays.ids.size();
  if (count == 0 || arrays.model_classes.size() != count || arrays.durations_ns.size() != count) {
    return std::nullopt;
  }
  std::map<std::uint64_t, core::WorkerSpec> by_id;
  for (std::size_t index = 0; index < count; ++index) {
    const std::int64_t id = arrays.ids.at(index);
    const std::int64_t model = arrays.model_classes.at(index);
    if (!std::in_range<std::uint64_t>(id) || !std::in_range<std::uint32_t>(model)) {
      return std::nullopt;
    }
    const auto worker_id = static_cast<std::uint64_t>(id);
    auto [entry, inserted] = by_id.try_emplace(
        worker_id, core::WorkerSpec{.id = core::WorkerId{worker_id}, .profiles = {}});
    entry->second.profiles.push_back(
        core::ExecutionProfile{.model = core::ModelClass{static_cast<std::uint32_t>(model)},
                               .duration = core::Duration::from_ns(arrays.durations_ns.at(index))});
  }
  std::vector<core::WorkerSpec> workers;
  workers.reserve(by_id.size());
  for (auto& [id, worker] : by_id) {
    workers.push_back(std::move(worker));
  }
  return workers;
}

bool is_valid(const SchedulerConfig& config) {
  if (config.cycle_period.ns() <= 0 || config.release_tolerance.ns() < 0 ||
      config.horizon_slots <= 0 || config.horizon_slots > kMaximumHorizonSlots ||
      config.max_pending_jobs <= 0 || config.max_pending_jobs > kMaximumPendingJobs ||
      config.workers.empty()) {
    return false;
  }
  std::int64_t window_ns{};
  if (__builtin_mul_overflow(config.horizon_slots, config.profile.slot_length.ns(), &window_ns) ||
      config.release_tolerance.ns() > window_ns) {
    return false;
  }
  return core::open_window(config.profile, config.workers, core::TimePoint{}, config.horizon_slots)
      .has_value();
}

std::uint64_t config_digest(const SchedulerConfig& config) {
  core::Fnv1aDigest digest;
  digest.add(static_cast<std::uint64_t>(config.policy));
  digest.add(config.cycle_period.ns());
  digest.add(config.release_tolerance.ns());
  digest.add(config.horizon_slots);
  digest.add(config.max_pending_jobs);
  const core::ServiceProfile& profile = config.profile;
  digest.add(profile.slot_length.ns());
  add_segments(digest, profile.uplink);
  add_segments(digest, profile.downlink);
  digest.add(static_cast<std::uint64_t>(profile.medium.has_value()));
  if (profile.medium) {
    add_segments(digest, *profile.medium);
  }
  digest.add(profile.uplink_delay.ns());
  digest.add(profile.downlink_delay.ns());
  digest.add(profile.dispatch_lead.ns());
  digest.add(static_cast<std::uint64_t>(profile.source));
  digest.add(config.workers.size());
  for (const core::WorkerSpec& worker : config.workers) {
    digest.add(worker.id.value());
    digest.add(worker.profiles.size());
    for (const core::ExecutionProfile& execution : worker.profiles) {
      digest.add(std::uint64_t{execution.model.value()});
      digest.add(execution.duration.ns());
    }
  }
  return digest.value();
}

}  // namespace rovi::scheduler
