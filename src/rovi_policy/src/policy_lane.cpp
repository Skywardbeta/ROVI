#include "rovi_policy/policy_lane.hpp"

#include <cstdint>
#include <iterator>
#include <map>
#include <optional>
#include <utility>

#include "rovi_core/units.hpp"
#include "rovi_policy/wire.hpp"

namespace rovi::policy {

PolicyLane::PolicyLane(LaneTiming timing) : timing_{timing} {}

std::optional<PolicyLane> PolicyLane::create(LaneTiming timing) {
  if (timing.orphan_timeout.ns() <= 0 || timing.max_lead.ns() < 0) {
    return std::nullopt;
  }
  return PolicyLane{timing};
}

bool PolicyLane::assign(const LaneJob& job, core::TimePoint now) {
  const std::optional<core::TimePoint> latest_start = core::checked_add(now, timing_.max_lead);
  if (!latest_start || job.planned_start > *latest_start || job.planned_end < job.planned_start ||
      running_ == job.job) {
    return false;
  }
  return assigned_.try_emplace(job.job, job).second;
}

bool PolicyLane::arrive(core::JobId job, core::TimePoint at, core::TimePoint observation_time,
                        Observation observation) {
  if (running_ == job) {
    return false;
  }
  return arrived_
      .try_emplace(job, Arrival{.at = at,
                                .observation_time = observation_time,
                                .observation = std::move(observation)})
      .second;
}

void PolicyLane::prune(core::TimePoint now) {
  for (auto entry = assigned_.begin(); entry != assigned_.end();) {
    const bool ready = arrived_.contains(entry->first);
    const bool passed =
        ready ? now > entry->second.planned_end
              : core::elapsed_beyond(entry->second.planned_start, timing_.orphan_timeout, now);
    if (passed) {
      arrived_.erase(entry->first);
      entry = assigned_.erase(entry);
      ++discarded_;
    } else {
      entry = std::next(entry);
    }
  }
  discarded_ += std::erase_if(arrived_, [this, now](const auto& entry) {
    return !assigned_.contains(entry.first) &&
           core::elapsed_beyond(entry.second.at, timing_.orphan_timeout, now);
  });
}

std::optional<Started> PolicyLane::start(core::TimePoint now) {
  if (running_) {
    return std::nullopt;
  }
  auto next = assigned_.end();
  for (auto entry = assigned_.begin(); entry != assigned_.end(); ++entry) {
    if (arrived_.contains(entry->first) && entry->second.planned_start <= now &&
        (next == assigned_.end() || entry->second.planned_start < next->second.planned_start)) {
      next = entry;
    }
  }
  if (next == assigned_.end()) {
    return std::nullopt;
  }
  auto arrival = arrived_.find(next->first);
  Started started{.job = next->second,
                  .observation_time = arrival->second.observation_time,
                  .observation = std::move(arrival->second.observation)};
  arrived_.erase(arrival);
  assigned_.erase(next);
  running_ = started.job.job;
  return started;
}

bool PolicyLane::finish(core::JobId job) {
  if (running_ != job) {
    return false;
  }
  running_.reset();
  return true;
}

bool PolicyLane::busy() const { return running_.has_value(); }

std::uint64_t PolicyLane::discarded() const { return discarded_; }

}  // namespace rovi::policy
