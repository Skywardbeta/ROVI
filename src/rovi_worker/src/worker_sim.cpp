#include "rovi_worker/worker_sim.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <utility>
#include <vector>

#include "rovi_core/admission.hpp"
#include "rovi_core/units.hpp"

namespace rovi::worker {
namespace {

std::optional<core::TimePoint> finish_time(core::TimePoint start, core::Duration execution,
                                           core::Duration slowdown) {
  std::int64_t total_ns{};
  if (__builtin_add_overflow(execution.ns(), slowdown.ns(), &total_ns)) {
    return std::nullopt;
  }
  return core::checked_add(start, core::Duration::from_ns(total_ns));
}

}  // namespace

WorkerSim::WorkerSim(std::map<core::ModelClass, core::Duration> durations, WorkerTiming timing)
    : durations_{std::move(durations)}, timing_{timing} {}

std::optional<WorkerSim> WorkerSim::create(const std::vector<core::ExecutionProfile>& profiles,
                                           WorkerTiming timing) {
  if (profiles.empty() || timing.slowdown.ns() < 0 || timing.orphan_timeout.ns() <= 0 ||
      timing.max_lead.ns() < 0) {
    return std::nullopt;
  }
  std::map<core::ModelClass, core::Duration> durations;
  const bool invalid =
      std::ranges::any_of(profiles, [&durations](const core::ExecutionProfile& profile) {
        return profile.duration.ns() <= 0 ||
               !durations.emplace(profile.model, profile.duration).second;
      });
  if (invalid) {
    return std::nullopt;
  }
  return WorkerSim{std::move(durations), timing};
}

bool WorkerSim::assign(const Assignment& assignment, core::TimePoint now) {
  const bool running = running_ && running_->job == assignment.job;
  const std::optional<core::TimePoint> latest_start = core::checked_add(now, timing_.max_lead);
  const bool beyond_lead = latest_start && assignment.planned_start > *latest_start;
  if (running || beyond_lead || !durations_.contains(assignment.model)) {
    return false;
  }
  return assigned_.try_emplace(assignment.job, assignment).second;
}

void WorkerSim::arrive(core::JobId job, core::TimePoint at, core::TimePoint observation_time) {
  arrived_.try_emplace(job, Arrival{.at = at, .observation_time = observation_time});
}

std::vector<Completion> WorkerSim::advance(core::TimePoint now) {
  std::vector<Completion> completed;
  while (true) {
    if (running_) {
      if (running_->end > now) {
        break;
      }
      lane_free_ = running_->end;
      completed.push_back(*running_);
      running_.reset();
    }
    if (!start_next(now)) {
      break;
    }
  }
  prune(now);
  return completed;
}

bool WorkerSim::start_next(core::TimePoint now) {
  auto next = assigned_.end();
  for (auto entry = assigned_.begin(); entry != assigned_.end(); ++entry) {
    const bool ready = arrived_.contains(entry->first);
    if (ready &&
        (next == assigned_.end() || entry->second.planned_start < next->second.planned_start)) {
      next = entry;
    }
  }
  if (next == assigned_.end()) {
    return false;
  }
  const Assignment job = next->second;
  const Arrival arrival = arrived_.at(job.job);
  const core::TimePoint start =
      std::max({job.planned_start, arrival.at, lane_free_.value_or(job.planned_start)});
  if (start > now) {
    return false;
  }
  assigned_.erase(next);
  arrived_.erase(job.job);
  const std::optional<core::TimePoint> end =
      finish_time(start, durations_.at(job.model), timing_.slowdown);
  if (!end) {
    ++discarded_;
    return true;
  }
  running_ = Completion{.job = job.job,
                        .robot = job.robot,
                        .chunk_bytes = job.chunk_bytes,
                        .observation_time = arrival.observation_time,
                        .start = start,
                        .end = *end};
  return true;
}

void WorkerSim::prune(core::TimePoint now) {
  discarded_ += std::erase_if(arrived_, [this, now](const auto& entry) {
    return !assigned_.contains(entry.first) &&
           core::elapsed_beyond(entry.second.at, timing_.orphan_timeout, now);
  });
  discarded_ += std::erase_if(assigned_, [this, now](const auto& entry) {
    return !arrived_.contains(entry.first) &&
           core::elapsed_beyond(entry.second.planned_start, timing_.orphan_timeout, now);
  });
}

std::size_t WorkerSim::queued() const { return assigned_.size(); }

std::uint64_t WorkerSim::discarded() const { return discarded_; }

}  // namespace rovi::worker
