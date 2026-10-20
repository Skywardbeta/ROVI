#include "rovi_scheduler/job_book.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <map>
#include <optional>
#include <set>
#include <utility>
#include <vector>

#include "rovi_core/admission.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/scheduler.hpp"
#include "rovi_core/units.hpp"

namespace rovi::scheduler {
namespace {

core::JobDecision rejected(core::JobId job, core::ReasonCode reason) {
  return core::JobDecision{
      .job = job,
      .decision = core::Decision{.status = core::DecisionStatus::kRejected, .reason = reason},
      .reservation = std::nullopt};
}

std::optional<core::ReasonCode> lateness(const core::Job& job, core::TimePoint now) {
  const std::optional<core::TimePoint> deadline = core::effective_deadline(job);
  if (!deadline) {
    return core::ReasonCode::kArithmeticOverflow;
  }
  if (*deadline > now) {
    return std::nullopt;
  }
  const std::optional<core::TimePoint> fresh_until = core::freshness_deadline(job);
  return fresh_until && *fresh_until == *deadline ? core::ReasonCode::kStaleResult
                                                  : core::ReasonCode::kDeadlineMiss;
}

}  // namespace

JobBook::JobBook(BookLimits limits) : limits_{limits} {}

std::optional<core::ReasonCode> JobBook::refusal(const core::Job& job, core::TimePoint now,
                                                 std::uint32_t group_size) const {
  if (decided_.contains(job.id) ||
      std::ranges::find(pending_, job.id, &core::Job::id) != pending_.end()) {
    return core::ReasonCode::kDuplicateJob;
  }
  if (job.eligible_workers.size() > kMaximumRequestListLength ||
      job.predecessors.size() > kMaximumRequestListLength) {
    return core::ReasonCode::kMalformedRequest;
  }
  if (const std::optional<core::ReasonCode> reason = group_refusal(job, group_size)) {
    return reason;
  }
  const std::optional<core::TimePoint> latest_release =
      core::checked_add(now, limits_.release_tolerance);
  if (latest_release && job.release_time > *latest_release) {
    return core::ReasonCode::kInvalidTiming;
  }
  if (pending_.size() >= limits_.max_pending) {
    return core::ReasonCode::kQueueFull;
  }
  return std::nullopt;
}

std::optional<core::ReasonCode> JobBook::group_refusal(const core::Job& job,
                                                       std::uint32_t group_size) const {
  if (!job.synchronization_group) {
    return group_size == 0 ? std::nullopt : std::optional{core::ReasonCode::kMalformedRequest};
  }
  const core::GroupId group = *job.synchronization_group;
  if (group_size == 0 || group_size > kMaximumRequestListLength) {
    return core::ReasonCode::kMalformedRequest;
  }
  if (closed_groups_.contains(group)) {
    return core::ReasonCode::kGroupMemberUnavailable;
  }
  const auto known = group_sizes_.find(group);
  if (known != group_sizes_.end() && (known->second != group_size || group_complete(group))) {
    return core::ReasonCode::kMalformedRequest;
  }
  return std::nullopt;
}

std::optional<core::JobDecision> JobBook::submit(const core::Job& job, core::TimePoint now,
                                                 std::uint32_t group_size) {
  const std::optional<core::ReasonCode> reason = refusal(job, now, group_size);
  if (!reason) {
    pending_.push_back(job);
    if (job.synchronization_group) {
      group_sizes_.try_emplace(*job.synchronization_group, group_size);
    }
    return std::nullopt;
  }
  if (*reason != core::ReasonCode::kDuplicateJob) {
    decided_.insert(job.id);
  }
  return rejected(job.id, *reason);
}

bool JobBook::group_complete(core::GroupId group) const {
  const auto known = group_sizes_.find(group);
  const auto members =
      std::ranges::count(pending_, std::optional{group}, &core::Job::synchronization_group);
  return known != group_sizes_.end() && std::cmp_equal(members, known->second);
}

bool JobBook::group_ready(const core::Job& job, core::TimePoint now) const {
  if (!job.synchronization_group) {
    return true;
  }
  return group_complete(*job.synchronization_group) &&
         std::ranges::all_of(pending_, [&job, now](const core::Job& other) {
           return other.synchronization_group != job.synchronization_group ||
                  other.release_time <= now;
         });
}

void JobBook::close_group(const core::Job& job) {
  if (!job.synchronization_group) {
    return;
  }
  group_sizes_.erase(*job.synchronization_group);
  if (!closed_groups_.insert(*job.synchronization_group).second) {
    return;
  }
  closed_order_.push_back(*job.synchronization_group);
  if (closed_order_.size() > kClosedGroupMemory) {
    closed_groups_.erase(closed_order_.front());
    closed_order_.pop_front();
  }
}

bool JobBook::complete(core::JobId job) {
  return admitted_.contains(job) && completed_.insert(job).second;
}

std::vector<core::Job> JobBook::released(core::TimePoint now) const {
  std::vector<core::Job> ready;
  std::ranges::copy_if(pending_, std::back_inserter(ready), [this, now](const core::Job& job) {
    return job.release_time <= now && group_ready(job, now);
  });
  return ready;
}

std::vector<core::JobId> JobBook::completed_for(const std::vector<core::Job>& jobs) const {
  std::vector<core::JobId> relevant;
  for (const core::Job& job : jobs) {
    std::ranges::copy_if(
        job.predecessors, std::back_inserter(relevant),
        [this](core::JobId predecessor) { return completed_.contains(predecessor); });
  }
  std::ranges::sort(relevant);
  const auto duplicates = std::ranges::unique(relevant);
  relevant.erase(duplicates.begin(), duplicates.end());
  return relevant;
}

std::size_t JobBook::pending_count() const { return pending_.size(); }

std::vector<Notice> JobBook::expire(core::TimePoint now) {
  std::map<core::JobId, core::ReasonCode> expiring;
  std::set<core::GroupId> expiring_groups;
  for (const core::Job& job : pending_) {
    const bool incomplete =
        job.synchronization_group && !group_complete(*job.synchronization_group);
    if (!incomplete && !reported_deferrals_.contains(job.id)) {
      continue;
    }
    if (const std::optional<core::ReasonCode> reason = lateness(job, now)) {
      expiring.emplace(job.id, incomplete ? core::ReasonCode::kGroupMemberUnavailable : *reason);
      if (job.synchronization_group) {
        expiring_groups.insert(*job.synchronization_group);
      }
    }
  }
  std::vector<Notice> notices;
  std::erase_if(pending_, [&](const core::Job& job) {
    const auto own = expiring.find(job.id);
    const bool with_group =
        job.synchronization_group && expiring_groups.contains(*job.synchronization_group);
    if (own == expiring.end() && !with_group) {
      return false;
    }
    const core::ReasonCode reason =
        own != expiring.end() ? own->second : core::ReasonCode::kGroupMemberUnavailable;
    notices.push_back(Notice{.job = job, .decision = rejected(job.id, reason)});
    decided_.insert(job.id);
    reported_deferrals_.erase(job.id);
    return true;
  });
  for (const Notice& notice : notices) {
    close_group(notice.job);
  }
  return notices;
}

std::vector<Notice> JobBook::settle(const std::vector<core::JobDecision>& decisions) {
  std::vector<Notice> notices;
  for (const core::JobDecision& decision : decisions) {
    const auto pending = std::ranges::find(pending_, decision.job, &core::Job::id);
    if (pending == pending_.end()) {
      continue;
    }
    if (decision.decision.status == core::DecisionStatus::kDeferred) {
      const auto [reported, first_report] =
          reported_deferrals_.try_emplace(decision.job, decision.decision.reason);
      if (!first_report && reported->second == decision.decision.reason) {
        continue;
      }
      reported->second = decision.decision.reason;
      notices.push_back(Notice{.job = *pending, .decision = decision});
      continue;
    }
    notices.push_back(Notice{.job = *pending, .decision = decision});
    close_group(*pending);
    decided_.insert(decision.job);
    if (decision.decision.status == core::DecisionStatus::kAdmitted) {
      admitted_.insert(decision.job);
    }
    reported_deferrals_.erase(decision.job);
    pending_.erase(pending);
  }
  return notices;
}

}  // namespace rovi::scheduler
