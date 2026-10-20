#include "rovi_coordination/group_releaser.hpp"

#include <algorithm>
#include <cstddef>
#include <map>
#include <optional>
#include <ranges>
#include <vector>

#include "rovi_coordination/group.hpp"
#include "rovi_core/admission.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/units.hpp"

namespace rovi::coordination {
namespace {

bool contains(const AssembledGroup& group, core::JobId job) {
  return std::ranges::find(group.members, job, &core::Job::id) != group.members.end();
}

bool past_deadline(const AssembledGroup& group, core::Duration grace, core::TimePoint now) {
  return std::ranges::any_of(group.members, [grace, now](const core::Job& member) {
    const std::optional<core::TimePoint> deadline = core::effective_deadline(member);
    const std::optional<core::TimePoint> limit =
        deadline ? core::checked_add(*deadline, grace) : std::nullopt;
    return !limit || *limit <= now;
  });
}

std::optional<core::Duration> spread(const std::map<core::JobId, core::TimePoint>& delivered) {
  if (delivered.empty()) {
    return core::Duration{};
  }
  const auto [earliest, latest] = std::ranges::minmax(delivered | std::views::values);
  return core::elapsed(earliest, latest);
}

std::optional<core::TimePoint> group_deadline(const AssembledGroup& group) {
  std::optional<core::TimePoint> earliest;
  for (const core::Job& member : group.members) {
    const std::optional<core::TimePoint> deadline = core::effective_deadline(member);
    if (!deadline) {
      return std::nullopt;
    }
    earliest = earliest ? std::min(*earliest, *deadline) : *deadline;
  }
  return earliest;
}

core::Duration bound(const AssembledGroup& group) {
  return std::ranges::min(group.members | std::views::transform([](const core::Job& member) {
                            return member.maximum_result_ready_skew;
                          }));
}

ReleaseOutcome outcome_for(const AssembledGroup& group, core::ReasonCode reason,
                           core::TimePoint release_time, core::Duration skew) {
  ReleaseOutcome outcome{.workcell = group.workcell,
                         .group = group.group,
                         .released = reason == core::ReasonCode::kNone,
                         .reason = reason,
                         .release_time = release_time,
                         .skew = skew,
                         .jobs = {},
                         .robots = {}};
  for (const core::Job& member : group.members) {
    outcome.jobs.push_back(member.id);
    outcome.robots.push_back(member.robot);
  }
  return outcome;
}

}  // namespace

GroupReleaser::GroupReleaser(std::size_t max_tracked, core::Duration grace)
    : max_tracked_{max_tracked}, grace_{grace} {}

bool GroupReleaser::track(const AssembledGroup& group) {
  const bool known = std::ranges::any_of(
      groups_, [&group](const Tracked& tracked) { return tracked.group.group == group.group; });
  if (group.members.empty() || known || groups_.size() >= max_tracked_) {
    return false;
  }
  groups_.push_back(Tracked{.group = group, .delivered = {}});
  return true;
}

void GroupReleaser::on_decision(core::JobId job, core::DecisionStatus status,
                                core::ReasonCode reason) {
  if (status != core::DecisionStatus::kRejected || reason == core::ReasonCode::kDuplicateJob) {
    return;
  }
  std::erase_if(groups_, [job](const Tracked& tracked) { return contains(tracked.group, job); });
}

std::optional<ReleaseOutcome> GroupReleaser::on_delivery(core::JobId job,
                                                         core::TimePoint delivered) {
  const auto found = std::ranges::find_if(
      groups_, [job](const Tracked& candidate) { return contains(candidate.group, job); });
  if (found == groups_.end()) {
    return std::nullopt;
  }
  found->delivered.try_emplace(job, delivered);
  if (found->delivered.size() < found->group.members.size()) {
    return std::nullopt;
  }
  const std::optional<core::Duration> skew = spread(found->delivered);
  const core::TimePoint release_time = std::ranges::max(found->delivered | std::views::values);
  const std::optional<core::TimePoint> deadline = group_deadline(found->group);
  core::ReasonCode reason = core::ReasonCode::kNone;
  if (!deadline || release_time > *deadline) {
    reason = core::ReasonCode::kDeadlineMiss;
  } else if (!skew || *skew > bound(found->group)) {
    reason = core::ReasonCode::kGroupSkewExceeded;
  }
  ReleaseOutcome outcome =
      outcome_for(found->group, reason, release_time, skew.value_or(core::Duration{}));
  groups_.erase(found);
  return outcome;
}

std::vector<ReleaseOutcome> GroupReleaser::expire(core::TimePoint now) {
  std::vector<ReleaseOutcome> expired;
  std::erase_if(groups_, [this, now, &expired](const Tracked& tracked) {
    if (!past_deadline(tracked.group, grace_, now)) {
      return false;
    }
    expired.push_back(outcome_for(tracked.group, core::ReasonCode::kGroupMemberUnavailable, now,
                                  spread(tracked.delivered).value_or(core::Duration{})));
    return true;
  });
  return expired;
}

std::size_t GroupReleaser::tracked() const { return groups_.size(); }

}  // namespace rovi::coordination
