#include "rovi_coordination/group_assembler.hpp"

#include <algorithm>
#include <cstdint>
#include <map>
#include <optional>
#include <ranges>
#include <set>
#include <utility>
#include <vector>

#include "rovi_coordination/group.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/units.hpp"
#include "rovi_scheduler/job_book.hpp"

namespace rovi::coordination {
namespace {

bool valid(const std::vector<Workcell>& workcells) {
  std::set<std::uint32_t> ids;
  std::set<core::RobotId> robots;
  for (const Workcell& workcell : workcells) {
    if (workcell.members.empty() || workcell.members.size() > kMaximumWorkcellSize ||
        workcell.skew < core::Duration{} || !ids.insert(workcell.id).second) {
      return false;
    }
    if (std::ranges::any_of(workcell.members, [&robots](core::RobotId robot) {
          return !robots.insert(robot).second;
        })) {
      return false;
    }
  }
  return !workcells.empty();
}

bool expired(const std::map<core::RobotId, core::Job>& round, core::TimePoint now) {
  return std::ranges::any_of(round, [now](const auto& entry) {
    const std::optional<core::TimePoint> deadline = core::effective_deadline(entry.second);
    return !deadline || *deadline <= now;
  });
}

std::optional<core::TimePoint> group_deadline(const std::map<core::RobotId, core::Job>& round) {
  std::optional<core::TimePoint> earliest;
  for (const core::Job& member : round | std::views::values) {
    const std::optional<core::TimePoint> deadline = core::effective_deadline(member);
    if (deadline && (!earliest || *deadline < *earliest)) {
      earliest = deadline;
    }
  }
  return earliest;
}

}  // namespace

GroupAssembler::GroupAssembler(std::vector<Workcell> workcells)
    : workcells_{std::move(workcells)} {}

std::optional<GroupAssembler> GroupAssembler::create(std::vector<Workcell> workcells) {
  if (!valid(workcells)) {
    return std::nullopt;
  }
  return GroupAssembler{std::move(workcells)};
}

ProposalOutcome GroupAssembler::propose(const core::Job& job) {
  if (job.synchronization_group) {
    return ProposalOutcome{.status = ProposalStatus::kAlreadyGrouped, .group = std::nullopt};
  }
  if (job.eligible_workers.size() > scheduler::kMaximumRequestListLength ||
      job.predecessors.size() > scheduler::kMaximumRequestListLength) {
    return ProposalOutcome{.status = ProposalStatus::kMalformed, .group = std::nullopt};
  }
  const auto workcell = std::ranges::find_if(workcells_, [&job](const Workcell& candidate) {
    return std::ranges::find(candidate.members, job.robot) != candidate.members.end();
  });
  if (workcell == workcells_.end()) {
    return ProposalOutcome{.status = ProposalStatus::kUnknownRobot, .group = std::nullopt};
  }
  std::map<core::RobotId, core::Job>& round = rounds_[workcell->id];
  round.insert_or_assign(job.robot, job);
  if (round.size() < workcell->members.size()) {
    return ProposalOutcome{.status = ProposalStatus::kHeld, .group = std::nullopt};
  }
  AssembledGroup group{
      .workcell = workcell->id, .group = core::GroupId{next_group_}, .members = {}};
  ++next_group_;
  const std::optional<core::TimePoint> deadline = group_deadline(round);
  for (const core::RobotId robot : workcell->members) {
    core::Job member = round.at(robot);
    member.synchronization_group = group.group;
    member.maximum_result_ready_skew = workcell->skew;
    member.task_deadline = deadline ? deadline : member.task_deadline;
    group.members.push_back(std::move(member));
  }
  rounds_.erase(workcell->id);
  return ProposalOutcome{.status = ProposalStatus::kAssembled, .group = std::move(group)};
}

std::vector<AssembledGroup> GroupAssembler::expire(core::TimePoint now) {
  std::vector<AssembledGroup> dropped;
  std::erase_if(rounds_, [now, &dropped](const auto& entry) {
    const auto& [workcell, round] = entry;
    if (!expired(round, now)) {
      return false;
    }
    AssembledGroup group{.workcell = workcell, .group = core::GroupId{0}, .members = {}};
    for (const auto& member : round | std::views::values) {
      group.members.push_back(member);
    }
    dropped.push_back(std::move(group));
    return true;
  });
  return dropped;
}

}  // namespace rovi::coordination
