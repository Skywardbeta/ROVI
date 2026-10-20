#include "rovi_core/scheduler.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <map>
#include <optional>
#include <ranges>
#include <tuple>
#include <utility>
#include <vector>

#include "rovi_core/admission.hpp"
#include "rovi_core/capacity_calendar.hpp"
#include "rovi_core/digest.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/units.hpp"

namespace rovi::core {
namespace {

using Members = std::vector<std::size_t>;

struct Partition {
  std::vector<Members> units;
  std::vector<JobDecision> decided;
};

struct UnitPlan {
  std::vector<JobDecision> decisions;
  std::optional<ResourceSnapshot> snapshot;
  Duration slack;
  TimePoint deadline;
};

struct RankedUnit {
  Members members;
  Duration slack;
  TimePoint deadline;
  TimePoint release;
  std::uint64_t sequence{0};
  JobId first;
};

void add_transfer(Fnv1aDigest& digest, const Transfer& transfer) {
  digest.add(transfer.slices.size());
  for (const ServiceSlice& slice : transfer.slices) {
    digest.add(slice.slot.value());
    digest.add(slice.bytes.value());
  }
  digest.add(transfer.completion.ns());
}

bool precedes(const Job& lhs, const Job& rhs) {
  return std::tie(lhs.sequence, lhs.id) < std::tie(rhs.sequence, rhs.id);
}

JobDecision decided(const Job& job, DecisionStatus status, ReasonCode reason) {
  return JobDecision{
      .job = job.id, .decision = {.status = status, .reason = reason}, .reservation = std::nullopt};
}

std::optional<ReasonCode> blocking_reason(const Job& job,
                                          const std::map<JobId, std::size_t>& id_counts,
                                          const std::vector<JobId>& completed) {
  if (id_counts.at(job.id) > 1) {
    return ReasonCode::kDuplicateJob;
  }
  const bool ready = std::ranges::all_of(job.predecessors, [&completed](JobId predecessor) {
    return std::ranges::find(completed, predecessor) != completed.end();
  });
  return ready ? std::nullopt : std::optional{ReasonCode::kPredecessorIncomplete};
}

JobDecision blocked(const Job& job, ReasonCode reason) {
  const DecisionStatus status =
      reason == ReasonCode::kDuplicateJob ? DecisionStatus::kRejected : DecisionStatus::kDeferred;
  return decided(job, status, reason);
}

Partition partition_units(const std::vector<Job>& pending, const std::vector<JobId>& completed) {
  std::map<JobId, std::size_t> id_counts;
  for (const Job& job : pending) {
    ++id_counts[job.id];
  }
  const auto reason_for = [&](std::size_t index) {
    return blocking_reason(pending.at(index), id_counts, completed);
  };
  std::map<GroupId, Members> groups;
  Partition partition;
  for (std::size_t index = 0; index < pending.size(); ++index) {
    const Job& job = pending.at(index);
    if (job.synchronization_group) {
      groups[*job.synchronization_group].push_back(index);
    } else if (const std::optional<ReasonCode> reason = reason_for(index)) {
      partition.decided.push_back(blocked(job, *reason));
    } else {
      partition.units.push_back({index});
    }
  }
  for (Members& members : groups | std::views::values) {
    std::ranges::sort(members, [&pending](std::size_t lhs, std::size_t rhs) {
      return precedes(pending.at(lhs), pending.at(rhs));
    });
    if (std::ranges::none_of(members,
                             [&](std::size_t index) { return reason_for(index).has_value(); })) {
      partition.units.push_back(members);
      continue;
    }
    for (const std::size_t index : members) {
      const std::optional<ReasonCode> reason = reason_for(index);
      partition.decided.push_back(reason ? blocked(pending.at(index), *reason)
                                         : decided(pending.at(index), DecisionStatus::kDeferred,
                                                   ReasonCode::kGroupMemberUnavailable));
    }
  }
  return partition;
}

std::vector<JobDecision> unit_decisions(const std::vector<Job>& pending, const Members& members,
                                        std::optional<std::size_t> failing, Decision decision) {
  std::vector<JobDecision> decisions;
  for (const std::size_t index : members) {
    const Decision outcome = !failing || index == *failing
                                 ? decision
                                 : Decision{.status = DecisionStatus::kDeferred,
                                            .reason = ReasonCode::kGroupMemberUnavailable};
    decisions.push_back(
        JobDecision{.job = pending.at(index).id, .decision = outcome, .reservation = std::nullopt});
  }
  return decisions;
}

bool skew_exceeded(const std::vector<Job>& pending, const Members& members,
                   const std::vector<ChainReservation>& chains) {
  if (chains.size() < 2) {
    return false;
  }
  const auto [earliest, latest] =
      std::ranges::minmax(chains | std::views::transform(&ChainReservation::delivery));
  const Duration bound = std::ranges::min(members | std::views::transform([&](std::size_t index) {
                                            return pending.at(index).maximum_result_ready_skew;
                                          }));
  const std::optional<Duration> spread = elapsed(earliest, latest);
  return !spread || *spread > bound;
}

UnitPlan plan_unit(const ResourceSnapshot& snapshot, const std::vector<Job>& pending,
                   const Members& members, TimePoint now) {
  ResourceSnapshot trial = snapshot;
  std::vector<ChainReservation> chains;
  for (const std::size_t index : members) {
    AdmissionResult result = admit(trial, pending.at(index), now);
    if (result.decision.status != DecisionStatus::kAdmitted || !result.reservation) {
      return UnitPlan{.decisions = unit_decisions(pending, members, index, result.decision),
                      .snapshot = std::nullopt,
                      .slack = {},
                      .deadline = {}};
    }
    chains.push_back(std::move(*result.reservation));
    trial = std::move(result.snapshot);
  }
  if (skew_exceeded(pending, members, chains)) {
    const Decision skew{.status = DecisionStatus::kDeferred,
                        .reason = ReasonCode::kGroupSkewExceeded};
    return UnitPlan{.decisions = unit_decisions(pending, members, std::nullopt, skew),
                    .snapshot = std::nullopt,
                    .slack = {},
                    .deadline = {}};
  }
  UnitPlan plan{.decisions = {},
                .snapshot = std::move(trial),
                .slack = chains.front().slack,
                .deadline = chains.front().deadline};
  for (ChainReservation& chain : chains) {
    plan.slack = std::min(plan.slack, chain.slack);
    plan.deadline = std::min(plan.deadline, chain.deadline);
    plan.decisions.push_back(
        JobDecision{.job = chain.job,
                    .decision = {.status = DecisionStatus::kAdmitted, .reason = ReasonCode::kNone},
                    .reservation = std::move(chain)});
  }
  return plan;
}

RankedUnit rank(const Members& members, const UnitPlan& plan, const std::vector<Job>& pending) {
  const Job& first = pending.at(members.front());
  const TimePoint release =
      std::ranges::min(members | std::views::transform([&](std::size_t index) {
                         return pending.at(index).release_time;
                       }));
  return RankedUnit{.members = members,
                    .slack = plan.slack,
                    .deadline = plan.deadline,
                    .release = release,
                    .sequence = first.sequence,
                    .first = first.id};
}

bool ranks_before(Policy policy, const RankedUnit& lhs, const RankedUnit& rhs) {
  switch (policy) {
    case Policy::kLeastSlack:
      return std::tie(lhs.slack, lhs.deadline, lhs.sequence, lhs.first) <
             std::tie(rhs.slack, rhs.deadline, rhs.sequence, rhs.first);
    case Policy::kEarliestDeadline:
      return std::tie(lhs.deadline, lhs.sequence, lhs.first) <
             std::tie(rhs.deadline, rhs.sequence, rhs.first);
    case Policy::kFirstInFirstOut:
      return std::tie(lhs.release, lhs.sequence, lhs.first) <
             std::tie(rhs.release, rhs.sequence, rhs.first);
  }
  return false;
}

ScheduleResult finish(const std::vector<Job>& pending, ResourceSnapshot snapshot,
                      std::vector<JobDecision> decisions) {
  std::map<JobId, std::uint64_t> sequence_of;
  for (const Job& job : pending) {
    const auto [entry, inserted] = sequence_of.emplace(job.id, job.sequence);
    entry->second = inserted ? entry->second : std::min(entry->second, job.sequence);
  }
  std::ranges::stable_sort(decisions, [&sequence_of](const JobDecision& lhs,
                                                     const JobDecision& rhs) {
    return std::tie(sequence_of.at(lhs.job), lhs.job) < std::tie(sequence_of.at(rhs.job), rhs.job);
  });
  const std::uint64_t digest = decision_digest(decisions);
  const ServiceSource source = snapshot.source;
  return ScheduleResult{.decisions = std::move(decisions),
                        .snapshot = std::move(snapshot),
                        .source = source,
                        .digest = digest};
}

}  // namespace

ScheduleResult schedule(const ResourceSnapshot& snapshot, const std::vector<Job>& pending,
                        const std::vector<JobId>& completed, TimePoint now, Policy policy) {
  if (!is_consistent(snapshot)) {
    std::vector<JobDecision> decisions;
    decisions.reserve(pending.size());
    std::ranges::transform(pending, std::back_inserter(decisions), [](const Job& job) {
      return decided(job, DecisionStatus::kRejected, ReasonCode::kInvalidSnapshot);
    });
    return finish(pending, snapshot, std::move(decisions));
  }
  Partition partition = partition_units(pending, completed);
  std::vector<JobDecision> decisions = std::move(partition.decided);
  std::vector<RankedUnit> ranked;
  for (const Members& members : partition.units) {
    const UnitPlan plan = plan_unit(snapshot, pending, members, now);
    if (plan.snapshot) {
      ranked.push_back(rank(members, plan, pending));
    } else {
      decisions.insert(decisions.end(), plan.decisions.begin(), plan.decisions.end());
    }
  }
  std::ranges::sort(ranked, [policy](const RankedUnit& lhs, const RankedUnit& rhs) {
    return ranks_before(policy, lhs, rhs);
  });
  ResourceSnapshot working = snapshot;
  for (const RankedUnit& unit : ranked) {
    UnitPlan plan = plan_unit(working, pending, unit.members, now);
    decisions.insert(decisions.end(), plan.decisions.begin(), plan.decisions.end());
    if (plan.snapshot) {
      working = std::move(*plan.snapshot);
    }
  }
  return finish(pending, std::move(working), std::move(decisions));
}

std::uint64_t decision_digest(const std::vector<JobDecision>& decisions) {
  Fnv1aDigest digest;
  for (const JobDecision& decision : decisions) {
    digest.add(decision.job.value());
    digest.add(static_cast<std::uint64_t>(decision.decision.status));
    digest.add(static_cast<std::uint64_t>(decision.decision.reason));
    if (!decision.reservation) {
      digest.add(std::uint64_t{0});
      continue;
    }
    const ChainReservation& chain = *decision.reservation;
    digest.add(std::uint64_t{1});
    digest.add(chain.worker.value());
    add_transfer(digest, chain.upload);
    digest.add(chain.compute.start.ns());
    digest.add(chain.compute.end.ns());
    add_transfer(digest, chain.download);
    digest.add(chain.deadline.ns());
    digest.add(chain.slack.ns());
    digest.add(chain.delivery.ns());
  }
  return digest.value();
}

}  // namespace rovi::core
