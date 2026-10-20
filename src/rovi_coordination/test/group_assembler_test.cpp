#include "rovi_coordination/group_assembler.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "rovi_coordination/group.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/units.hpp"
#include "rovi_scheduler/job_book.hpp"

#include "test_jobs.hpp"

namespace rovi::coordination {
namespace {

Workcell cell(std::uint32_t id, const std::vector<std::uint32_t>& robots) {
  Workcell workcell{.id = id, .members = {}, .skew = ms(50)};
  for (const std::uint32_t robot : robots) {
    workcell.members.emplace_back(robot);
  }
  return workcell;
}

GroupAssembler two_workcells() {
  return GroupAssembler::create({cell(0, {0, 1, 2}), cell(1, {3, 4, 5})}).value();
}

core::Job grouped(core::Job job, std::uint32_t group, std::int64_t group_deadline_ms = 500) {
  job.synchronization_group = core::GroupId{group};
  job.maximum_result_ready_skew = ms(50);
  job.task_deadline = at_ms(group_deadline_ms);
  return job;
}

TEST(GroupAssembler, HoldsProposalsUntilEveryMemberHasProposed) {
  GroupAssembler assembler = two_workcells();

  EXPECT_EQ(assembler.propose(proposal(12, 2, 500)).status, ProposalStatus::kHeld);
  EXPECT_EQ(assembler.propose(proposal(10, 0, 500)).status, ProposalStatus::kHeld);
  const ProposalOutcome outcome = assembler.propose(proposal(11, 1, 500));

  const AssembledGroup expected{
      .workcell = 0,
      .group = core::GroupId{1},
      .members = {grouped(proposal(10, 0, 500), 1), grouped(proposal(11, 1, 500), 1),
                  grouped(proposal(12, 2, 500), 1)}};
  EXPECT_EQ(outcome, (ProposalOutcome{.status = ProposalStatus::kAssembled, .group = expected}));
}

TEST(GroupAssembler, AssignsAFreshGroupIdToEveryRoundInEveryWorkcell) {
  GroupAssembler assembler = two_workcells();
  std::vector<std::uint32_t> groups;
  for (const std::uint32_t robot : {0U, 3U, 1U, 4U, 2U, 5U, 0U, 1U, 2U}) {
    const ProposalOutcome outcome =
        assembler.propose(proposal(100 + groups.size() + robot, robot, 500));
    if (outcome.group) {
      groups.push_back(outcome.group->group.value());
      EXPECT_EQ(outcome.group->workcell, robot < 3 ? 0U : 1U);
    }
  }

  EXPECT_EQ(groups, (std::vector<std::uint32_t>{1, 2, 3}));
}

TEST(GroupAssembler, ALaterProposalReplacesAnEarlierOneFromTheSameRobot) {
  GroupAssembler assembler = two_workcells();
  static_cast<void>(assembler.propose(proposal(10, 0, 500)));
  static_cast<void>(assembler.propose(proposal(20, 0, 600)));
  static_cast<void>(assembler.propose(proposal(11, 1, 500)));
  const ProposalOutcome outcome = assembler.propose(proposal(12, 2, 500));

  ASSERT_TRUE(outcome.group.has_value());
  EXPECT_EQ(outcome.group->members.front().id, core::JobId{20});
  EXPECT_EQ(outcome.group->members.size(), 3U);
}

TEST(GroupAssembler, IgnoresUnknownRobotsAndProposalsThatAlreadyCarryAGroup) {
  GroupAssembler assembler = two_workcells();

  EXPECT_EQ(assembler.propose(proposal(10, 9, 500)).status, ProposalStatus::kUnknownRobot);
  EXPECT_EQ(assembler.propose(grouped(proposal(11, 0, 500), 4, 500)).status,
            ProposalStatus::kAlreadyGrouped);
  static_cast<void>(assembler.propose(proposal(12, 1, 500)));
  static_cast<void>(assembler.propose(proposal(13, 2, 500)));
  EXPECT_EQ(assembler.propose(proposal(14, 0, 500)).status, ProposalStatus::kAssembled);
}

TEST(GroupAssembler, DropsAnIncompleteRoundOnceItsEarliestDeadlinePasses) {
  GroupAssembler assembler = two_workcells();
  static_cast<void>(assembler.propose(proposal(10, 0, 100)));
  static_cast<void>(assembler.propose(proposal(11, 1, 50)));
  static_cast<void>(assembler.propose(proposal(13, 3, 400)));

  EXPECT_TRUE(assembler.expire(at_ms(40)).empty());
  const std::vector<AssembledGroup> dropped = assembler.expire(at_ms(50));

  const std::vector<AssembledGroup> expected{
      AssembledGroup{.workcell = 0,
                     .group = core::GroupId{0},
                     .members = {proposal(10, 0, 100), proposal(11, 1, 50)}}};
  EXPECT_EQ(dropped, expected);
  EXPECT_EQ(assembler.propose(proposal(12, 2, 500)).status, ProposalStatus::kHeld);
}

TEST(GroupAssembler, DropsARoundWhoseDeadlineCannotBeRepresented) {
  GroupAssembler assembler = two_workcells();
  core::Job unbounded = proposal(10, 0, 100);
  unbounded.buffer_empty_time = core::TimePoint::from_ns(std::numeric_limits<std::int64_t>::min());
  unbounded.safety_margin = ms(1);
  static_cast<void>(assembler.propose(unbounded));

  EXPECT_EQ(assembler.expire(at_ms(0)).size(), 1U);
}

TEST(GroupAssembler, CreateRejectsAnInvalidWorkcellList) {
  Workcell oversized = cell(2, {});
  for (std::uint32_t robot = 0; robot <= kMaximumWorkcellSize; ++robot) {
    oversized.members.emplace_back(100 + robot);
  }
  Workcell negative = cell(2, {7});
  negative.skew = ms(-1);

  EXPECT_FALSE(GroupAssembler::create({}).has_value());
  EXPECT_FALSE(GroupAssembler::create({cell(0, {})}).has_value());
  EXPECT_FALSE(GroupAssembler::create({cell(0, {0, 1}), cell(1, {1, 2})}).has_value());
  EXPECT_FALSE(GroupAssembler::create({cell(0, {0, 0})}).has_value());
  EXPECT_FALSE(GroupAssembler::create({cell(0, {0}), cell(0, {1})}).has_value());
  EXPECT_FALSE(GroupAssembler::create({negative}).has_value());
  EXPECT_FALSE(GroupAssembler::create({oversized}).has_value());
  EXPECT_TRUE(GroupAssembler::create({cell(0, {0}), cell(1, {1})}).has_value());
}

TEST(GroupAssembler, GivesEveryMemberTheEarliestMemberDeadline) {
  GroupAssembler assembler = two_workcells();
  core::Job fresh = proposal(11, 1, 900);
  fresh.maximum_result_age = ms(300);
  static_cast<void>(assembler.propose(proposal(10, 0, 700)));
  static_cast<void>(assembler.propose(fresh));
  const ProposalOutcome outcome = assembler.propose(proposal(12, 2, 600));

  ASSERT_TRUE(outcome.group.has_value());
  for (const core::Job& member : outcome.group->members) {
    EXPECT_EQ(member.task_deadline, at_ms(300)) << member.id.value();
    EXPECT_EQ(core::effective_deadline(member), at_ms(300)) << member.id.value();
  }
}

TEST(GroupAssembler, RefusesProposalsWhoseListsExceedTheSchedulerLimit) {
  GroupAssembler assembler = two_workcells();
  core::Job many_workers = proposal(10, 0, 500);
  many_workers.eligible_workers.assign(scheduler::kMaximumRequestListLength + 1, core::WorkerId{1});
  core::Job many_predecessors = proposal(11, 1, 500);
  many_predecessors.predecessors.assign(scheduler::kMaximumRequestListLength + 1, core::JobId{1});

  EXPECT_EQ(assembler.propose(many_workers).status, ProposalStatus::kMalformed);
  EXPECT_EQ(assembler.propose(many_predecessors).status, ProposalStatus::kMalformed);
  static_cast<void>(assembler.propose(proposal(12, 1, 500)));
  static_cast<void>(assembler.propose(proposal(13, 2, 500)));
  EXPECT_EQ(assembler.propose(proposal(14, 0, 500)).status, ProposalStatus::kAssembled);
}

}  // namespace
}  // namespace rovi::coordination
