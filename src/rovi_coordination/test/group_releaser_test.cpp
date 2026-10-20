#include "rovi_coordination/group_releaser.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "rovi_coordination/group.hpp"
#include "rovi_core/admission.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/units.hpp"

#include "test_jobs.hpp"

namespace rovi::coordination {
namespace {

constexpr std::size_t kCapacity = 4;
constexpr core::Duration kNoGrace{};

AssembledGroup group_of(std::uint32_t group, const std::vector<std::uint64_t>& jobs,
                        std::int64_t deadline_ms = 500) {
  AssembledGroup assembled{.workcell = 2, .group = core::GroupId{group}, .members = {}};
  for (const std::uint64_t job : jobs) {
    core::Job member = proposal(job, static_cast<std::uint32_t>(job % 100), deadline_ms);
    member.synchronization_group = core::GroupId{group};
    member.maximum_result_ready_skew = ms(50);
    assembled.members.push_back(member);
  }
  return assembled;
}

TEST(GroupReleaser, ReleasesAGroupOnceEveryChunkArrivesWithinTheSkewBound) {
  GroupReleaser releaser{kCapacity, kNoGrace};
  ASSERT_TRUE(releaser.track(group_of(7, {101, 102, 103})));

  EXPECT_EQ(releaser.on_delivery(core::JobId{102}, at_ms(120)), std::nullopt);
  EXPECT_EQ(releaser.on_delivery(core::JobId{101}, at_ms(100)), std::nullopt);
  const std::optional<ReleaseOutcome> outcome = releaser.on_delivery(core::JobId{103}, at_ms(130));

  const ReleaseOutcome expected{.workcell = 2,
                                .group = core::GroupId{7},
                                .released = true,
                                .reason = core::ReasonCode::kNone,
                                .release_time = at_ms(130),
                                .skew = ms(30),
                                .jobs = {core::JobId{101}, core::JobId{102}, core::JobId{103}},
                                .robots = {core::RobotId{1}, core::RobotId{2}, core::RobotId{3}}};
  EXPECT_EQ(outcome, expected);
  EXPECT_EQ(releaser.tracked(), 0U);
}

TEST(GroupReleaser, RefusesAGroupWhoseChunksSpreadBeyondTheSkewBound) {
  GroupReleaser releaser{kCapacity, kNoGrace};
  ASSERT_TRUE(releaser.track(group_of(7, {101, 102})));
  static_cast<void>(releaser.on_delivery(core::JobId{101}, at_ms(100)));

  const std::optional<ReleaseOutcome> outcome = releaser.on_delivery(core::JobId{102}, at_ms(200));

  ASSERT_TRUE(outcome.has_value());
  EXPECT_FALSE(outcome->released);
  EXPECT_EQ(outcome->reason, core::ReasonCode::kGroupSkewExceeded);
  EXPECT_EQ(outcome->skew, ms(100));
  EXPECT_EQ(outcome->release_time, at_ms(200));
}

TEST(GroupReleaser, StopsTrackingAGroupOnceTheSchedulerRejectsAMember) {
  GroupReleaser releaser{kCapacity, kNoGrace};
  ASSERT_TRUE(releaser.track(group_of(7, {101, 102})));

  releaser.on_decision(core::JobId{101}, core::DecisionStatus::kDeferred, core::ReasonCode::kNone);
  releaser.on_decision(core::JobId{102}, core::DecisionStatus::kAdmitted, core::ReasonCode::kNone);
  EXPECT_EQ(releaser.tracked(), 1U);
  releaser.on_decision(core::JobId{101}, core::DecisionStatus::kRejected, core::ReasonCode::kNone);
  EXPECT_EQ(releaser.tracked(), 0U);
  EXPECT_EQ(releaser.on_delivery(core::JobId{102}, at_ms(100)), std::nullopt);
}

TEST(GroupReleaser, RefusesAGroupWithAMissingChunkOnceItsEarliestDeadlinePasses) {
  GroupReleaser releaser{kCapacity, kNoGrace};
  ASSERT_TRUE(releaser.track(group_of(7, {101, 102}, 300)));
  static_cast<void>(releaser.on_delivery(core::JobId{101}, at_ms(100)));

  EXPECT_TRUE(releaser.expire(at_ms(299)).empty());
  const std::vector<ReleaseOutcome> expired = releaser.expire(at_ms(300));

  ASSERT_EQ(expired.size(), 1U);
  EXPECT_FALSE(expired.front().released);
  EXPECT_EQ(expired.front().reason, core::ReasonCode::kGroupMemberUnavailable);
  EXPECT_EQ(expired.front().release_time, at_ms(300));
  EXPECT_EQ(expired.front().jobs, (std::vector<core::JobId>{core::JobId{101}, core::JobId{102}}));
  EXPECT_EQ(releaser.tracked(), 0U);
}

TEST(GroupReleaser, IgnoresUnknownAndRepeatedDeliveries) {
  GroupReleaser releaser{kCapacity, kNoGrace};
  ASSERT_TRUE(releaser.track(group_of(7, {101, 102})));

  EXPECT_EQ(releaser.on_delivery(core::JobId{999}, at_ms(100)), std::nullopt);
  EXPECT_EQ(releaser.on_delivery(core::JobId{101}, at_ms(100)), std::nullopt);
  EXPECT_EQ(releaser.on_delivery(core::JobId{101}, at_ms(400)), std::nullopt);
  const std::optional<ReleaseOutcome> outcome = releaser.on_delivery(core::JobId{102}, at_ms(120));

  ASSERT_TRUE(outcome.has_value());
  EXPECT_TRUE(outcome->released);
  EXPECT_EQ(outcome->skew, ms(20));
}

TEST(GroupReleaser, RefusesToTrackBeyondCapacityOrTheSameGroupTwice) {
  GroupReleaser releaser{1, kNoGrace};

  EXPECT_TRUE(releaser.track(group_of(7, {101, 102})));
  EXPECT_FALSE(releaser.track(group_of(7, {101, 102})));
  EXPECT_FALSE(releaser.track(group_of(8, {103, 104})));
  EXPECT_FALSE(GroupReleaser(kCapacity, kNoGrace).track(AssembledGroup{}));
}

TEST(GroupReleaser, WaitsAGracePeriodForAChunkDeliveredByTheDeadline) {
  GroupReleaser releaser{kCapacity, ms(50)};
  ASSERT_TRUE(releaser.track(group_of(7, {101, 102}, 300)));
  ASSERT_TRUE(releaser.track(group_of(8, {103, 104}, 300)));
  static_cast<void>(releaser.on_delivery(core::JobId{101}, at_ms(280)));
  static_cast<void>(releaser.on_delivery(core::JobId{103}, at_ms(280)));

  EXPECT_TRUE(releaser.expire(at_ms(320)).empty());
  const std::optional<ReleaseOutcome> released = releaser.on_delivery(core::JobId{102}, at_ms(300));
  ASSERT_TRUE(released.has_value());
  EXPECT_TRUE(released->released);
  EXPECT_TRUE(releaser.expire(at_ms(349)).empty());
  const std::vector<ReleaseOutcome> expired = releaser.expire(at_ms(350));

  ASSERT_EQ(expired.size(), 1U);
  EXPECT_EQ(expired.front().group, core::GroupId{8});
}

TEST(GroupReleaser, RefusesAGroupDeliveredAfterItsDeadline) {
  GroupReleaser releaser{kCapacity, ms(50)};
  ASSERT_TRUE(releaser.track(group_of(7, {101, 102}, 300)));
  static_cast<void>(releaser.on_delivery(core::JobId{101}, at_ms(290)));

  const std::optional<ReleaseOutcome> outcome = releaser.on_delivery(core::JobId{102}, at_ms(310));

  ASSERT_TRUE(outcome.has_value());
  EXPECT_FALSE(outcome->released);
  EXPECT_EQ(outcome->reason, core::ReasonCode::kDeadlineMiss);
}

TEST(GroupReleaser, KeepsTrackingWhenTheSchedulerRejectsADuplicateRequest) {
  GroupReleaser releaser{kCapacity, kNoGrace};
  ASSERT_TRUE(releaser.track(group_of(7, {101, 102})));

  releaser.on_decision(core::JobId{101}, core::DecisionStatus::kRejected,
                       core::ReasonCode::kDuplicateJob);

  EXPECT_EQ(releaser.tracked(), 1U);
}

TEST(GroupReleaser, RefusesAGroupWhoseSkewCannotBeRepresented) {
  GroupReleaser releaser{kCapacity, kNoGrace};
  AssembledGroup group = group_of(7, {101, 102});
  for (core::Job& member : group.members) {
    member.buffer_empty_time = core::TimePoint::from_ns(std::numeric_limits<std::int64_t>::max());
  }
  ASSERT_TRUE(releaser.track(group));
  static_cast<void>(releaser.on_delivery(
      core::JobId{101}, core::TimePoint::from_ns(std::numeric_limits<std::int64_t>::min())));

  const std::optional<ReleaseOutcome> outcome = releaser.on_delivery(core::JobId{102}, at_ms(0));

  ASSERT_TRUE(outcome.has_value());
  EXPECT_FALSE(outcome->released);
  EXPECT_EQ(outcome->reason, core::ReasonCode::kGroupSkewExceeded);
}

}  // namespace
}  // namespace rovi::coordination
