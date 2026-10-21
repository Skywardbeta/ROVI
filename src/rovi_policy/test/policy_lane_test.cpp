#include "rovi_policy/policy_lane.hpp"

#include <cstdint>
#include <optional>

#include <gtest/gtest.h>

#include "rovi_core/units.hpp"
#include "rovi_policy/wire.hpp"

namespace rovi::policy {
namespace {

core::TimePoint at_ms(std::int64_t ms) { return core::TimePoint::from_ns(ms * 1'000'000); }

LaneJob job(std::uint64_t id, std::int64_t start_ms, std::int64_t end_ms) {
  return LaneJob{.job = core::JobId{id},
                 .robot = core::RobotId{0},
                 .planned_start = at_ms(start_ms),
                 .planned_end = at_ms(end_ms),
                 .chunk_bytes = core::Bytes{1'280}};
}

Observation frame() {
  return Observation{.state = {1.0F}, .width = 1, .height = 1, .rgb = {7, 8, 9}};
}

PolicyLane lane() {
  return *PolicyLane::create(LaneTiming{.orphan_timeout = core::Duration::from_ns(500'000'000),
                                        .max_lead = core::Duration::from_ns(2'000'000'000)});
}

TEST(PolicyLane, StartsAJobOnlyAtItsReservedStartOnceItsObservationArrived) {
  PolicyLane policy = lane();
  ASSERT_TRUE(policy.assign(job(1, 100, 1'100), at_ms(0)));
  EXPECT_FALSE(policy.start(at_ms(100)));
  ASSERT_TRUE(policy.arrive(core::JobId{1}, at_ms(90), at_ms(10), frame()));
  EXPECT_FALSE(policy.start(at_ms(99)));
  const std::optional<Started> started = policy.start(at_ms(100));
  ASSERT_TRUE(started.has_value());
  EXPECT_EQ(started->job, job(1, 100, 1'100));
  EXPECT_EQ(started->observation_time, at_ms(10));
  EXPECT_EQ(started->observation, frame());
  EXPECT_TRUE(policy.busy());
}

TEST(PolicyLane, RunsOneJobAtATimeInReservedStartOrder) {
  PolicyLane policy = lane();
  ASSERT_TRUE(policy.assign(job(2, 300, 1'300), at_ms(0)));
  ASSERT_TRUE(policy.assign(job(1, 200, 1'200), at_ms(0)));
  ASSERT_TRUE(policy.arrive(core::JobId{2}, at_ms(0), at_ms(0), frame()));
  ASSERT_TRUE(policy.arrive(core::JobId{1}, at_ms(0), at_ms(0), frame()));
  EXPECT_EQ(policy.start(at_ms(400))->job.job, core::JobId{1});
  EXPECT_FALSE(policy.start(at_ms(400)));
  EXPECT_FALSE(policy.finish(core::JobId{2}));
  EXPECT_TRUE(policy.finish(core::JobId{1}));
  EXPECT_EQ(policy.start(at_ms(400))->job.job, core::JobId{2});
}

TEST(PolicyLane, DropsAReadyJobWhoseReservedComputeHasEnded) {
  PolicyLane policy = lane();
  ASSERT_TRUE(policy.assign(job(1, 100, 1'100), at_ms(0)));
  ASSERT_TRUE(policy.arrive(core::JobId{1}, at_ms(50), at_ms(0), frame()));
  policy.prune(at_ms(1'100));
  EXPECT_EQ(policy.discarded(), 0U);
  policy.prune(at_ms(1'101));
  EXPECT_EQ(policy.discarded(), 1U);
  EXPECT_FALSE(policy.start(at_ms(1'101)));
}

TEST(PolicyLane, DropsOrphanedAssignmentsAndArrivals) {
  PolicyLane policy = lane();
  ASSERT_TRUE(policy.assign(job(1, 100, 5'000), at_ms(0)));
  ASSERT_TRUE(policy.arrive(core::JobId{2}, at_ms(100), at_ms(0), frame()));
  policy.prune(at_ms(600));
  EXPECT_EQ(policy.discarded(), 0U);
  policy.prune(at_ms(601));
  EXPECT_EQ(policy.discarded(), 2U);
}

TEST(PolicyLane, AcceptsAnObservationBeforeItsDecision) {
  PolicyLane policy = lane();
  ASSERT_TRUE(policy.arrive(core::JobId{1}, at_ms(0), at_ms(0), frame()));
  EXPECT_FALSE(policy.arrive(core::JobId{1}, at_ms(1), at_ms(0), frame()));
  ASSERT_TRUE(policy.assign(job(1, 10, 100), at_ms(5)));
  EXPECT_TRUE(policy.start(at_ms(10)).has_value());
}

TEST(PolicyLane, RefusesAJobBeyondTheMaximumLeadAnInvertedSlotOrADuplicate) {
  PolicyLane policy = lane();
  EXPECT_FALSE(policy.assign(job(1, 2'001, 3'000), at_ms(0)));
  EXPECT_FALSE(policy.assign(job(2, 200, 100), at_ms(0)));
  EXPECT_TRUE(policy.assign(job(3, 100, 200), at_ms(0)));
  EXPECT_FALSE(policy.assign(job(3, 100, 200), at_ms(0)));
}

TEST(PolicyLane, CreateRejectsInvalidTiming) {
  EXPECT_FALSE(
      PolicyLane::create({.orphan_timeout = core::Duration{}, .max_lead = core::Duration{}}));
  EXPECT_FALSE(PolicyLane::create(
      {.orphan_timeout = core::Duration::from_ns(1), .max_lead = core::Duration::from_ns(-1)}));
}

}  // namespace
}  // namespace rovi::policy
