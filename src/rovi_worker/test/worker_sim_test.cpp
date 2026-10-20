#include "rovi_worker/worker_sim.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "rovi_core/admission.hpp"
#include "rovi_core/units.hpp"

namespace rovi::worker {
namespace {

constexpr std::int64_t kLongLead = std::numeric_limits<std::int64_t>::max();

core::TimePoint t(std::int64_t ns) { return core::TimePoint::from_ns(ns); }

core::Duration d(std::int64_t ns) { return core::Duration::from_ns(ns); }

core::ExecutionProfile model(std::uint32_t id, std::int64_t duration_ns) {
  return core::ExecutionProfile{.model = core::ModelClass{id}, .duration = d(duration_ns)};
}

WorkerTiming timing(std::int64_t slowdown_ns, std::int64_t orphan_timeout_ns,
                    std::int64_t max_lead_ns) {
  return WorkerTiming{.slowdown = d(slowdown_ns),
                      .orphan_timeout = d(orphan_timeout_ns),
                      .max_lead = d(max_lead_ns)};
}

WorkerSim worker(std::int64_t slowdown_ns = 0, std::int64_t orphan_timeout_ns = 1'000) {
  return WorkerSim::create({model(1, 10), model(2, 30)},
                           timing(slowdown_ns, orphan_timeout_ns, kLongLead))
      .value();
}

Assignment assignment(std::uint64_t job, std::int64_t planned_start,
                      std::uint32_t model_class = 1) {
  return Assignment{.job = core::JobId{job},
                    .robot = core::RobotId{7},
                    .model = core::ModelClass{model_class},
                    .planned_start = t(planned_start),
                    .chunk_bytes = core::Bytes{500}};
}

Completion completion(std::uint64_t job, std::int64_t start, std::int64_t end) {
  return Completion{.job = core::JobId{job},
                    .robot = core::RobotId{7},
                    .chunk_bytes = core::Bytes{500},
                    .observation_time = t(40),
                    .start = t(start),
                    .end = t(end)};
}

TEST(WorkerSim, ExecutesAtThePlannedStartForTheModelDuration) {
  WorkerSim sim = worker();
  ASSERT_TRUE(sim.assign(assignment(1, 100), t(0)));
  sim.arrive(core::JobId{1}, t(50), t(40));

  EXPECT_TRUE(sim.advance(t(99)).empty());
  EXPECT_EQ(sim.queued(), 1U);
  EXPECT_TRUE(sim.advance(t(109)).empty());
  EXPECT_EQ(sim.queued(), 0U);
  EXPECT_EQ(sim.advance(t(110)), (std::vector<Completion>{completion(1, 100, 110)}));
}

TEST(WorkerSim, WaitsForTheObservationBeforeStarting) {
  WorkerSim sim = worker();
  ASSERT_TRUE(sim.assign(assignment(1, 100), t(0)));

  EXPECT_TRUE(sim.advance(t(200)).empty());
  sim.arrive(core::JobId{1}, t(205), t(40));

  EXPECT_EQ(sim.advance(t(215)), (std::vector<Completion>{completion(1, 205, 215)}));
}

TEST(WorkerSim, RunsReadyJobsNonPreemptivelyInPlannedStartOrder) {
  WorkerSim sim = worker();
  ASSERT_TRUE(sim.assign(assignment(1, 105), t(0)));
  ASSERT_TRUE(sim.assign(assignment(2, 100, 2), t(0)));
  sim.arrive(core::JobId{1}, t(0), t(40));
  sim.arrive(core::JobId{2}, t(0), t(40));

  EXPECT_EQ(sim.advance(t(140)),
            (std::vector<Completion>{completion(2, 100, 130), completion(1, 130, 140)}));
}

TEST(WorkerSim, AppliesTheInjectedSlowdownToEveryExecution) {
  WorkerSim sim = worker(5);
  ASSERT_TRUE(sim.assign(assignment(1, 100), t(0)));
  sim.arrive(core::JobId{1}, t(0), t(40));

  EXPECT_TRUE(sim.advance(t(114)).empty());
  EXPECT_EQ(sim.advance(t(115)), (std::vector<Completion>{completion(1, 100, 115)}));
}

TEST(WorkerSim, RejectsAnUnknownModelOrARepeatedAssignment) {
  WorkerSim sim = worker();

  EXPECT_FALSE(sim.assign(assignment(1, 100, 9), t(0)));
  EXPECT_TRUE(sim.assign(assignment(1, 100), t(0)));
  EXPECT_FALSE(sim.assign(assignment(1, 120), t(0)));
  sim.arrive(core::JobId{1}, t(0), t(40));
  EXPECT_TRUE(sim.advance(t(100)).empty());
  EXPECT_FALSE(sim.assign(assignment(1, 120), t(0)));
}

TEST(WorkerSim, KeepsTheFirstArrivalOfAnObservation) {
  WorkerSim sim = worker();
  ASSERT_TRUE(sim.assign(assignment(1, 0), t(0)));
  sim.arrive(core::JobId{1}, t(20), t(40));
  sim.arrive(core::JobId{1}, t(50), t(45));

  EXPECT_EQ(sim.advance(t(30)), (std::vector<Completion>{completion(1, 20, 30)}));
}

TEST(WorkerSim, RejectsAnAssignmentPlannedBeyondTheMaximumLead) {
  WorkerSim sim = WorkerSim::create({model(1, 10)}, timing(0, 1'000, 50)).value();

  EXPECT_TRUE(sim.assign(assignment(1, 150), t(100)));
  EXPECT_FALSE(sim.assign(assignment(2, 151), t(100)));
}

TEST(WorkerSim, CreateRejectsInvalidProfilesSlowdownTimeoutOrLead) {
  EXPECT_FALSE(WorkerSim::create({}, timing(0, 10, kLongLead)).has_value());
  EXPECT_FALSE(WorkerSim::create({model(1, 0)}, timing(0, 10, kLongLead)).has_value());
  EXPECT_FALSE(WorkerSim::create({model(1, 5), model(1, 6)}, timing(0, 10, kLongLead)).has_value());
  EXPECT_FALSE(WorkerSim::create({model(1, 5)}, timing(-1, 10, kLongLead)).has_value());
  EXPECT_FALSE(WorkerSim::create({model(1, 5)}, timing(0, 0, kLongLead)).has_value());
  EXPECT_FALSE(WorkerSim::create({model(1, 5)}, timing(0, 10, -1)).has_value());
}

TEST(WorkerSim, DiscardsObservationsAndAssignmentsThatNeverMatch) {
  WorkerSim sim = worker(0, 100);
  sim.arrive(core::JobId{1}, t(0), t(0));
  ASSERT_TRUE(sim.assign(assignment(2, 50), t(0)));

  EXPECT_TRUE(sim.advance(t(100)).empty());
  EXPECT_EQ(sim.discarded(), 0U);
  EXPECT_TRUE(sim.advance(t(101)).empty());
  EXPECT_EQ(sim.discarded(), 1U);
  EXPECT_TRUE(sim.advance(t(151)).empty());
  EXPECT_EQ(sim.discarded(), 2U);
  EXPECT_EQ(sim.queued(), 0U);
}

TEST(WorkerSim, DiscardsAJobWhoseCompletionCannotBeRepresented) {
  constexpr std::int64_t kMax = std::numeric_limits<std::int64_t>::max();
  WorkerSim sim = worker(0, kMax);
  ASSERT_TRUE(sim.assign(assignment(1, kMax - 5), t(0)));
  sim.arrive(core::JobId{1}, t(0), t(0));

  EXPECT_TRUE(sim.advance(t(kMax)).empty());
  EXPECT_EQ(sim.discarded(), 1U);
  EXPECT_EQ(sim.queued(), 0U);
}

}  // namespace
}  // namespace rovi::worker
