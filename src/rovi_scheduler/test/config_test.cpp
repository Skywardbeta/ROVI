#include "rovi_scheduler/config.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "rovi_core/admission.hpp"
#include "rovi_core/scheduler.hpp"
#include "rovi_core/units.hpp"
#include "rovi_core/window.hpp"
#include "rovi_core/wireless_profile.hpp"

namespace rovi::scheduler {
namespace {

constexpr std::int64_t kNsPerMs = 1'000'000;

core::Duration ms(std::int64_t count) { return core::Duration::from_ns(count * kNsPerMs); }

core::ServiceSegment segment(std::int64_t first_slot, std::int64_t rate, std::int32_t reduction) {
  return core::ServiceSegment{.first_slot = first_slot,
                              .rate = core::BitsPerSecond{rate},
                              .reduction = core::Permille{reduction}};
}

core::ExecutionProfile inference(std::uint32_t model, std::int64_t duration_ns) {
  return core::ExecutionProfile{.model = core::ModelClass{model},
                                .duration = core::Duration::from_ns(duration_ns)};
}

SchedulerConfig reference_config() {
  return SchedulerConfig{.policy = core::Policy::kLeastSlack,
                         .cycle_period = ms(10),
                         .release_tolerance = ms(50),
                         .horizon_slots = 200,
                         .max_pending_jobs = 64,
                         .profile = core::ServiceProfile{.slot_length = ms(1),
                                                         .uplink = {segment(0, 100'000'000, 100)},
                                                         .downlink = {segment(0, 50'000'000, 100)},
                                                         .uplink_delay = ms(2),
                                                         .downlink_delay = ms(2)},
                         .workers = {core::WorkerSpec{.id = core::WorkerId{1},
                                                      .profiles = {inference(1, 40 * kNsPerMs)}}}};
}

TEST(SchedulerConfig, ParsesEveryPolicyName) {
  EXPECT_EQ(policy_from("least_slack"), core::Policy::kLeastSlack);
  EXPECT_EQ(policy_from("earliest_deadline"), core::Policy::kEarliestDeadline);
  EXPECT_EQ(policy_from("first_in_first_out"), core::Policy::kFirstInFirstOut);
  EXPECT_EQ(policy_from("optimal"), std::nullopt);
  EXPECT_EQ(policy_from(""), std::nullopt);
}

TEST(SchedulerConfig, BuildsSegmentsFromParallelArrays) {
  const std::optional<std::vector<core::ServiceSegment>> segments = segments_from(
      {.first_slots = {0, 5}, .rates_bps = {1'000, 0}, .reductions_permille = {100, 0}});

  ASSERT_TRUE(segments.has_value());
  EXPECT_EQ(segments.value(),
            (std::vector<core::ServiceSegment>{segment(0, 1'000, 100), segment(5, 0, 0)}));
}

TEST(SchedulerConfig, RejectsEmptyMismatchedOrNarrowingSegmentArrays) {
  constexpr std::int64_t kBeyondInt32 = std::int64_t{std::numeric_limits<std::int32_t>::max()} + 1;

  EXPECT_EQ(segments_from({}), std::nullopt);
  EXPECT_EQ(segments_from({.first_slots = {0}, .rates_bps = {1}, .reductions_permille = {}}),
            std::nullopt);
  EXPECT_EQ(segments_from({.first_slots = {0}, .rates_bps = {1, 2}, .reductions_permille = {0}}),
            std::nullopt);
  EXPECT_EQ(
      segments_from({.first_slots = {0}, .rates_bps = {1}, .reductions_permille = {kBeyondInt32}}),
      std::nullopt);
  EXPECT_EQ(segments_from(
                {.first_slots = {0}, .rates_bps = {1}, .reductions_permille = {-kBeyondInt32 - 1}}),
            std::nullopt);
}

TEST(SchedulerConfig, GroupsWorkerProfilesByAscendingWorkerId) {
  const std::optional<std::vector<core::WorkerSpec>> workers =
      workers_from({.ids = {2, 1, 2}, .model_classes = {1, 1, 2}, .durations_ns = {30, 10, 40}});

  const std::vector<core::WorkerSpec> expected{
      {.id = core::WorkerId{1}, .profiles = {inference(1, 10)}},
      {.id = core::WorkerId{2}, .profiles = {inference(1, 30), inference(2, 40)}}};
  ASSERT_TRUE(workers.has_value());
  EXPECT_EQ(workers.value(), expected);
}

TEST(SchedulerConfig, RejectsEmptyMismatchedOrNarrowingWorkerArrays) {
  constexpr std::int64_t kBeyondUint32 =
      std::int64_t{std::numeric_limits<std::uint32_t>::max()} + 1;

  EXPECT_EQ(workers_from({}), std::nullopt);
  EXPECT_EQ(workers_from({.ids = {1}, .model_classes = {1, 2}, .durations_ns = {10}}),
            std::nullopt);
  EXPECT_EQ(workers_from({.ids = {1}, .model_classes = {1}, .durations_ns = {}}), std::nullopt);
  EXPECT_EQ(workers_from({.ids = {-1}, .model_classes = {1}, .durations_ns = {10}}), std::nullopt);
  EXPECT_EQ(workers_from({.ids = {1}, .model_classes = {-1}, .durations_ns = {10}}), std::nullopt);
  EXPECT_EQ(workers_from({.ids = {1}, .model_classes = {kBeyondUint32}, .durations_ns = {10}}),
            std::nullopt);
}

TEST(SchedulerConfig, AcceptsTheReferenceConfigurationAndAToleranceEqualToTheWindow) {
  SchedulerConfig window_tolerance = reference_config();
  window_tolerance.release_tolerance = ms(200);

  EXPECT_TRUE(is_valid(reference_config()));
  EXPECT_TRUE(is_valid(window_tolerance));
}

TEST(SchedulerConfig, RejectsNonPositivePeriodsHorizonsOrAnUnusableWindow) {
  SchedulerConfig zero_cycle = reference_config();
  zero_cycle.cycle_period = ms(0);
  SchedulerConfig negative_tolerance = reference_config();
  negative_tolerance.release_tolerance = ms(-1);
  SchedulerConfig zero_horizon = reference_config();
  zero_horizon.horizon_slots = 0;
  SchedulerConfig huge_horizon = reference_config();
  huge_horizon.horizon_slots = kMaximumHorizonSlots + 1;
  SchedulerConfig no_pending_room = reference_config();
  no_pending_room.max_pending_jobs = 0;
  SchedulerConfig huge_queue = reference_config();
  huge_queue.max_pending_jobs = kMaximumPendingJobs + 1;
  SchedulerConfig tolerance_beyond_window = reference_config();
  tolerance_beyond_window.release_tolerance = ms(201);
  SchedulerConfig no_workers = reference_config();
  no_workers.workers.clear();
  SchedulerConfig idle_worker = reference_config();
  idle_worker.workers.front().profiles.front().duration = ms(0);
  SchedulerConfig bad_profile = reference_config();
  bad_profile.profile.uplink.front().first_slot = 3;
  SchedulerConfig negative_lead = reference_config();
  negative_lead.profile.dispatch_lead = core::Duration::from_ns(-1);

  EXPECT_FALSE(is_valid(zero_cycle));
  EXPECT_FALSE(is_valid(negative_tolerance));
  EXPECT_FALSE(is_valid(zero_horizon));
  EXPECT_FALSE(is_valid(huge_horizon));
  EXPECT_FALSE(is_valid(no_pending_room));
  EXPECT_FALSE(is_valid(huge_queue));
  EXPECT_FALSE(is_valid(tolerance_beyond_window));
  EXPECT_FALSE(is_valid(no_workers));
  EXPECT_FALSE(is_valid(idle_worker));
  EXPECT_FALSE(is_valid(bad_profile));
  EXPECT_FALSE(is_valid(negative_lead));
}

TEST(SchedulerConfig, DigestIsStableAndSensitiveToEveryDecisionRelevantField) {
  const SchedulerConfig base = reference_config();
  std::vector<SchedulerConfig> variants(12, base);
  variants.at(0).policy = core::Policy::kEarliestDeadline;
  variants.at(1).cycle_period = ms(11);
  variants.at(2).release_tolerance = ms(51);
  variants.at(3).horizon_slots = 201;
  variants.at(4).profile.slot_length = ms(2);
  variants.at(5).profile.downlink.front().rate = core::BitsPerSecond{50'000'001};
  variants.at(6).profile.medium = std::vector<core::ServiceSegment>{segment(0, 1, 0)};
  variants.at(7).profile.uplink_delay = ms(3);
  variants.at(8).profile.source = core::ServiceSource::kReplayed;
  variants.at(9).workers.front().profiles.front().duration = ms(41);
  variants.at(10).max_pending_jobs = 65;
  variants.at(11).profile.dispatch_lead = ms(1);

  EXPECT_EQ(config_digest(base), config_digest(reference_config()));
  for (const SchedulerConfig& variant : variants) {
    EXPECT_NE(config_digest(variant), config_digest(base));
  }
}

}  // namespace
}  // namespace rovi::scheduler
