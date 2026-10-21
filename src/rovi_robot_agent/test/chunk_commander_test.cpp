#include "rovi_robot_agent/chunk_commander.hpp"

#include <cstdint>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "rovi_core/units.hpp"
#include "rovi_robot_agent/chunk_player.hpp"
#include "rovi_robot_agent/joint_calibration.hpp"
#include "rovi_robot_agent/motion_monitor.hpp"

namespace rovi::robot_agent {
namespace {

constexpr double kDegree = std::numbers::pi / 180.0;

core::TimePoint at_ms(std::int64_t ms) { return core::TimePoint::from_ns(ms * 1'000'000); }

ChunkCommander commander() {
  return ChunkCommander{
      std::vector<std::string>{"pan", "grip"},
      MotionMonitor{core::RobotId{0},
                    MonitorTiming{.state_timeout = core::Duration::from_ns(500'000'000),
                                  .feedback_timeout = core::Duration::from_ns(500'000'000)}},
      *CalibrationMap::create(
          {JointCalibration{.sign = 1.0, .zero_deg = 0.0, .lower_rad = -2.0, .upper_rad = 2.0},
           JointCalibration{.sign = 1.0, .zero_deg = 0.0, .lower_rad = -0.2, .upper_rad = 1.8}})};
}

void report(ChunkCommander& arm, bool executing, std::int64_t buffer_empty_ms,
            std::int64_t now_ms) {
  arm.on_state(ReportedState{.robot = core::RobotId{0},
                             .executing = executing,
                             .buffer_empty = at_ms(buffer_empty_ms)},
               at_ms(now_ms));
  arm.on_joint_state({"grip", "pan"}, {0.5, 0.25}, at_ms(now_ms));
}

PlayedChunk chunk_at(std::int64_t start_ms) {
  return PlayedChunk{.start = at_ms(start_ms),
                     .period = core::Duration::from_ns(100'000'000),
                     .steps = 2,
                     .dim = 2,
                     .actions = {10.0F, 50.0F, 20.0F, 60.0F}};
}

TEST(ChunkCommander, CommandsNothingBeforeAnyChunkPlays) {
  ChunkCommander arm = commander();
  report(arm, true, 2'000, 1'000);
  EXPECT_FALSE(arm.command(at_ms(1'000)).has_value());
  ASSERT_TRUE(arm.on_chunk(chunk_at(1'100)));
  EXPECT_FALSE(arm.command(at_ms(1'050)).has_value());
  EXPECT_FALSE(arm.playing());
}

TEST(ChunkCommander, PlaysTheMappedActionWhileMotionIsPermitted) {
  ChunkCommander arm = commander();
  report(arm, true, 2'000, 1'000);
  ASSERT_TRUE(arm.on_chunk(chunk_at(1'000)));
  const std::optional<std::vector<double>> first = arm.command(at_ms(1'000));
  ASSERT_TRUE(first.has_value());
  EXPECT_DOUBLE_EQ(first->at(0), 10.0 * kDegree);
  EXPECT_DOUBLE_EQ(first->at(1), 50.0 * kDegree);
  EXPECT_DOUBLE_EQ(arm.command(at_ms(1'100))->at(0), 20.0 * kDegree);
  EXPECT_TRUE(arm.playing());
}

TEST(ChunkCommander, HoldsTheMeasuredPoseWhenMotionStopsOrTheChunkEnds) {
  ChunkCommander fallback = commander();
  report(fallback, true, 2'000, 1'000);
  ASSERT_TRUE(fallback.on_chunk(chunk_at(1'000)));
  ASSERT_TRUE(fallback.command(at_ms(1'000)).has_value());
  report(fallback, false, 1'050, 1'050);
  EXPECT_EQ(fallback.command(at_ms(1'050)), (std::vector<double>{0.25, 0.5}));
  EXPECT_FALSE(fallback.playing());

  ChunkCommander ended = commander();
  report(ended, true, 2'000, 1'000);
  ASSERT_TRUE(ended.on_chunk(chunk_at(1'000)));
  ASSERT_TRUE(ended.command(at_ms(1'150)).has_value());
  report(ended, true, 2'000, 1'200);
  EXPECT_EQ(ended.command(at_ms(1'200)), (std::vector<double>{0.25, 0.5}));
}

TEST(ChunkCommander, ResumesWithTheNextChunkAfterHolding) {
  ChunkCommander arm = commander();
  report(arm, true, 3'000, 1'000);
  ASSERT_TRUE(arm.on_chunk(chunk_at(1'000)));
  ASSERT_TRUE(arm.command(at_ms(1'000)).has_value());
  EXPECT_EQ(arm.command(at_ms(1'250)), (std::vector<double>{0.25, 0.5}));
  ASSERT_TRUE(arm.on_chunk(chunk_at(1'300)));
  EXPECT_DOUBLE_EQ(arm.command(at_ms(1'300))->at(0), 10.0 * kDegree);
}

TEST(ChunkCommander, CountsTheCommandsClampedToTheJointLimits) {
  ChunkCommander arm = commander();
  report(arm, true, 2'000, 1'000);
  PlayedChunk wide = chunk_at(1'000);
  wide.actions = {10.0F, 50.0F, 10.0F, 120.0F};
  ASSERT_TRUE(arm.on_chunk(wide));
  ASSERT_TRUE(arm.command(at_ms(1'000)).has_value());
  EXPECT_EQ(arm.clamped_commands(), 0U);
  EXPECT_DOUBLE_EQ(arm.command(at_ms(1'100))->at(1), 1.8);
  EXPECT_EQ(arm.clamped_commands(), 1U);
}

TEST(ChunkCommander, DoesNotMoveOnJointStatesThatMissAJoint) {
  ChunkCommander arm = commander();
  arm.on_state(
      ReportedState{.robot = core::RobotId{0}, .executing = true, .buffer_empty = at_ms(2'000)},
      at_ms(1'000));
  arm.on_joint_state({"pan", "elbow"}, {0.25, 0.5}, at_ms(1'000));
  ASSERT_TRUE(arm.on_chunk(chunk_at(1'000)));
  EXPECT_FALSE(arm.command(at_ms(1'000)).has_value());
}

TEST(ChunkCommander, RefusesAChunkForAnotherJointCount) {
  ChunkCommander arm = commander();
  PlayedChunk single = chunk_at(1'000);
  single.dim = 1;
  single.actions = {1.0F, 2.0F};
  EXPECT_FALSE(arm.on_chunk(single));
}

}  // namespace
}  // namespace rovi::robot_agent
