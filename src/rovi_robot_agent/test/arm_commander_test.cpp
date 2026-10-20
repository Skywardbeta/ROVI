#include "rovi_robot_agent/arm_commander.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "rovi_core/units.hpp"
#include "rovi_robot_agent/joint_goal_follower.hpp"
#include "rovi_robot_agent/motion_monitor.hpp"

namespace rovi::robot_agent {
namespace {

constexpr std::int64_t kNsPerMs = 1'000'000;
constexpr core::RobotId kRobot{0};
constexpr core::Duration kStep = core::Duration::from_ns(100 * kNsPerMs);
constexpr double kPrecision = 1e-12;

core::TimePoint at_ms(std::int64_t count) { return core::TimePoint::from_ns(count * kNsPerMs); }

std::vector<std::string> joints() { return {"j1", "j2"}; }

ArmCommander commander() {
  const MotionMonitor monitor{
      kRobot, MonitorTiming{.state_timeout = core::Duration::from_ns(300 * kNsPerMs),
                            .feedback_timeout = core::Duration::from_ns(300 * kNsPerMs)}};
  JointGoalFollower follower = JointGoalFollower::create(JointFollowerConfig{.goals = {{1.0, -1.0}},
                                                                             .lower = {-2.0, -2.0},
                                                                             .upper = {2.0, 2.0},
                                                                             .max_speed = 0.5,
                                                                             .tolerance = 0.05,
                                                                             .settle_time = kStep})
                                   .value();
  return ArmCommander{joints(), monitor, std::move(follower)};
}

ReportedState executing_until(std::int64_t buffer_empty_ms) {
  return ReportedState{.robot = kRobot, .executing = true, .buffer_empty = at_ms(buffer_empty_ms)};
}

void expect_near(const std::optional<std::vector<double>>& actual,
                 const std::vector<double>& expected) {
  ASSERT_TRUE(actual.has_value());
  ASSERT_EQ(actual->size(), expected.size());
  for (std::size_t joint = 0; joint < expected.size(); ++joint) {
    EXPECT_NEAR(actual->at(joint), expected.at(joint), kPrecision) << "joint " << joint;
  }
}

TEST(ArmCommander, CommandsNothingUntilMotionHasBeenPermitted) {
  ArmCommander arm = commander();
  arm.on_joint_state(joints(), {0.0, 0.0}, at_ms(0));

  EXPECT_EQ(arm.command(at_ms(0), kStep), std::nullopt);
}

TEST(ArmCommander, StepsTowardTheGoalFromJointStatesReportedInAnyOrder) {
  ArmCommander arm = commander();
  arm.on_state(executing_until(1000), at_ms(0));
  arm.on_joint_state({"j2", "finger", "j1"}, {-0.5, 9.0, 0.5}, at_ms(0));

  expect_near(arm.command(at_ms(0), kStep), {0.55, -0.55});
  arm.on_joint_state(joints(), {0.5, -0.5}, at_ms(100));
  expect_near(arm.command(at_ms(100), kStep), {0.60, -0.60});
}

TEST(ArmCommander, HoldsTheMeasuredPoseOnceTheBufferEmptiesAndKeepsRepeatingIt) {
  ArmCommander arm = commander();
  arm.on_state(executing_until(150), at_ms(0));
  arm.on_joint_state(joints(), {0.0, 0.0}, at_ms(0));
  expect_near(arm.command(at_ms(0), kStep), {0.05, -0.05});
  arm.on_joint_state(joints(), {0.03, -0.02}, at_ms(100));
  expect_near(arm.command(at_ms(100), kStep), {0.10, -0.10});

  expect_near(arm.command(at_ms(200), kStep), {0.03, -0.02});
  arm.on_joint_state(joints(), {0.08, -0.07}, at_ms(250));
  expect_near(arm.command(at_ms(250), kStep), {0.03, -0.02});
}

TEST(ArmCommander, ResumesFromTheMeasuredPoseAfterAFallback) {
  ArmCommander arm = commander();
  arm.on_state(executing_until(150), at_ms(0));
  arm.on_joint_state(joints(), {0.0, 0.0}, at_ms(0));
  static_cast<void>(arm.command(at_ms(0), kStep));
  static_cast<void>(arm.command(at_ms(200), kStep));

  arm.on_state(executing_until(1000), at_ms(300));
  arm.on_joint_state(joints(), {0.04, -0.03}, at_ms(300));

  expect_near(arm.command(at_ms(300), kStep), {0.09, -0.08});
}

TEST(ArmCommander, IgnoresInvalidJointStatesSoTheyCannotKeepTheArmMoving) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  ArmCommander arm = commander();
  arm.on_state(executing_until(5000), at_ms(0));
  arm.on_joint_state(joints(), {0.0, 0.0}, at_ms(0));
  expect_near(arm.command(at_ms(0), kStep), {0.05, -0.05});

  arm.on_state(executing_until(5000), at_ms(100));
  arm.on_joint_state(joints(), {nan, 0.0}, at_ms(100));
  arm.on_state(executing_until(5000), at_ms(200));
  arm.on_joint_state({"j1"}, {0.0}, at_ms(200));
  arm.on_state(executing_until(5000), at_ms(300));
  arm.on_joint_state(joints(), {0.0}, at_ms(300));

  expect_near(arm.command(at_ms(350), kStep), {0.0, 0.0});
  expect_near(arm.command(at_ms(400), kStep), {0.0, 0.0});
}

TEST(ArmCommander, ReportsGoalCyclesAndWhetherItIsMoving) {
  ArmCommander arm = commander();
  EXPECT_FALSE(arm.moving());
  arm.on_state(executing_until(150), at_ms(0));
  arm.on_joint_state(joints(), {1.0, -1.0}, at_ms(0));

  static_cast<void>(arm.command(at_ms(0), kStep));
  EXPECT_TRUE(arm.moving());
  EXPECT_EQ(arm.cycles(), 1U);
  arm.on_joint_state(joints(), {1.0, -1.0}, at_ms(100));
  static_cast<void>(arm.command(at_ms(100), kStep));
  EXPECT_EQ(arm.goal(), 0U);
  EXPECT_EQ(arm.cycles(), 2U);
  static_cast<void>(arm.command(at_ms(200), kStep));

  EXPECT_FALSE(arm.moving());
}

}  // namespace
}  // namespace rovi::robot_agent
