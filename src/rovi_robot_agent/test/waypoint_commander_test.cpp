#include "rovi_robot_agent/waypoint_commander.hpp"

#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

#include "rovi_core/units.hpp"
#include "rovi_robot_agent/motion_monitor.hpp"
#include "rovi_robot_agent/waypoint_follower.hpp"

namespace rovi::robot_agent {
namespace {

constexpr std::int64_t kNsPerMs = 1'000'000;
constexpr core::RobotId kRobot{0};
constexpr double kMaxLinear = 0.5;

core::TimePoint at_ms(std::int64_t count) { return core::TimePoint::from_ns(count * kNsPerMs); }

WaypointCommander commander() {
  const MotionMonitor monitor{
      kRobot, MonitorTiming{.state_timeout = core::Duration::from_ns(300 * kNsPerMs),
                            .feedback_timeout = core::Duration::from_ns(300 * kNsPerMs)}};
  return WaypointCommander{
      monitor, WaypointFollower::create(FollowerConfig{.waypoints = {{.x = 5.0, .y = 0.0}},
                                                       .arrival_tolerance = 0.3,
                                                       .max_linear = kMaxLinear,
                                                       .max_angular = 1.0,
                                                       .heading_gain = 1.5})
                   .value()};
}

ReportedState executing_until(std::int64_t buffer_empty_ms) {
  return ReportedState{.robot = kRobot, .executing = true, .buffer_empty = at_ms(buffer_empty_ms)};
}

TEST(WaypointCommander, StaysStillUntilBothStateAndAPoseHaveArrived) {
  WaypointCommander only_state = commander();
  only_state.on_state(executing_until(1000), at_ms(0));
  WaypointCommander only_pose = commander();
  only_pose.on_pose(Pose2D{}, at_ms(0));

  EXPECT_EQ(commander().command(at_ms(0)), Velocity{});
  EXPECT_EQ(only_state.command(at_ms(0)), Velocity{});
  EXPECT_EQ(only_pose.command(at_ms(0)), Velocity{});
}

TEST(WaypointCommander, DrivesTowardTheWaypointWhileExecuting) {
  WaypointCommander robot = commander();
  robot.on_state(executing_until(1000), at_ms(0));
  robot.on_pose(Pose2D{}, at_ms(0));

  EXPECT_EQ(robot.command(at_ms(0)), (Velocity{.linear = kMaxLinear, .angular = 0.0}));
}

TEST(WaypointCommander, IgnoresNonFiniteOdometrySoItCannotKeepTheRobotMoving) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  WaypointCommander robot = commander();
  robot.on_state(executing_until(5000), at_ms(0));
  robot.on_pose(planar_pose(0.0, 0.0, 0.0, 0.0, 0.0, 1.0), at_ms(0));
  ASSERT_NE(robot.command(at_ms(0)), Velocity{});

  for (std::int64_t ms = 100; ms <= 300; ms += 100) {
    robot.on_state(executing_until(5000), at_ms(ms));
    robot.on_pose(planar_pose(nan, 0.0, 0.0, 0.0, 0.0, 1.0), at_ms(ms));
  }

  EXPECT_EQ(robot.command(at_ms(350)), Velocity{});
}

TEST(WaypointCommander, StopsWhenTheAgentReportsFallback) {
  WaypointCommander robot = commander();
  robot.on_state(executing_until(1000), at_ms(0));
  robot.on_pose(Pose2D{}, at_ms(0));
  robot.on_state(ReportedState{.robot = kRobot, .executing = false, .buffer_empty = at_ms(0)},
                 at_ms(100));
  robot.on_pose(Pose2D{}, at_ms(100));

  EXPECT_EQ(robot.command(at_ms(100)), Velocity{});
}

}  // namespace
}  // namespace rovi::robot_agent
