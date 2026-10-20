#include "rovi_robot_agent/motion_monitor.hpp"

#include <cstdint>

#include <gtest/gtest.h>

#include "rovi_core/units.hpp"

namespace rovi::robot_agent {
namespace {

constexpr std::int64_t kNsPerMs = 1'000'000;
constexpr core::RobotId kRobot{3};

core::TimePoint at_ms(std::int64_t count) { return core::TimePoint::from_ns(count * kNsPerMs); }

MotionMonitor monitor() {
  return MotionMonitor{kRobot,
                       MonitorTiming{.state_timeout = core::Duration::from_ns(300 * kNsPerMs),
                                     .feedback_timeout = core::Duration::from_ns(300 * kNsPerMs)}};
}

ReportedState executing_until(std::int64_t buffer_empty_ms) {
  return ReportedState{.robot = kRobot, .executing = true, .buffer_empty = at_ms(buffer_empty_ms)};
}

TEST(MotionMonitor, StaysStoppedUntilBothStateAndFeedbackHaveArrived) {
  MotionMonitor only_feedback = monitor();
  only_feedback.on_feedback(at_ms(0));
  MotionMonitor only_state = monitor();
  only_state.on_state(executing_until(500), at_ms(0));

  EXPECT_FALSE(monitor().may_move(at_ms(0)));
  EXPECT_FALSE(only_feedback.may_move(at_ms(0)));
  EXPECT_FALSE(only_state.may_move(at_ms(0)));
}

TEST(MotionMonitor, AllowsMotionWhileExecutingWithFreshInputsAndBufferedActions) {
  MotionMonitor gate = monitor();
  gate.on_state(executing_until(500), at_ms(0));
  gate.on_feedback(at_ms(0));

  EXPECT_TRUE(gate.may_move(at_ms(100)));
}

TEST(MotionMonitor, IgnoresTheStateOfAnotherRobot) {
  MotionMonitor gate = monitor();
  gate.on_feedback(at_ms(0));

  gate.on_state({.robot = core::RobotId{9}, .executing = true, .buffer_empty = at_ms(500)},
                at_ms(0));

  EXPECT_FALSE(gate.may_move(at_ms(0)));
}

TEST(MotionMonitor, StopsWhenTheAgentReportsFallback) {
  MotionMonitor gate = monitor();
  gate.on_feedback(at_ms(0));
  gate.on_state(executing_until(500), at_ms(0));

  gate.on_state({.robot = kRobot, .executing = false, .buffer_empty = at_ms(500)}, at_ms(10));

  EXPECT_FALSE(gate.may_move(at_ms(10)));
}

TEST(MotionMonitor, StopsOnceTheActionBufferEmptiesBetweenStateReports) {
  MotionMonitor gate = monitor();
  gate.on_state(executing_until(100), at_ms(0));
  gate.on_feedback(at_ms(50));

  EXPECT_TRUE(gate.may_move(at_ms(99)));
  EXPECT_FALSE(gate.may_move(at_ms(100)));
}

TEST(MotionMonitor, StopsWhenStateOrFeedbackOutlivesItsTimeout) {
  MotionMonitor gate = monitor();
  gate.on_state(executing_until(5'000), at_ms(0));
  gate.on_feedback(at_ms(0));

  EXPECT_TRUE(gate.may_move(at_ms(300)));
  EXPECT_FALSE(gate.may_move(at_ms(301)));
  gate.on_state(executing_until(5'000), at_ms(301));
  EXPECT_FALSE(gate.may_move(at_ms(301)));
  gate.on_feedback(at_ms(301));
  EXPECT_TRUE(gate.may_move(at_ms(301)));
}

}  // namespace
}  // namespace rovi::robot_agent
