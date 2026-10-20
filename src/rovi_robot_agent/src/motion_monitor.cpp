#include "rovi_robot_agent/motion_monitor.hpp"

#include <optional>

#include "rovi_core/units.hpp"

namespace rovi::robot_agent {
namespace {

bool fresh(const std::optional<core::TimePoint>& received, core::Duration timeout,
           core::TimePoint now) {
  return received && !core::elapsed_beyond(*received, timeout, now);
}

}  // namespace

MotionMonitor::MotionMonitor(core::RobotId robot, MonitorTiming timing)
    : robot_{robot}, timing_{timing} {}

void MotionMonitor::on_state(const ReportedState& state, core::TimePoint now) {
  if (state.robot != robot_) {
    return;
  }
  state_received_ = now;
  executing_ = state.executing;
  buffer_empty_ = state.buffer_empty;
}

void MotionMonitor::on_feedback(core::TimePoint now) { feedback_received_ = now; }

bool MotionMonitor::may_move(core::TimePoint now) const {
  return executing_ && now < buffer_empty_ && fresh(state_received_, timing_.state_timeout, now) &&
         fresh(feedback_received_, timing_.feedback_timeout, now);
}

}  // namespace rovi::robot_agent
