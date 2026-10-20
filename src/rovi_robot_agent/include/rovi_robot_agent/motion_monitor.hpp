// Ownership, freshness, and action-buffer checks that decide whether the robot may move.
#ifndef ROVI_ROBOT_AGENT_MOTION_MONITOR_HPP_
#define ROVI_ROBOT_AGENT_MOTION_MONITOR_HPP_

#include <optional>

#include "rovi_core/units.hpp"

namespace rovi::robot_agent {

struct ReportedState {
  core::RobotId robot;
  bool executing{false};
  core::TimePoint buffer_empty;
};

struct MonitorTiming {
  core::Duration state_timeout;
  core::Duration feedback_timeout;
};

class MotionMonitor {
 public:
  MotionMonitor(core::RobotId robot, MonitorTiming timing);

  void on_state(const ReportedState& state, core::TimePoint now);
  void on_feedback(core::TimePoint now);
  [[nodiscard]] bool may_move(core::TimePoint now) const;

 private:
  core::RobotId robot_;
  MonitorTiming timing_;
  std::optional<core::TimePoint> state_received_;
  bool executing_{false};
  core::TimePoint buffer_empty_{};
  std::optional<core::TimePoint> feedback_received_;
};

}  // namespace rovi::robot_agent

#endif  // ROVI_ROBOT_AGENT_MOTION_MONITOR_HPP_
