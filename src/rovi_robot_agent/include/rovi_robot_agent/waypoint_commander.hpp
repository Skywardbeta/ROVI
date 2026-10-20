// Gates waypoint velocity commands on motion permission and valid odometry, stopping otherwise.
#ifndef ROVI_ROBOT_AGENT_WAYPOINT_COMMANDER_HPP_
#define ROVI_ROBOT_AGENT_WAYPOINT_COMMANDER_HPP_

#include <optional>

#include "rovi_core/units.hpp"
#include "rovi_robot_agent/motion_monitor.hpp"
#include "rovi_robot_agent/waypoint_follower.hpp"

namespace rovi::robot_agent {

class WaypointCommander {
 public:
  WaypointCommander(MotionMonitor monitor, WaypointFollower follower);

  void on_state(const ReportedState& state, core::TimePoint now);
  void on_pose(const std::optional<Pose2D>& pose, core::TimePoint now);
  [[nodiscard]] Velocity command(core::TimePoint now);

 private:
  MotionMonitor monitor_;
  WaypointFollower follower_;
  Pose2D pose_{};
};

}  // namespace rovi::robot_agent

#endif  // ROVI_ROBOT_AGENT_WAYPOINT_COMMANDER_HPP_
