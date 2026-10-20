#include "rovi_robot_agent/waypoint_commander.hpp"

#include <optional>
#include <utility>

#include "rovi_core/units.hpp"
#include "rovi_robot_agent/motion_monitor.hpp"
#include "rovi_robot_agent/waypoint_follower.hpp"

namespace rovi::robot_agent {

WaypointCommander::WaypointCommander(MotionMonitor monitor, WaypointFollower follower)
    : monitor_{monitor}, follower_{std::move(follower)} {}

void WaypointCommander::on_state(const ReportedState& state, core::TimePoint now) {
  monitor_.on_state(state, now);
}

void WaypointCommander::on_pose(const std::optional<Pose2D>& pose, core::TimePoint now) {
  if (!pose) {
    return;
  }
  pose_ = *pose;
  monitor_.on_feedback(now);
}

Velocity WaypointCommander::command(core::TimePoint now) {
  return monitor_.may_move(now) ? follower_.command(pose_) : Velocity{};
}

}  // namespace rovi::robot_agent
