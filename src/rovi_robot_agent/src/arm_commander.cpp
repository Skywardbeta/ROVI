#include "rovi_robot_agent/arm_commander.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "rovi_core/units.hpp"
#include "rovi_robot_agent/arm_joints.hpp"
#include "rovi_robot_agent/joint_goal_follower.hpp"
#include "rovi_robot_agent/motion_monitor.hpp"

namespace rovi::robot_agent {

ArmCommander::ArmCommander(std::vector<std::string> joints, MotionMonitor monitor,
                           JointGoalFollower follower)
    : joints_{std::move(joints)}, monitor_{monitor}, follower_{std::move(follower)} {}

void ArmCommander::on_state(const ReportedState& state, core::TimePoint now) {
  monitor_.on_state(state, now);
}

void ArmCommander::on_joint_state(const std::vector<std::string>& names,
                                  const std::vector<double>& positions, core::TimePoint now) {
  std::optional<std::vector<double>> ordered = ordered_positions(names, positions, joints_);
  if (!ordered) {
    return;
  }
  measured_ = std::move(*ordered);
  monitor_.on_feedback(now);
}

std::optional<std::vector<double>> ArmCommander::command(core::TimePoint now, core::Duration dt) {
  const bool permitted = monitor_.may_move(now);
  std::optional<std::vector<double>> next;
  if (permitted) {
    next = follower_.step(measured_, dt);
  } else if (moving_) {
    next = follower_.stop(measured_);
  }
  moving_ = permitted;
  if (next) {
    last_command_ = std::move(next);
  }
  return last_command_;
}

std::size_t ArmCommander::goal() const { return follower_.goal(); }

std::uint64_t ArmCommander::cycles() const { return follower_.cycles(); }

bool ArmCommander::moving() const { return moving_; }

}  // namespace rovi::robot_agent
