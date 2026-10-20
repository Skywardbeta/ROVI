// Gates an arm's joint-goal reference on motion permission and holds the measured pose otherwise.
#ifndef ROVI_ROBOT_AGENT_ARM_COMMANDER_HPP_
#define ROVI_ROBOT_AGENT_ARM_COMMANDER_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "rovi_core/units.hpp"
#include "rovi_robot_agent/joint_goal_follower.hpp"
#include "rovi_robot_agent/motion_monitor.hpp"

namespace rovi::robot_agent {

class ArmCommander {
 public:
  ArmCommander(std::vector<std::string> joints, MotionMonitor monitor, JointGoalFollower follower);

  void on_state(const ReportedState& state, core::TimePoint now);
  void on_joint_state(const std::vector<std::string>& names, const std::vector<double>& positions,
                      core::TimePoint now);
  [[nodiscard]] std::optional<std::vector<double>> command(core::TimePoint now, core::Duration dt);
  [[nodiscard]] std::size_t goal() const;
  [[nodiscard]] std::uint64_t cycles() const;
  [[nodiscard]] bool moving() const;

 private:
  std::vector<std::string> joints_;
  MotionMonitor monitor_;
  JointGoalFollower follower_;
  std::vector<double> measured_;
  std::optional<std::vector<double>> last_command_;
  bool moving_{false};
};

}  // namespace rovi::robot_agent

#endif  // ROVI_ROBOT_AGENT_ARM_COMMANDER_HPP_
