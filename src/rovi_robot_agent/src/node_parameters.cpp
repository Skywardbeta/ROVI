#include "rovi_robot_agent/node_parameters.hpp"

#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/node.hpp>

#include "rovi_core/units.hpp"
#include "rovi_robot_agent/arm_joints.hpp"
#include "rovi_robot_agent/motion_monitor.hpp"

namespace rovi::robot_agent {
namespace {

constexpr std::int64_t kShortestControlPeriodNs = 1'000'000;
constexpr std::int64_t kLongestControlPeriodNs = 1'000'000'000;

}  // namespace

void reject(const rclcpp::Node& node, const std::string& name) {
  throw std::invalid_argument{std::string{node.get_name()} + ": invalid parameter " + name};
}

core::Duration positive_duration(rclcpp::Node& node, const std::string& name) {
  const std::int64_t value = node.declare_parameter<std::int64_t>(name);
  if (value <= 0) {
    reject(node, name);
  }
  return core::Duration::from_ns(value);
}

core::Duration period(rclcpp::Node& node, const std::string& name) {
  const std::int64_t value = node.declare_parameter<std::int64_t>(name);
  if (value < kShortestControlPeriodNs || value > kLongestControlPeriodNs) {
    reject(node, name);
  }
  return core::Duration::from_ns(value);
}

MotionMonitor load_monitor(rclcpp::Node& node, const std::string& feedback_timeout) {
  const std::int64_t robot = node.declare_parameter<std::int64_t>("robot_id");
  if (!std::in_range<std::uint32_t>(robot)) {
    reject(node, "robot_id");
  }
  return MotionMonitor{
      core::RobotId{static_cast<std::uint32_t>(robot)},
      MonitorTiming{.state_timeout = positive_duration(node, "state_timeout_ns"),
                    .feedback_timeout = positive_duration(node, feedback_timeout)}};
}

std::vector<std::string> load_joints(rclcpp::Node& node) {
  std::vector<std::string> joints = node.declare_parameter<std::vector<std::string>>("joint_names");
  if (!valid_joint_names(joints)) {
    reject(node, "joint_names");
  }
  return joints;
}

}  // namespace rovi::robot_agent
