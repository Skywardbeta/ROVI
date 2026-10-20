// Parameter loading and validation shared by the robot agent's controller components.
#ifndef ROVI_ROBOT_AGENT_NODE_PARAMETERS_HPP_
#define ROVI_ROBOT_AGENT_NODE_PARAMETERS_HPP_

#include <string>

#include <rclcpp/node.hpp>

#include "rovi_core/units.hpp"
#include "rovi_robot_agent/motion_monitor.hpp"

namespace rovi::robot_agent {

[[noreturn]] void reject(const rclcpp::Node& node, const std::string& name);

[[nodiscard]] core::Duration positive_duration(rclcpp::Node& node, const std::string& name);

[[nodiscard]] core::Duration period(rclcpp::Node& node, const std::string& name);

[[nodiscard]] MotionMonitor load_monitor(rclcpp::Node& node, const std::string& feedback_timeout);

}  // namespace rovi::robot_agent

#endif  // ROVI_ROBOT_AGENT_NODE_PARAMETERS_HPP_
