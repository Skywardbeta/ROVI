// ROS 2 component that drives an arm through joint goals only while its action buffer allows
// motion.
#ifndef ROVI_ROBOT_AGENT_ARM_CONTROLLER_NODE_HPP_
#define ROVI_ROBOT_AGENT_ARM_CONTROLLER_NODE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include "rovi_core/units.hpp"
#include "rovi_interfaces/msg/robot_state.hpp"
#include "rovi_interfaces/msg/task_progress.hpp"
#include "rovi_robot_agent/arm_commander.hpp"
#include "rovi_telemetry/run_clock.hpp"

namespace rovi::robot_agent {

class ArmControllerNode : public rclcpp::Node {
 public:
  explicit ArmControllerNode(const rclcpp::NodeOptions& options);

 private:
  void on_state(const rovi_interfaces::msg::RobotState& state);
  void on_joint_state(const sensor_msgs::msg::JointState& state);
  void on_control();
  void on_progress();

  telemetry::RunClock clock_;
  std::vector<std::string> joints_;
  std::uint32_t robot_id_{0};
  core::Duration control_period_;
  ArmCommander commander_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> commands_;
  rclcpp::Subscription<rovi_interfaces::msg::RobotState>::SharedPtr states_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_;
  rclcpp::Publisher<rovi_interfaces::msg::TaskProgress>::SharedPtr progress_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr progress_timer_;
  rclcpp::Publisher<rovi_interfaces::msg::ComponentSettings>::SharedPtr settings_;
};

}  // namespace rovi::robot_agent

#endif  // ROVI_ROBOT_AGENT_ARM_CONTROLLER_NODE_HPP_
