// ROS 2 component that follows waypoints only while the robot's action buffer allows motion.
#ifndef ROVI_ROBOT_AGENT_WAYPOINT_CONTROLLER_NODE_HPP_
#define ROVI_ROBOT_AGENT_WAYPOINT_CONTROLLER_NODE_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>

#include "rovi_core/units.hpp"
#include "rovi_interfaces/msg/robot_state.hpp"
#include "rovi_robot_agent/waypoint_commander.hpp"
#include "rovi_telemetry/run_clock.hpp"

namespace rovi::robot_agent {

class WaypointControllerNode : public rclcpp::Node {
 public:
  explicit WaypointControllerNode(const rclcpp::NodeOptions& options);
  WaypointControllerNode(const WaypointControllerNode&) = delete;
  WaypointControllerNode& operator=(const WaypointControllerNode&) = delete;
  WaypointControllerNode(WaypointControllerNode&&) = delete;
  WaypointControllerNode& operator=(WaypointControllerNode&&) = delete;
  ~WaypointControllerNode() override;

 private:
  void on_state(const rovi_interfaces::msg::RobotState& state);
  void on_odometry(const nav_msgs::msg::Odometry& odometry);
  void on_control();

  telemetry::RunClock clock_;
  WaypointCommander commander_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr commands_;
  rclcpp::Subscription<rovi_interfaces::msg::RobotState>::SharedPtr states_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Publisher<rovi_interfaces::msg::ComponentSettings>::SharedPtr settings_;
};

}  // namespace rovi::robot_agent

#endif  // ROVI_ROBOT_AGENT_WAYPOINT_CONTROLLER_NODE_HPP_
