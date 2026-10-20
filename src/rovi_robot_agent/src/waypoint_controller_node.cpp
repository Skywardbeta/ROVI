#include "rovi_robot_agent/waypoint_controller_node.hpp"

#include <chrono>
#include <cstddef>
#include <exception>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "rovi_core/units.hpp"
#include "rovi_interfaces/msg/robot_state.hpp"
#include "rovi_robot_agent/motion_monitor.hpp"
#include "rovi_robot_agent/node_parameters.hpp"
#include "rovi_robot_agent/waypoint_commander.hpp"
#include "rovi_robot_agent/waypoint_follower.hpp"
#include "rovi_telemetry/run_clock.hpp"

namespace rovi::robot_agent {
namespace {

using rovi_interfaces::msg::RobotState;

constexpr std::size_t kStateDepth = 10;
constexpr std::size_t kSensorDepth = 10;

WaypointFollower load_follower(rclcpp::Node& node) {
  const std::vector<double> flat = node.declare_parameter<std::vector<double>>("waypoints_xy");
  if (flat.size() % 2 != 0) {
    reject(node, "waypoints_xy");
  }
  std::vector<Point2D> waypoints;
  for (std::size_t index = 0; index + 1 < flat.size(); index += 2) {
    waypoints.push_back(Point2D{.x = flat.at(index), .y = flat.at(index + 1)});
  }
  std::optional<WaypointFollower> follower = WaypointFollower::create(
      FollowerConfig{.waypoints = std::move(waypoints),
                     .arrival_tolerance = node.declare_parameter<double>("arrival_tolerance_m"),
                     .max_linear = node.declare_parameter<double>("max_linear_mps"),
                     .max_angular = node.declare_parameter<double>("max_angular_rps"),
                     .heading_gain = node.declare_parameter<double>("heading_gain")});
  if (!follower) {
    reject(node, "waypoints or motion limits");
  }
  return std::move(*follower);
}

}  // namespace

WaypointControllerNode::WaypointControllerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("waypoint_controller", options),
      clock_{*this},
      commander_{load_monitor(*this, "odometry_timeout_ns"), load_follower(*this)} {
  const core::Duration control = period(*this, "control_period_ns");
  commands_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(kSensorDepth));
  states_ = create_subscription<RobotState>("state",
                                            rclcpp::QoS(rclcpp::KeepLast(kStateDepth)).reliable(),
                                            [this](const RobotState& state) { on_state(state); });
  odometry_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::QoS(kSensorDepth),
      [this](const nav_msgs::msg::Odometry& odometry) { on_odometry(odometry); });
  control_timer_ = create_timer(std::chrono::nanoseconds{control.ns()}, [this] { on_control(); });
  settings_ = telemetry::announce_settings(*this);
}

WaypointControllerNode::~WaypointControllerNode() {
  try {
    if (get_node_base_interface()->get_context()->is_valid()) {
      commands_->publish(geometry_msgs::msg::Twist{});
    }
  } catch (const std::exception& error) {
    RCLCPP_WARN(get_logger(), "could not publish the final stop command: %s", error.what());
  }
}

void WaypointControllerNode::on_state(const RobotState& state) {
  const std::optional<core::TimePoint> now = clock_.now();
  if (!now) {
    return;
  }
  commander_.on_state(
      ReportedState{.robot = core::RobotId{state.robot_id},
                    .executing = state.mode == RobotState::MODE_EXECUTING,
                    .buffer_empty = core::TimePoint::from_ns(state.buffer_empty_time_ns)},
      *now);
}

void WaypointControllerNode::on_odometry(const nav_msgs::msg::Odometry& odometry) {
  const std::optional<core::TimePoint> now = clock_.now();
  if (!now) {
    return;
  }
  const auto& position = odometry.pose.pose.position;
  const auto& orientation = odometry.pose.pose.orientation;
  commander_.on_pose(planar_pose(position.x, position.y, orientation.x, orientation.y,
                                 orientation.z, orientation.w),
                     *now);
}

// After the run is invalidated the robot is held still, since the drive keeps its last command.
void WaypointControllerNode::on_control() {
  geometry_msgs::msg::Twist command;
  if (const std::optional<core::TimePoint> now = clock_.now()) {
    const Velocity velocity = commander_.command(*now);
    command.linear.x = velocity.linear;
    command.angular.z = velocity.angular;
  }
  commands_->publish(command);
}

}  // namespace rovi::robot_agent

RCLCPP_COMPONENTS_REGISTER_NODE(rovi::robot_agent::WaypointControllerNode)
