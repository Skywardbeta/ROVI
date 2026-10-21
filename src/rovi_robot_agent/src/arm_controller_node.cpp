#include "rovi_robot_agent/arm_controller_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include "rovi_core/units.hpp"
#include "rovi_interfaces/msg/robot_state.hpp"
#include "rovi_interfaces/msg/task_progress.hpp"
#include "rovi_robot_agent/arm_commander.hpp"
#include "rovi_robot_agent/arm_joints.hpp"
#include "rovi_robot_agent/joint_goal_follower.hpp"
#include "rovi_robot_agent/motion_monitor.hpp"
#include "rovi_robot_agent/node_parameters.hpp"
#include "rovi_telemetry/run_clock.hpp"

namespace rovi::robot_agent {
namespace {

using rovi_interfaces::msg::RobotState;

constexpr std::size_t kStateDepth = 10;
constexpr std::size_t kSensorDepth = 10;

JointGoalFollower load_follower(rclcpp::Node& node, std::size_t joints) {
  std::optional<std::vector<std::vector<double>>> goals =
      split_goals(node.declare_parameter<std::vector<double>>("goals_rad"), joints);
  if (!goals) {
    reject(node, "goals_rad");
  }
  std::optional<JointGoalFollower> follower = JointGoalFollower::create(
      JointFollowerConfig{.goals = std::move(*goals),
                          .lower = node.declare_parameter<std::vector<double>>("joint_lower_rad"),
                          .upper = node.declare_parameter<std::vector<double>>("joint_upper_rad"),
                          .max_speed = node.declare_parameter<double>("max_joint_speed_rps"),
                          .tolerance = node.declare_parameter<double>("tolerance_rad"),
                          .settle_time = positive_duration(node, "settle_time_ns")});
  if (!follower) {
    reject(node, "goals_rad, joint limits, max_joint_speed_rps, or tolerance_rad");
  }
  return std::move(*follower);
}

}  // namespace

ArmControllerNode::ArmControllerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("arm_controller", options),
      clock_{*this},
      joints_{load_joints(*this)},
      control_period_{period(*this, "control_period_ns")},
      commander_{joints_, load_monitor(*this, "feedback_timeout_ns"),
                 load_follower(*this, joints_.size())} {
  std::ranges::transform(joints_, std::back_inserter(commands_), [this](const std::string& joint) {
    return create_publisher<std_msgs::msg::Float64>("arm/" + joint + "/position",
                                                    rclcpp::QoS(kSensorDepth));
  });
  states_ = create_subscription<RobotState>("state",
                                            rclcpp::QoS(rclcpp::KeepLast(kStateDepth)).reliable(),
                                            [this](const RobotState& state) { on_state(state); });
  joint_states_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::QoS(kSensorDepth),
      [this](const sensor_msgs::msg::JointState& state) { on_joint_state(state); });
  control_timer_ =
      create_timer(std::chrono::nanoseconds{control_period_.ns()}, [this] { on_control(); });
  robot_id_ = static_cast<std::uint32_t>(get_parameter("robot_id").as_int());
  progress_ = create_publisher<rovi_interfaces::msg::TaskProgress>(
      "/rovi/task_progress", rclcpp::QoS(rclcpp::KeepLast(kStateDepth)).reliable());
  progress_timer_ = create_timer(std::chrono::nanoseconds{period(*this, "progress_period_ns").ns()},
                                 [this] { on_progress(); });
  settings_ = telemetry::announce_settings(*this);
}

void ArmControllerNode::on_state(const RobotState& state) {
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

void ArmControllerNode::on_joint_state(const sensor_msgs::msg::JointState& state) {
  const std::optional<core::TimePoint> now = clock_.now();
  if (!now) {
    return;
  }
  commander_.on_joint_state(state.name, state.position, *now);
}

void ArmControllerNode::on_control() {
  const std::optional<core::TimePoint> now = clock_.now();
  if (!now) {
    return;
  }
  const std::optional<std::vector<double>> setpoint = commander_.command(*now, control_period_);
  if (!setpoint) {
    return;
  }
  for (std::size_t joint = 0; joint < commands_.size(); ++joint) {
    std_msgs::msg::Float64 command;
    command.data = setpoint->at(joint);
    commands_.at(joint)->publish(command);
  }
}

void ArmControllerNode::on_progress() {
  const std::optional<core::TimePoint> now = clock_.now();
  if (!now) {
    return;
  }
  rovi_interfaces::msg::TaskProgress progress;
  progress.robot_id = robot_id_;
  progress.stamp_ns = now->ns();
  progress.goal_index = static_cast<std::uint32_t>(commander_.goal());
  progress.cycles_completed = commander_.cycles();
  progress.moving = commander_.moving();
  progress_->publish(progress);
}

}  // namespace rovi::robot_agent

RCLCPP_COMPONENTS_REGISTER_NODE(rovi::robot_agent::ArmControllerNode)
