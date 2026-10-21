#include "rovi_robot_agent/chunk_playback_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include "rovi_core/units.hpp"
#include "rovi_interfaces/msg/action_chunk.hpp"
#include "rovi_interfaces/msg/robot_state.hpp"
#include "rovi_robot_agent/chunk_commander.hpp"
#include "rovi_robot_agent/chunk_player.hpp"
#include "rovi_robot_agent/joint_calibration.hpp"
#include "rovi_robot_agent/motion_monitor.hpp"
#include "rovi_robot_agent/node_parameters.hpp"
#include "rovi_telemetry/run_clock.hpp"

namespace rovi::robot_agent {
namespace {

using rovi_interfaces::msg::ActionChunk;
using rovi_interfaces::msg::RobotState;

constexpr std::size_t kStateDepth = 10;
constexpr std::size_t kSensorDepth = 10;
constexpr int kWarningThrottleMs = 1'000;

std::vector<double> doubles(rclcpp::Node& node, const std::string& name, std::size_t count) {
  std::vector<double> values = node.declare_parameter<std::vector<double>>(name);
  if (values.size() != count) {
    reject(node, name);
  }
  return values;
}

CalibrationMap load_calibration(rclcpp::Node& node, std::size_t joints) {
  const std::vector<double> signs = doubles(node, "joint_signs", joints);
  const std::vector<double> zeros = doubles(node, "joint_zero_deg", joints);
  const std::vector<double> lower = doubles(node, "joint_lower_rad", joints);
  const std::vector<double> upper = doubles(node, "joint_upper_rad", joints);
  std::vector<JointCalibration> calibration(joints);
  for (std::size_t joint = 0; joint < joints; ++joint) {
    calibration.at(joint) = JointCalibration{.sign = signs.at(joint),
                                             .zero_deg = zeros.at(joint),
                                             .lower_rad = lower.at(joint),
                                             .upper_rad = upper.at(joint)};
  }
  std::optional<CalibrationMap> map = CalibrationMap::create(std::move(calibration));
  if (!map) {
    reject(node, "joint_signs, joint_zero_deg, joint_lower_rad, or joint_upper_rad");
  }
  return std::move(*map);
}

}  // namespace

ChunkPlaybackNode::ChunkPlaybackNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("chunk_playback", options),
      clock_{*this},
      joints_{load_joints(*this)},
      commander_{joints_, load_monitor(*this, "feedback_timeout_ns"),
                 load_calibration(*this, joints_.size())} {
  robot_id_ = static_cast<std::uint32_t>(get_parameter("robot_id").as_int());
  const core::Duration control_period = period(*this, "control_period_ns");
  std::ranges::transform(joints_, std::back_inserter(commands_), [this](const std::string& joint) {
    return create_publisher<std_msgs::msg::Float64>("arm/" + joint + "/position",
                                                    rclcpp::QoS(kSensorDepth));
  });
  const rclcpp::QoS reliable = rclcpp::QoS(rclcpp::KeepLast(kStateDepth)).reliable();
  chunks_ = create_subscription<ActionChunk>("action_chunk", reliable,
                                             [this](const ActionChunk& chunk) { on_chunk(chunk); });
  states_ = create_subscription<RobotState>("state", reliable,
                                            [this](const RobotState& state) { on_state(state); });
  joint_states_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::QoS(kSensorDepth),
      [this](const sensor_msgs::msg::JointState& state) { on_joint_state(state); });
  control_timer_ =
      create_timer(std::chrono::nanoseconds{control_period.ns()}, [this] { on_control(); });
  settings_ = telemetry::announce_settings(*this);
}

void ChunkPlaybackNode::on_chunk(const ActionChunk& chunk) {
  if (!clock_.now() || chunk.robot_id != robot_id_) {
    return;
  }
  const bool queued =
      commander_.on_chunk(PlayedChunk{.start = core::TimePoint::from_ns(chunk.start_ns),
                                      .period = core::Duration::from_ns(chunk.period_ns),
                                      .steps = chunk.steps,
                                      .dim = chunk.dim,
                                      .actions = chunk.actions});
  if (!queued) {
    RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), kWarningThrottleMs,
                                "cannot play the action chunk of job " << chunk.job_id);
  }
}

void ChunkPlaybackNode::on_state(const RobotState& state) {
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

void ChunkPlaybackNode::on_joint_state(const sensor_msgs::msg::JointState& state) {
  const std::optional<core::TimePoint> now = clock_.now();
  if (!now) {
    return;
  }
  commander_.on_joint_state(state.name, state.position, *now);
}

void ChunkPlaybackNode::on_control() {
  const std::optional<core::TimePoint> now = clock_.now();
  if (!now) {
    return;
  }
  const std::optional<std::vector<double>> setpoint = commander_.command(*now);
  if (!setpoint) {
    return;
  }
  if (commander_.clamped_commands() != clamped_commands_) {
    clamped_commands_ = commander_.clamped_commands();
    RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), kWarningThrottleMs,
                                clamped_commands_ << " commands clamped to the joint limits");
  }
  for (std::size_t joint = 0; joint < commands_.size(); ++joint) {
    std_msgs::msg::Float64 command;
    command.data = setpoint->at(joint);
    commands_.at(joint)->publish(command);
  }
}

}  // namespace rovi::robot_agent

RCLCPP_COMPONENTS_REGISTER_NODE(rovi::robot_agent::ChunkPlaybackNode)
