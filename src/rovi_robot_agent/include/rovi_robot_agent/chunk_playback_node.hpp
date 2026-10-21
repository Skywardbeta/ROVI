// ROS 2 component that plays a robot's accepted action chunks onto its position-controlled joints.
#ifndef ROVI_ROBOT_AGENT_CHUNK_PLAYBACK_NODE_HPP_
#define ROVI_ROBOT_AGENT_CHUNK_PLAYBACK_NODE_HPP_

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

#include "rovi_interfaces/msg/action_chunk.hpp"
#include "rovi_interfaces/msg/robot_state.hpp"
#include "rovi_robot_agent/chunk_commander.hpp"
#include "rovi_telemetry/run_clock.hpp"

namespace rovi::robot_agent {

class ChunkPlaybackNode : public rclcpp::Node {
 public:
  explicit ChunkPlaybackNode(const rclcpp::NodeOptions& options);

 private:
  void on_chunk(const rovi_interfaces::msg::ActionChunk& chunk);
  void on_state(const rovi_interfaces::msg::RobotState& state);
  void on_joint_state(const sensor_msgs::msg::JointState& state);
  void on_control();

  telemetry::RunClock clock_;
  std::vector<std::string> joints_;
  std::uint32_t robot_id_{0};
  ChunkCommander commander_;
  std::uint64_t clamped_commands_{0};
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> commands_;
  rclcpp::Subscription<rovi_interfaces::msg::ActionChunk>::SharedPtr chunks_;
  rclcpp::Subscription<rovi_interfaces::msg::RobotState>::SharedPtr states_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Publisher<rovi_interfaces::msg::ComponentSettings>::SharedPtr settings_;
};

}  // namespace rovi::robot_agent

#endif  // ROVI_ROBOT_AGENT_CHUNK_PLAYBACK_NODE_HPP_
