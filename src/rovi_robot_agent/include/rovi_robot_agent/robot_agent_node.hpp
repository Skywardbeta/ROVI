// ROS 2 component that runs one robot's action buffer, job generation, and fallback state.
#ifndef ROVI_ROBOT_AGENT_ROBOT_AGENT_NODE_HPP_
#define ROVI_ROBOT_AGENT_ROBOT_AGENT_NODE_HPP_

#include <cstdint>
#include <optional>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>

#include "rovi_core/units.hpp"
#include "rovi_interfaces/msg/decision.hpp"
#include "rovi_interfaces/msg/group_release.hpp"
#include "rovi_interfaces/msg/job_request.hpp"
#include "rovi_interfaces/msg/payload.hpp"
#include "rovi_interfaces/msg/request_receipt.hpp"
#include "rovi_interfaces/msg/robot_state.hpp"
#include "rovi_policy/wire.hpp"
#include "rovi_robot_agent/robot_agent.hpp"
#include "rovi_telemetry/run_clock.hpp"

namespace rovi::robot_agent {

class RobotAgentNode : public rclcpp::Node {
 public:
  explicit RobotAgentNode(const rclcpp::NodeOptions& options);

 private:
  void publish_receipt(std::uint8_t event, core::JobId job, core::TimePoint stamp,
                       core::TimePoint deadline);
  RobotAgent* agent(core::TimePoint now);
  void on_observe();
  void on_decision(const rovi_interfaces::msg::Decision& decision);
  void on_chunk(const rovi_interfaces::msg::Payload& payload);
  void on_release(const rovi_interfaces::msg::GroupRelease& release);
  void on_state();

  telemetry::RunClock clock_;
  AgentConfig config_;
  std::optional<policy::Observation> recorded_;
  std::optional<RobotAgent> agent_;
  rclcpp::Publisher<rovi_interfaces::msg::JobRequest>::SharedPtr requests_;
  rclcpp::Publisher<rovi_interfaces::msg::Payload>::SharedPtr observations_;
  rclcpp::Publisher<rovi_interfaces::msg::RobotState>::SharedPtr states_;
  rclcpp::Publisher<rovi_interfaces::msg::RobotState>::SharedPtr own_state_;
  rclcpp::Subscription<rovi_interfaces::msg::Decision>::SharedPtr decisions_;
  rclcpp::Subscription<rovi_interfaces::msg::Payload>::SharedPtr chunks_;
  rclcpp::Subscription<rovi_interfaces::msg::GroupRelease>::SharedPtr releases_;
  rclcpp::TimerBase::SharedPtr observe_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::Publisher<rovi_interfaces::msg::RequestReceipt>::SharedPtr receipts_;
  rclcpp::Publisher<rovi_interfaces::msg::ComponentSettings>::SharedPtr settings_;
};

}  // namespace rovi::robot_agent

#endif  // ROVI_ROBOT_AGENT_ROBOT_AGENT_NODE_HPP_
