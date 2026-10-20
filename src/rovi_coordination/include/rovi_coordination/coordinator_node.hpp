// ROS 2 component that assembles workcell groups and releases their action chunks together.
#ifndef ROVI_COORDINATION_COORDINATOR_NODE_HPP_
#define ROVI_COORDINATION_COORDINATOR_NODE_HPP_

#include <map>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>

#include "rovi_coordination/group.hpp"
#include "rovi_coordination/group_assembler.hpp"
#include "rovi_coordination/group_releaser.hpp"
#include "rovi_core/units.hpp"
#include "rovi_interfaces/msg/decision.hpp"
#include "rovi_interfaces/msg/group_release.hpp"
#include "rovi_interfaces/msg/job_request.hpp"
#include "rovi_interfaces/msg/payload.hpp"
#include "rovi_telemetry/run_clock.hpp"

namespace rovi::coordination {

class CoordinatorNode : public rclcpp::Node {
 public:
  explicit CoordinatorNode(const rclcpp::NodeOptions& options);

 private:
  void on_proposal(const rovi_interfaces::msg::JobRequest& request);
  void on_decision(const rovi_interfaces::msg::Decision& decision);
  void on_delivery(const rovi_interfaces::msg::Payload& payload);
  void on_expiry();
  void submit(const AssembledGroup& group, core::TimePoint now);
  void publish(const ReleaseOutcome& outcome);

  telemetry::RunClock clock_;
  GroupAssembler assembler_;
  GroupReleaser releaser_;
  std::map<core::RobotId, rovi_interfaces::msg::JobRequest> proposals_;
  rclcpp::Publisher<rovi_interfaces::msg::JobRequest>::SharedPtr requests_;
  rclcpp::Publisher<rovi_interfaces::msg::GroupRelease>::SharedPtr releases_;
  rclcpp::Subscription<rovi_interfaces::msg::JobRequest>::SharedPtr proposal_subscription_;
  rclcpp::Subscription<rovi_interfaces::msg::Decision>::SharedPtr decisions_;
  rclcpp::Subscription<rovi_interfaces::msg::Payload>::SharedPtr deliveries_;
  rclcpp::TimerBase::SharedPtr expiry_timer_;
  rclcpp::Publisher<rovi_interfaces::msg::ComponentSettings>::SharedPtr settings_;
};

}  // namespace rovi::coordination

#endif  // ROVI_COORDINATION_COORDINATOR_NODE_HPP_
