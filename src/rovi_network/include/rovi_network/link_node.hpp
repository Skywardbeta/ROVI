// ROS 2 component that carries payloads over the logical wireless service at reserved times.
#ifndef ROVI_NETWORK_LINK_NODE_HPP_
#define ROVI_NETWORK_LINK_NODE_HPP_

#include <cstdint>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>

#include "rovi_core/units.hpp"
#include "rovi_interfaces/msg/decision.hpp"
#include "rovi_interfaces/msg/payload.hpp"
#include "rovi_interfaces/msg/reservation_violation.hpp"
#include "rovi_network/logical_link.hpp"
#include "rovi_telemetry/run_clock.hpp"

namespace rovi::network {

class LinkNode : public rclcpp::Node {
 public:
  explicit LinkNode(const rclcpp::NodeOptions& options);

 private:
  void on_decision(const rovi_interfaces::msg::Decision& decision);
  void on_sent(const rovi_interfaces::msg::Payload& payload, Direction direction);
  void on_poll();
  void report(const Refusal& refusal);

  telemetry::RunClock clock_;
  LogicalLink<rovi_interfaces::msg::Payload> link_;
  std::uint64_t reported_unused_{0};
  rclcpp::Publisher<rovi_interfaces::msg::Payload>::SharedPtr uplink_delivered_;
  rclcpp::Publisher<rovi_interfaces::msg::Payload>::SharedPtr downlink_delivered_;
  rclcpp::Publisher<rovi_interfaces::msg::ReservationViolation>::SharedPtr violations_;
  rclcpp::Subscription<rovi_interfaces::msg::Decision>::SharedPtr decisions_;
  rclcpp::Subscription<rovi_interfaces::msg::Payload>::SharedPtr uplink_sent_;
  rclcpp::Subscription<rovi_interfaces::msg::Payload>::SharedPtr downlink_sent_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::Publisher<rovi_interfaces::msg::ComponentSettings>::SharedPtr settings_;
};

}  // namespace rovi::network

#endif  // ROVI_NETWORK_LINK_NODE_HPP_
