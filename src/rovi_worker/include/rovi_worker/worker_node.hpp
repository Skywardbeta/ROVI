// ROS 2 component that executes admitted inference jobs on one simulated edge-worker lane.
#ifndef ROVI_WORKER_WORKER_NODE_HPP_
#define ROVI_WORKER_WORKER_NODE_HPP_

#include <cstdint>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>

#include "rovi_core/units.hpp"
#include "rovi_interfaces/msg/decision.hpp"
#include "rovi_interfaces/msg/payload.hpp"
#include "rovi_telemetry/run_clock.hpp"
#include "rovi_worker/worker_sim.hpp"

namespace rovi::worker {

class WorkerNode : public rclcpp::Node {
 public:
  explicit WorkerNode(const rclcpp::NodeOptions& options);

 private:
  void on_decision(const rovi_interfaces::msg::Decision& decision);
  void on_observation(const rovi_interfaces::msg::Payload& payload);
  void on_poll();

  telemetry::RunClock clock_;
  core::WorkerId id_;
  WorkerSim sim_;
  std::uint64_t reported_discards_{0};
  rclcpp::Publisher<rovi_interfaces::msg::Payload>::SharedPtr chunks_;
  rclcpp::Subscription<rovi_interfaces::msg::Decision>::SharedPtr decisions_;
  rclcpp::Subscription<rovi_interfaces::msg::Payload>::SharedPtr observations_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::Publisher<rovi_interfaces::msg::ComponentSettings>::SharedPtr settings_;
};

}  // namespace rovi::worker

#endif  // ROVI_WORKER_WORKER_NODE_HPP_
