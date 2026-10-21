// ROS 2 component that runs admitted jobs on a remote policy server, one at a time, in their
// reserved compute slots.
#ifndef ROVI_POLICY_POLICY_WORKER_NODE_HPP_
#define ROVI_POLICY_POLICY_WORKER_NODE_HPP_

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <stop_token>
#include <thread>
#include <vector>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>

#include "rovi_core/units.hpp"
#include "rovi_interfaces/msg/decision.hpp"
#include "rovi_interfaces/msg/inference_record.hpp"
#include "rovi_interfaces/msg/payload.hpp"
#include "rovi_interfaces/msg/policy_server.hpp"
#include "rovi_policy/policy_client.hpp"
#include "rovi_policy/policy_lane.hpp"
#include "rovi_policy/wire.hpp"
#include "rovi_telemetry/run_clock.hpp"

namespace rovi::policy {

class PolicyWorkerNode : public rclcpp::Node {
 public:
  explicit PolicyWorkerNode(const rclcpp::NodeOptions& options);

 private:
  struct Call {
    LaneJob job;
    core::TimePoint observation_time;
    core::TimePoint requested;
    Request request;
  };

  struct Finished {
    Call call;
    CallResult result;
    std::int64_t round_trip_ns{0};
    std::int64_t action_period_ns{0};
    std::uint32_t connection{0};
    std::optional<Hello> server;
  };

  void on_decision(const rovi_interfaces::msg::Decision& decision);
  void on_observation(const rovi_interfaces::msg::Payload& payload);
  void on_poll();
  void complete(const Finished& finished, core::TimePoint now);
  void call(const std::stop_token& stop);

  telemetry::RunClock clock_;
  core::WorkerId id_;
  std::vector<core::ModelClass> models_;
  PolicyId policy_{};
  PolicyLane lane_;
  Endpoint endpoint_;
  PolicyClient client_;
  std::uint64_t reported_discards_{0};
  rclcpp::Publisher<rovi_interfaces::msg::Payload>::SharedPtr chunks_;
  rclcpp::Publisher<rovi_interfaces::msg::InferenceRecord>::SharedPtr records_;
  rclcpp::Publisher<rovi_interfaces::msg::PolicyServer>::SharedPtr servers_;
  rclcpp::Subscription<rovi_interfaces::msg::Decision>::SharedPtr decisions_;
  rclcpp::Subscription<rovi_interfaces::msg::Payload>::SharedPtr observations_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::Publisher<rovi_interfaces::msg::ComponentSettings>::SharedPtr settings_;
  std::mutex mutex_;
  std::condition_variable_any wake_;
  std::optional<Call> pending_;
  std::optional<Finished> finished_;
  // Used only by the inference thread.
  std::uint32_t announced_connections_{0};
  // Declared last so the inference thread stops before anything it uses is destroyed.
  std::jthread caller_;
};

}  // namespace rovi::policy

#endif  // ROVI_POLICY_POLICY_WORKER_NODE_HPP_
