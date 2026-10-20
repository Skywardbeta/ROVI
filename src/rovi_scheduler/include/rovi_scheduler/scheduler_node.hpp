// ROS 2 component that runs rolling-horizon rovi_core admission cycles on the node clock.
#ifndef ROVI_SCHEDULER_SCHEDULER_NODE_HPP_
#define ROVI_SCHEDULER_SCHEDULER_NODE_HPP_

#include <cstdint>
#include <optional>
#include <string>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>

#include "rovi_core/admission.hpp"
#include "rovi_core/units.hpp"
#include "rovi_interfaces/msg/decision.hpp"
#include "rovi_interfaces/msg/job_request.hpp"
#include "rovi_interfaces/msg/payload.hpp"
#include "rovi_interfaces/msg/run_metadata.hpp"
#include "rovi_interfaces/msg/scheduler_cycle.hpp"
#include "rovi_scheduler/config.hpp"
#include "rovi_scheduler/job_book.hpp"
#include "rovi_telemetry/run_clock.hpp"

namespace rovi::scheduler {

struct RunIdentity {
  std::string run_id;
  std::uint64_t seed{0};
  std::string git_revision;
  std::string network_profile_id;
};

class SchedulerNode : public rclcpp::Node {
 public:
  explicit SchedulerNode(const rclcpp::NodeOptions& options);

 private:
  struct Window {
    core::TimePoint epoch;
    core::ResourceSnapshot snapshot;
  };

  void on_request(const rovi_interfaces::msg::JobRequest& request);
  void on_delivered(const rovi_interfaces::msg::Payload& payload);
  void on_cycle();
  bool slide_window(core::TimePoint now);
  void publish_notice(const Notice& notice, const core::ResourceSnapshot* snapshot,
                      core::TimePoint now);
  void publish_metadata(core::TimePoint epoch);

  telemetry::RunClock clock_;
  SchedulerConfig config_;
  RunIdentity run_;
  JobBook book_;
  std::optional<Window> window_;
  std::uint64_t cycle_{0};
  rclcpp::Publisher<rovi_interfaces::msg::Decision>::SharedPtr decisions_;
  rclcpp::Publisher<rovi_interfaces::msg::SchedulerCycle>::SharedPtr cycles_;
  rclcpp::Publisher<rovi_interfaces::msg::RunMetadata>::SharedPtr metadata_;
  rclcpp::Subscription<rovi_interfaces::msg::JobRequest>::SharedPtr requests_;
  rclcpp::Subscription<rovi_interfaces::msg::Payload>::SharedPtr delivered_;
  rclcpp::TimerBase::SharedPtr cycle_timer_;
  rclcpp::Publisher<rovi_interfaces::msg::ComponentSettings>::SharedPtr settings_;
};

}  // namespace rovi::scheduler

#endif  // ROVI_SCHEDULER_SCHEDULER_NODE_HPP_
