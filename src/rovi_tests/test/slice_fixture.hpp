// In-process vertical-slice composition with a stepped simulation clock and topic probes.
#ifndef ROVI_TESTS_SLICE_FIXTURE_HPP_
#define ROVI_TESTS_SLICE_FIXTURE_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp_components/component_manager.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <rclcpp_components/node_instance_wrapper.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include "rovi_interfaces/msg/component_settings.hpp"
#include "rovi_interfaces/msg/decision.hpp"
#include "rovi_interfaces/msg/job_request.hpp"
#include "rovi_interfaces/msg/payload.hpp"
#include "rovi_interfaces/msg/request_receipt.hpp"
#include "rovi_interfaces/msg/reservation_violation.hpp"
#include "rovi_interfaces/msg/robot_state.hpp"
#include "rovi_interfaces/msg/run_invalidation.hpp"
#include "rovi_interfaces/msg/run_metadata.hpp"
#include "rovi_interfaces/msg/scheduler_cycle.hpp"

namespace rovi::tests {

inline constexpr std::int64_t kStartNs = 1'000'000'000;
inline constexpr std::int64_t kStepNs = 1'000'000;

struct Component {
  std::string package;
  std::string plugin;
  std::string name;
  std::string name_space;
};

using Overrides = std::map<std::string, std::vector<std::string>>;

[[nodiscard]] const std::vector<Component>& vertical_slice();

[[nodiscard]] std::shared_ptr<rclcpp_components::NodeFactory> factory_for(
    rclcpp_components::ComponentManager& manager, const Component& component);

[[nodiscard]] rclcpp::NodeOptions options_for(const Component& component,
                                              const std::vector<std::string>& overrides,
                                              const std::string& scenario = "vertical_slice");

class Slice {
 public:
  explicit Slice(const Overrides& overrides);
  Slice(const Slice&) = delete;
  Slice& operator=(const Slice&) = delete;
  Slice(Slice&&) = delete;
  Slice& operator=(Slice&&) = delete;
  ~Slice();

  void run_until(std::int64_t end_ns);
  void rewind_to(std::int64_t ns);
  void invalidate_from(const std::string& node);
  void submit(const rovi_interfaces::msg::JobRequest& request);
  void upload(const rovi_interfaces::msg::Payload& observation);

  [[nodiscard]] const std::vector<rovi_interfaces::msg::Decision>& decisions() const;
  [[nodiscard]] const std::vector<rovi_interfaces::msg::Payload>& chunks() const;
  [[nodiscard]] const std::vector<rovi_interfaces::msg::ReservationViolation>& violations() const;
  [[nodiscard]] const std::vector<rovi_interfaces::msg::SchedulerCycle>& cycles() const;
  [[nodiscard]] const std::optional<rovi_interfaces::msg::RobotState>& last_state() const;
  [[nodiscard]] const std::optional<rovi_interfaces::msg::RunMetadata>& metadata() const;
  [[nodiscard]] const std::vector<rovi_interfaces::msg::RequestReceipt>& receipts() const;
  [[nodiscard]] const std::vector<rovi_interfaces::msg::RunInvalidation>& invalidations() const;
  [[nodiscard]] const std::vector<rovi_interfaces::msg::ComponentSettings>& settings() const;

 private:
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<rclcpp_components::ComponentManager> manager_;
  std::vector<rclcpp_components::NodeInstanceWrapper> instances_;
  rclcpp::Node::SharedPtr probe_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_;
  rclcpp::Publisher<rovi_interfaces::msg::JobRequest>::SharedPtr requests_;
  rclcpp::Publisher<rovi_interfaces::msg::Payload>::SharedPtr uplink_;
  rclcpp::Publisher<rovi_interfaces::msg::RunInvalidation>::SharedPtr invalidations_out_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::vector<rovi_interfaces::msg::Decision> decisions_;
  std::vector<rovi_interfaces::msg::Payload> chunks_;
  std::vector<rovi_interfaces::msg::ReservationViolation> violations_;
  std::vector<rovi_interfaces::msg::SchedulerCycle> cycles_;
  std::optional<rovi_interfaces::msg::RobotState> last_state_;
  std::optional<rovi_interfaces::msg::RunMetadata> metadata_;
  std::vector<rovi_interfaces::msg::RequestReceipt> receipts_;
  std::vector<rovi_interfaces::msg::RunInvalidation> invalidations_;
  std::vector<rovi_interfaces::msg::ComponentSettings> settings_;
  std::int64_t next_ns_{kStartNs};
};

}  // namespace rovi::tests

#endif  // ROVI_TESTS_SLICE_FIXTURE_HPP_
