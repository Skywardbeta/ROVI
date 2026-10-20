#include "slice_fixture.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcl/time.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
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
namespace {

using rovi_interfaces::msg::ComponentSettings;
using rovi_interfaces::msg::Decision;
using rovi_interfaces::msg::Payload;
using rovi_interfaces::msg::RequestReceipt;
using rovi_interfaces::msg::ReservationViolation;
using rovi_interfaces::msg::RobotState;
using rovi_interfaces::msg::RunInvalidation;
using rovi_interfaces::msg::RunMetadata;
using rovi_interfaces::msg::SchedulerCycle;

constexpr std::size_t kProbeDepth = 1'000;
constexpr std::chrono::seconds kSpinBudget{10};

rclcpp::QoS events() { return rclcpp::QoS(rclcpp::KeepLast(kProbeDepth)).reliable(); }

}  // namespace

const std::vector<Component>& vertical_slice() {
  static const std::vector<Component> components{{.package = "rovi_scheduler",
                                                  .plugin = "rovi::scheduler::SchedulerNode",
                                                  .name = "rovi_scheduler",
                                                  .name_space = "/"},
                                                 {.package = "rovi_network",
                                                  .plugin = "rovi::network::LinkNode",
                                                  .name = "rovi_link",
                                                  .name_space = "/"},
                                                 {.package = "rovi_worker",
                                                  .plugin = "rovi::worker::WorkerNode",
                                                  .name = "rovi_worker_1",
                                                  .name_space = "/"},
                                                 {.package = "rovi_robot_agent",
                                                  .plugin = "rovi::robot_agent::RobotAgentNode",
                                                  .name = "robot_agent",
                                                  .name_space = "/robot_00"}};
  return components;
}

std::shared_ptr<rclcpp_components::NodeFactory> factory_for(
    rclcpp_components::ComponentManager& manager, const Component& component) {
  const auto resources = manager.get_component_resources(component.package);
  const auto resource =
      std::ranges::find(resources, component.plugin, [](const auto& entry) { return entry.first; });
  if (resource == resources.end()) {
    throw std::runtime_error{"component not registered: " + component.plugin};
  }
  return manager.create_component_factory(*resource);
}

rclcpp::NodeOptions options_for(const Component& component,
                                const std::vector<std::string>& overrides,
                                const std::string& scenario) {
  const std::string params = ament_index_cpp::get_package_share_directory("rovi_bringup") +
                             "/config/" + scenario + ".yaml";
  std::vector<std::string> arguments{"--ros-args",
                                     "-r",
                                     "__node:=" + component.name,
                                     "-r",
                                     "__ns:=" + component.name_space,
                                     "--params-file",
                                     params,
                                     "-p",
                                     "use_sim_time:=true"};
  for (const std::string& assignment : overrides) {
    arguments.emplace_back("-p");
    arguments.push_back(assignment);
  }
  return rclcpp::NodeOptions()
      .arguments(arguments)
      .use_global_arguments(false)
      .use_intra_process_comms(true)
      .use_clock_thread(false);
}

Slice::Slice(const Overrides& overrides)
    : executor_{std::make_shared<rclcpp::executors::SingleThreadedExecutor>()},
      manager_{std::make_shared<rclcpp_components::ComponentManager>(executor_)} {
  for (const Component& component : vertical_slice()) {
    const auto found = overrides.find(component.name);
    instances_.push_back(
        factory_for(*manager_, component)
            ->create_node_instance(options_for(
                component, found == overrides.end() ? std::vector<std::string>{} : found->second)));
    executor_->add_node(instances_.back().get_node_base_interface());
  }
  probe_ = std::make_shared<rclcpp::Node>("rovi_probe",
                                          rclcpp::NodeOptions().use_intra_process_comms(true));
  clock_ = probe_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
  requests_ =
      probe_->create_publisher<rovi_interfaces::msg::JobRequest>("/rovi/job_requests", events());
  uplink_ = probe_->create_publisher<Payload>("/rovi/uplink/sent", events());
  invalidations_out_ = probe_->create_publisher<RunInvalidation>(
      "/rovi/run/invalidations", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  subscriptions_.push_back(probe_->create_subscription<Decision>(
      "/rovi/decisions", events(),
      [this](const Decision& message) { decisions_.push_back(message); }));
  subscriptions_.push_back(probe_->create_subscription<Payload>(
      "/rovi/downlink/delivered", events(),
      [this](const Payload& message) { chunks_.push_back(message); }));
  subscriptions_.push_back(probe_->create_subscription<ReservationViolation>(
      "/rovi/link/violations", events(),
      [this](const ReservationViolation& message) { violations_.push_back(message); }));
  subscriptions_.push_back(probe_->create_subscription<SchedulerCycle>(
      "/rovi/scheduler/cycles", events(),
      [this](const SchedulerCycle& message) { cycles_.push_back(message); }));
  subscriptions_.push_back(probe_->create_subscription<RobotState>(
      "/rovi/robot_state", events(), [this](const RobotState& message) { last_state_ = message; }));
  subscriptions_.push_back(probe_->create_subscription<RunMetadata>(
      "/rovi/run_metadata", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](const RunMetadata& message) { metadata_ = message; }));
  const rclcpp::QoS latched =
      rclcpp::QoS(rclcpp::KeepLast(kProbeDepth)).reliable().transient_local();
  subscriptions_.push_back(probe_->create_subscription<RequestReceipt>(
      "/rovi/requests/receipts", events(),
      [this](const RequestReceipt& message) { receipts_.push_back(message); }));
  subscriptions_.push_back(probe_->create_subscription<RunInvalidation>(
      "/rovi/run/invalidations", latched,
      [this](const RunInvalidation& message) { invalidations_.push_back(message); }));
  subscriptions_.push_back(probe_->create_subscription<ComponentSettings>(
      "/rovi/run/components", latched,
      [this](const ComponentSettings& message) { settings_.push_back(message); }));
  executor_->add_node(probe_);
}

Slice::~Slice() {
  executor_->remove_node(probe_);
  for (rclcpp_components::NodeInstanceWrapper& instance : instances_) {
    executor_->remove_node(instance.get_node_base_interface());
  }
}

void Slice::run_until(std::int64_t end_ns) {
  for (; next_ns_ <= end_ns; next_ns_ += kStepNs) {
    rosgraph_msgs::msg::Clock tick;
    tick.clock = rclcpp::Time(next_ns_, RCL_ROS_TIME);
    clock_->publish(tick);
    executor_->spin_all(kSpinBudget);
  }
}

void Slice::rewind_to(std::int64_t ns) { next_ns_ = ns; }

void Slice::invalidate_from(const std::string& node) {
  RunInvalidation invalidation;
  invalidation.node = node;
  invalidation.latest_ns = next_ns_ - kStepNs;
  invalidation.observed_ns = kStartNs;
  invalidations_out_->publish(invalidation);
  executor_->spin_all(kSpinBudget);
}

void Slice::submit(const rovi_interfaces::msg::JobRequest& request) { requests_->publish(request); }

void Slice::upload(const Payload& observation) { uplink_->publish(observation); }

const std::vector<Decision>& Slice::decisions() const { return decisions_; }

const std::vector<Payload>& Slice::chunks() const { return chunks_; }

const std::vector<ReservationViolation>& Slice::violations() const { return violations_; }

const std::vector<SchedulerCycle>& Slice::cycles() const { return cycles_; }

const std::optional<RobotState>& Slice::last_state() const { return last_state_; }

const std::optional<RunMetadata>& Slice::metadata() const { return metadata_; }

const std::vector<RequestReceipt>& Slice::receipts() const { return receipts_; }

const std::vector<RunInvalidation>& Slice::invalidations() const { return invalidations_; }

const std::vector<ComponentSettings>& Slice::settings() const { return settings_; }

}  // namespace rovi::tests
