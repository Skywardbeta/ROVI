// Lock-stepped in-process headless Gazebo scenario with the bridge, ROVI components, and probes.
#ifndef ROVI_TESTS_GAZEBO_FIXTURE_HPP_
#define ROVI_TESTS_GAZEBO_FIXTURE_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gz/sim/Server.hh>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp_components/component_manager.hpp>
#include <rclcpp_components/node_instance_wrapper.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "rovi_interfaces/msg/decision.hpp"
#include "rovi_interfaces/msg/group_release.hpp"
#include "rovi_interfaces/msg/job_request.hpp"
#include "rovi_interfaces/msg/payload.hpp"
#include "rovi_interfaces/msg/robot_state.hpp"
#include "rovi_interfaces/msg/run_metadata.hpp"
#include "rovi_interfaces/msg/task_progress.hpp"

#include "slice_fixture.hpp"

namespace rovi::tests {

inline constexpr int kArmJoints = 7;

struct GazeboScenarioSpec {
  std::string world;
  std::string entity;
  std::string bridge_config;
  std::string parameters;
  std::vector<Component> components;
  std::vector<std::string> joint_state_topics;
  std::vector<std::string> command_topics;
  std::int64_t physics_step_ns{1'000'000};
};

struct JointSample {
  std::size_t topic{0};
  sensor_msgs::msg::JointState state;
};

struct CommandSample {
  std::size_t topic{0};
  std::int64_t received_ns{0};
  double value{0.0};
};

struct OdometrySample {
  std::int64_t stamp_ns{0};
  double x{0.0};
  double y{0.0};
  double linear_speed{0.0};
};

[[nodiscard]] GazeboScenarioSpec mobile_robot_scenario();
[[nodiscard]] GazeboScenarioSpec arm_workcell_scenario();
[[nodiscard]] GazeboScenarioSpec thirty_arm_scenario();

[[nodiscard]] std::vector<std::pair<std::int64_t, std::int64_t>> fallback_intervals(
    const std::vector<rovi_interfaces::msg::RobotState>& states, std::uint32_t robot);

class GazeboScenario {
 public:
  explicit GazeboScenario(const GazeboScenarioSpec& spec);
  GazeboScenario(const GazeboScenario&) = delete;
  GazeboScenario& operator=(const GazeboScenario&) = delete;
  GazeboScenario(GazeboScenario&&) = delete;
  GazeboScenario& operator=(GazeboScenario&&) = delete;
  ~GazeboScenario();

  // Steps to absolute sim time sim_ns, letting callbacks settle after each step so components keep
  // pace with the simulation. Fails after wall_limit of host time.
  [[nodiscard]] bool run_until(std::int64_t sim_ns, std::chrono::seconds wall_limit);

  [[nodiscard]] const std::string& failure() const;
  [[nodiscard]] const std::vector<OdometrySample>& odometry() const;
  [[nodiscard]] const std::vector<JointSample>& joint_samples() const;
  [[nodiscard]] const std::vector<rovi_interfaces::msg::JobRequest>& requests() const;
  [[nodiscard]] const std::vector<rovi_interfaces::msg::Decision>& decisions() const;
  [[nodiscard]] const std::vector<rovi_interfaces::msg::GroupRelease>& releases() const;
  [[nodiscard]] const std::vector<rovi_interfaces::msg::TaskProgress>& progress() const;
  [[nodiscard]] const std::vector<CommandSample>& commands() const;
  [[nodiscard]] const std::vector<rovi_interfaces::msg::RobotState>& states() const;
  [[nodiscard]] const std::vector<rovi_interfaces::msg::Payload>& chunks() const;
  [[nodiscard]] const std::optional<rovi_interfaces::msg::RunMetadata>& metadata() const;

 private:
  std::unique_ptr<gz::sim::Server> server_;
  bool connect(std::chrono::steady_clock::time_point deadline);

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<rclcpp_components::ComponentManager> manager_;
  std::optional<rclcpp_components::NodeInstanceWrapper> bridge_;
  std::vector<rclcpp_components::NodeInstanceWrapper> instances_;
  rclcpp::Node::SharedPtr probe_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::int64_t clock_ns_{0};
  std::int64_t stepped_ns_{0};
  std::uint64_t iterations_{0};
  std::string failure_;
  std::vector<OdometrySample> odometry_;
  std::vector<JointSample> joint_samples_;
  std::vector<rovi_interfaces::msg::JobRequest> requests_;
  std::vector<rovi_interfaces::msg::Decision> decisions_;
  std::vector<rovi_interfaces::msg::GroupRelease> releases_;
  std::vector<rovi_interfaces::msg::TaskProgress> progress_;
  std::vector<CommandSample> commands_;
  std::vector<rovi_interfaces::msg::RobotState> states_;
  std::vector<rovi_interfaces::msg::Payload> chunks_;
  std::optional<rovi_interfaces::msg::RunMetadata> metadata_;
};

}  // namespace rovi::tests

#endif  // ROVI_TESTS_GAZEBO_FIXTURE_HPP_
