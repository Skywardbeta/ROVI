#include "gazebo_fixture.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/Util.hh>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_components/component_manager.hpp>
#include <rclcpp_components/node_instance_wrapper.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include "rovi_interfaces/msg/decision.hpp"
#include "rovi_interfaces/msg/group_release.hpp"
#include "rovi_interfaces/msg/job_request.hpp"
#include "rovi_interfaces/msg/payload.hpp"
#include "rovi_interfaces/msg/robot_state.hpp"
#include "rovi_interfaces/msg/run_metadata.hpp"
#include "rovi_interfaces/msg/task_progress.hpp"

#include "slice_fixture.hpp"

namespace rovi::tests {
namespace {

using rovi_interfaces::msg::Decision;
using rovi_interfaces::msg::GroupRelease;
using rovi_interfaces::msg::JobRequest;
using rovi_interfaces::msg::Payload;
using rovi_interfaces::msg::RobotState;
using rovi_interfaces::msg::RunMetadata;
using rovi_interfaces::msg::TaskProgress;

constexpr std::size_t kProbeDepth = 1'000;
constexpr std::chrono::milliseconds kSettle{3};
constexpr std::chrono::milliseconds kDrainLimit{500};
constexpr std::chrono::milliseconds kDiscoveryWait{50};
constexpr std::int64_t kMillisecondNs = 1'000'000;
constexpr int kTwoDigits = 10;
constexpr int kThirtyArms = 30;
constexpr int kThirtyArmWorkers = 3;
constexpr std::int64_t kGazeboStepNs = 10'000'000;

std::string share(const std::string& package) {
  return ament_index_cpp::get_package_share_directory(package);
}

rclcpp::NodeOptions options_with(const Component& component, std::vector<std::string> extra) {
  std::vector<std::string> arguments{"--ros-args", "-r", "__node:=" + component.name, "-r",
                                     "__ns:=" + component.name_space};
  arguments.insert(arguments.end(), extra.begin(), extra.end());
  return rclcpp::NodeOptions().arguments(arguments).use_global_arguments(false);
}

std::vector<std::string> rovi_arguments(const GazeboScenarioSpec& spec) {
  return {"--params-file", share("rovi_bringup") + "/config/" + spec.parameters, "-p",
          "use_sim_time:=true"};
}

rclcpp::NodeOptions bridge_options(const Component& bridge, const GazeboScenarioSpec& spec) {
  return options_with(
      bridge, {"-p", "config_file:=" + share("rovi_sim") + "/config/" + spec.bridge_config});
}

const Component& bridge_component() {
  static const Component bridge{.package = "ros_gz_bridge",
                                .plugin = "ros_gz_bridge::RosGzBridge",
                                .name = "gz_bridge",
                                .name_space = "/"};
  return bridge;
}

Component component(const std::string& package, const std::string& plugin, const std::string& name,
                    const std::string& name_space) {
  return Component{.package = package, .plugin = plugin, .name = name, .name_space = name_space};
}

std::string robot_namespace(int robot) {
  return std::string{"/robot_"} + (robot < kTwoDigits ? "0" : "") + std::to_string(robot);
}

std::vector<Component> core_components(int workers) {
  std::vector<Component> components{
      component("rovi_scheduler", "rovi::scheduler::SchedulerNode", "rovi_scheduler", "/"),
      component("rovi_network", "rovi::network::LinkNode", "rovi_link", "/")};
  for (int worker = 1; worker <= workers; ++worker) {
    components.push_back(component("rovi_worker", "rovi::worker::WorkerNode",
                                   "rovi_worker_" + std::to_string(worker), "/"));
  }
  return components;
}

Component agent_of(int robot) {
  return component("rovi_robot_agent", "rovi::robot_agent::RobotAgentNode", "robot_agent",
                   robot_namespace(robot));
}

Component arm_controller_of(int robot) {
  return component("rovi_robot_agent", "rovi::robot_agent::ArmControllerNode", "arm_controller",
                   robot_namespace(robot));
}

rclcpp::QoS events() { return rclcpp::QoS(rclcpp::KeepLast(kProbeDepth)).reliable(); }

std::unique_ptr<gz::sim::Server> loaded_server(const GazeboScenarioSpec& spec) {
  gz::sim::addResourcePaths({share("rovi_sim") + "/models"});
  gz::sim::ServerConfig config;
  if (!config.SetSdfFile(share("rovi_sim") + "/worlds/" + spec.world)) {
    throw std::runtime_error{"cannot load " + spec.world};
  }
  auto server = std::make_unique<gz::sim::Server>(config);
  if (!server->HasEntity(spec.entity)) {
    throw std::runtime_error{spec.world + " loaded without " + spec.entity +
                             "; run scripts/fetch_models.sh if its model is missing"};
  }
  return server;
}

}  // namespace

GazeboScenarioSpec mobile_robot_scenario() {
  std::vector<Component> components = core_components(1);
  components.push_back(agent_of(0));
  components.push_back(component("rovi_robot_agent", "rovi::robot_agent::WaypointControllerNode",
                                 "waypoint_controller", robot_namespace(0)));
  return {.world = "edge_warehouse.sdf",
          .entity = "robot_00",
          .bridge_config = "edge_warehouse_bridge.yaml",
          .parameters = "sim_one_robot.yaml",
          .components = std::move(components),
          .joint_state_topics = {},
          .command_topics = {},
          .physics_step_ns = kMillisecondNs};
}

GazeboScenarioSpec arm_workcell_scenario() {
  std::vector<Component> components = core_components(1);
  components.push_back(agent_of(0));
  components.push_back(arm_controller_of(0));
  std::vector<std::string> topics;
  for (int joint = 1; joint <= kArmJoints; ++joint) {
    topics.push_back(robot_namespace(0) + "/arm/panda_joint" + std::to_string(joint) + "/position");
  }
  return {.world = "arm_workcell.sdf",
          .entity = "panda_00",
          .bridge_config = "arm_workcell_bridge.yaml",
          .parameters = "arm_workcell.yaml",
          .components = std::move(components),
          .joint_state_topics = {robot_namespace(0) + "/joint_states"},
          .command_topics = std::move(topics),
          .physics_step_ns = kMillisecondNs};
}

GazeboScenarioSpec thirty_arm_scenario() {
  std::vector<Component> components = core_components(kThirtyArmWorkers);
  components.push_back(component("rovi_coordination", "rovi::coordination::CoordinatorNode",
                                 "rovi_coordinator", "/"));
  std::vector<std::string> joint_states;
  for (int robot = 0; robot < kThirtyArms; ++robot) {
    components.push_back(agent_of(robot));
    components.push_back(arm_controller_of(robot));
    joint_states.push_back(robot_namespace(robot) + "/joint_states");
  }
  return {.world = "thirty_arms.sdf",
          .entity = "panda_29",
          .bridge_config = "thirty_arms_bridge.yaml",
          .parameters = "thirty_arms.yaml",
          .components = std::move(components),
          .joint_state_topics = std::move(joint_states),
          .command_topics = {},
          .physics_step_ns = 2 * kMillisecondNs};
}

std::vector<std::pair<std::int64_t, std::int64_t>> fallback_intervals(
    const std::vector<RobotState>& states, std::uint32_t robot) {
  std::vector<std::pair<std::int64_t, std::int64_t>> intervals;
  bool in_fallback = false;
  std::int64_t start = 0;
  std::int64_t last = 0;
  for (const RobotState& state : states) {
    if (state.robot_id != robot) {
      continue;
    }
    last = state.stamp_ns;
    if (state.mode == RobotState::MODE_FALLBACK && !in_fallback) {
      in_fallback = true;
      start = state.stamp_ns;
    } else if (state.mode == RobotState::MODE_EXECUTING && in_fallback) {
      intervals.emplace_back(start, state.stamp_ns);
      in_fallback = false;
    }
  }
  if (in_fallback) {
    intervals.emplace_back(start, last);
  }
  return intervals;
}

GazeboScenario::GazeboScenario(const GazeboScenarioSpec& spec)
    : server_{loaded_server(spec)},
      executor_{std::make_shared<rclcpp::executors::SingleThreadedExecutor>()},
      manager_{std::make_shared<rclcpp_components::ComponentManager>(executor_)} {
  if (spec.physics_step_ns <= 0 || kGazeboStepNs % spec.physics_step_ns != 0) {
    throw std::invalid_argument{"the physics step must divide the 10 ms lock step"};
  }
  iterations_ = static_cast<std::uint64_t>(kGazeboStepNs / spec.physics_step_ns);
  bridge_ = factory_for(*manager_, bridge_component())
                ->create_node_instance(bridge_options(bridge_component(), spec));
  executor_->add_node(bridge_->get_node_base_interface());
  for (const Component& component : spec.components) {
    instances_.push_back(factory_for(*manager_, component)
                             ->create_node_instance(options_with(component, rovi_arguments(spec))));
    executor_->add_node(instances_.back().get_node_base_interface());
  }
  probe_ = std::make_shared<rclcpp::Node>("rovi_gazebo_probe");
  subscriptions_.push_back(probe_->create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::ClockQoS(), [this](const rosgraph_msgs::msg::Clock& message) {
        clock_ns_ = rclcpp::Time(message.clock).nanoseconds();
      }));
  subscriptions_.push_back(probe_->create_subscription<nav_msgs::msg::Odometry>(
      "/robot_00/odom", events(), [this](const nav_msgs::msg::Odometry& message) {
        odometry_.push_back(
            OdometrySample{.stamp_ns = rclcpp::Time(message.header.stamp).nanoseconds(),
                           .x = message.pose.pose.position.x,
                           .y = message.pose.pose.position.y,
                           .linear_speed = message.twist.twist.linear.x});
      }));
  for (std::size_t topic = 0; topic < spec.joint_state_topics.size(); ++topic) {
    subscriptions_.push_back(probe_->create_subscription<sensor_msgs::msg::JointState>(
        spec.joint_state_topics.at(topic), events(),
        [this, topic](const sensor_msgs::msg::JointState& message) {
          joint_samples_.push_back(JointSample{.topic = topic, .state = message});
        }));
  }
  subscriptions_.push_back(probe_->create_subscription<JobRequest>(
      "/rovi/job_requests", events(),
      [this](const JobRequest& message) { requests_.push_back(message); }));
  subscriptions_.push_back(probe_->create_subscription<Decision>(
      "/rovi/decisions", events(),
      [this](const Decision& message) { decisions_.push_back(message); }));
  subscriptions_.push_back(probe_->create_subscription<GroupRelease>(
      "/rovi/coordination/releases", events(),
      [this](const GroupRelease& message) { releases_.push_back(message); }));
  subscriptions_.push_back(probe_->create_subscription<TaskProgress>(
      "/rovi/task_progress", events(),
      [this](const TaskProgress& message) { progress_.push_back(message); }));
  for (std::size_t topic = 0; topic < spec.command_topics.size(); ++topic) {
    subscriptions_.push_back(probe_->create_subscription<std_msgs::msg::Float64>(
        spec.command_topics.at(topic), events(),
        [this, topic](const std_msgs::msg::Float64& message) {
          commands_.push_back(
              CommandSample{.topic = topic, .received_ns = clock_ns_, .value = message.data});
        }));
  }
  subscriptions_.push_back(probe_->create_subscription<RobotState>(
      "/rovi/robot_state", events(),
      [this](const RobotState& message) { states_.push_back(message); }));
  subscriptions_.push_back(probe_->create_subscription<Payload>(
      "/rovi/downlink/delivered", events(),
      [this](const Payload& message) { chunks_.push_back(message); }));
  subscriptions_.push_back(probe_->create_subscription<RunMetadata>(
      "/rovi/run_metadata", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](const RunMetadata& message) { metadata_ = message; }));
  executor_->add_node(probe_);
}

GazeboScenario::~GazeboScenario() {
  executor_->remove_node(probe_);
  executor_->remove_node(bridge_->get_node_base_interface());
  for (rclcpp_components::NodeInstanceWrapper& instance : instances_) {
    executor_->remove_node(instance.get_node_base_interface());
  }
  bridge_.reset();
  instances_.clear();
  server_->Stop();
  server_.reset();
}

bool GazeboScenario::connect(std::chrono::steady_clock::time_point deadline) {
  while (clock_ns_ == 0) {
    if (std::chrono::steady_clock::now() > deadline) {
      failure_ = "the bridge never delivered the simulation clock";
      return false;
    }
    if (!server_->Run(true, 1, false)) {
      failure_ = "the server refused to step during discovery";
      return false;
    }
    const auto settled = std::chrono::steady_clock::now() + kDiscoveryWait;
    while (std::chrono::steady_clock::now() < settled) {
      executor_->spin_all(kDrainLimit);
    }
  }
  stepped_ns_ = clock_ns_;
  return true;
}

bool GazeboScenario::run_until(std::int64_t sim_ns, std::chrono::seconds wall_limit) {
  const auto deadline = std::chrono::steady_clock::now() + wall_limit;
  if (stepped_ns_ == 0 && !connect(deadline)) {
    return false;
  }
  while (stepped_ns_ < sim_ns) {
    if (std::chrono::steady_clock::now() > deadline) {
      failure_ = "the wall-clock limit expired at " + std::to_string(stepped_ns_) + " ns";
      return false;
    }
    if (!server_->Run(true, iterations_, false)) {
      failure_ = "the server refused to step at " + std::to_string(stepped_ns_) + " ns";
      return false;
    }
    stepped_ns_ += kGazeboStepNs;
    while (clock_ns_ < stepped_ns_) {
      if (std::chrono::steady_clock::now() > deadline) {
        failure_ = "clock stalled at " + std::to_string(clock_ns_) + " ns while waiting for " +
                   std::to_string(stepped_ns_) + " ns";
        return false;
      }
      executor_->spin_all(kDrainLimit);
    }
    executor_->spin_all(kDrainLimit);
    const auto settled = std::chrono::steady_clock::now() + kSettle;
    while (std::chrono::steady_clock::now() < settled) {
      executor_->spin_all(kDrainLimit);
    }
    executor_->spin_all(kDrainLimit);
  }
  return true;
}

const std::string& GazeboScenario::failure() const { return failure_; }

const std::vector<OdometrySample>& GazeboScenario::odometry() const { return odometry_; }

const std::vector<JointSample>& GazeboScenario::joint_samples() const { return joint_samples_; }

const std::vector<JobRequest>& GazeboScenario::requests() const { return requests_; }

const std::vector<Decision>& GazeboScenario::decisions() const { return decisions_; }

const std::vector<GroupRelease>& GazeboScenario::releases() const { return releases_; }

const std::vector<TaskProgress>& GazeboScenario::progress() const { return progress_; }

const std::vector<CommandSample>& GazeboScenario::commands() const { return commands_; }

const std::vector<RobotState>& GazeboScenario::states() const { return states_; }

const std::vector<Payload>& GazeboScenario::chunks() const { return chunks_; }

const std::optional<RunMetadata>& GazeboScenario::metadata() const { return metadata_; }

}  // namespace rovi::tests
