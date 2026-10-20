#include "rovi_robot_agent/robot_agent_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "rovi_core/admission.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/receipt.hpp"
#include "rovi_core/units.hpp"
#include "rovi_interfaces/msg/decision.hpp"
#include "rovi_interfaces/msg/group_release.hpp"
#include "rovi_interfaces/msg/job_request.hpp"
#include "rovi_interfaces/msg/payload.hpp"
#include "rovi_interfaces/msg/request_receipt.hpp"
#include "rovi_interfaces/msg/robot_state.hpp"
#include "rovi_robot_agent/messages.hpp"
#include "rovi_robot_agent/robot_agent.hpp"
#include "rovi_telemetry/run_clock.hpp"

namespace rovi::robot_agent {
namespace {

using rovi_interfaces::msg::Decision;
using rovi_interfaces::msg::GroupRelease;
using rovi_interfaces::msg::JobRequest;
using rovi_interfaces::msg::Payload;
using rovi_interfaces::msg::RequestReceipt;
using rovi_interfaces::msg::RobotState;

constexpr std::size_t kEventDepth = 1'000;
constexpr std::size_t kStateDepth = 10;
constexpr int kLogThrottleMs = 1'000;

[[noreturn]] void reject(const std::string& name) {
  throw std::invalid_argument{"robot_agent: invalid parameter " + name};
}

template <typename Narrow>
Narrow narrowed(rclcpp::Node& node, const std::string& name) {
  const std::int64_t value = node.declare_parameter<std::int64_t>(name);
  if (!std::in_range<Narrow>(value)) {
    reject(name);
  }
  return static_cast<Narrow>(value);
}

core::Duration duration(rclcpp::Node& node, const std::string& name) {
  return core::Duration::from_ns(node.declare_parameter<std::int64_t>(name));
}

std::chrono::nanoseconds period(rclcpp::Node& node, const std::string& name) {
  const std::int64_t value = node.declare_parameter<std::int64_t>(name);
  if (value <= 0) {
    reject(name);
  }
  return std::chrono::nanoseconds{value};
}

AgentConfig load_config(rclcpp::Node& node) {
  std::vector<core::WorkerId> workers;
  for (const std::int64_t id :
       node.declare_parameter<std::vector<std::int64_t>>("eligible_workers")) {
    if (id < 0) {
      reject("eligible_workers");
    }
    workers.emplace_back(static_cast<std::uint64_t>(id));
  }
  AgentConfig config{
      .robot = core::RobotId{narrowed<std::uint32_t>(node, "robot_id")},
      .initial_buffer = duration(node, "initial_buffer_ns"),
      .chunk_horizon = duration(node, "chunk_horizon_ns"),
      .maximum_buffer = duration(node, "maximum_buffer_ns"),
      .safety_margin = duration(node, "safety_margin_ns"),
      .maximum_result_age = duration(node, "maximum_result_age_ns"),
      .upload_bytes = core::Bytes{node.declare_parameter<std::int64_t>("upload_bytes")},
      .action_chunk_bytes = core::Bytes{node.declare_parameter<std::int64_t>("action_chunk_bytes")},
      .model = core::ModelClass{narrowed<std::uint32_t>(node, "model_class")},
      .eligible_workers = std::move(workers),
      .grouped = node.declare_parameter<bool>("grouped", false),
      .release_grace =
          core::Duration::from_ns(node.declare_parameter<std::int64_t>("release_grace_ns", 0))};
  if (!RobotAgent::create(config, core::TimePoint{})) {
    reject("action buffer, payload, or worker configuration");
  }
  return config;
}

}  // namespace

RobotAgentNode::RobotAgentNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("robot_agent", options), clock_{*this}, config_{load_config(*this)} {
  const std::chrono::nanoseconds observation_period = period(*this, "observation_period_ns");
  const std::chrono::nanoseconds state_period = period(*this, "state_period_ns");
  const rclcpp::QoS events = rclcpp::QoS(rclcpp::KeepLast(kEventDepth)).reliable();
  requests_ = create_publisher<JobRequest>(
      config_.grouped ? "/rovi/coordination/proposals" : "/rovi/job_requests", events);
  observations_ = create_publisher<Payload>("/rovi/uplink/sent", events);
  states_ = create_publisher<RobotState>("/rovi/robot_state",
                                         rclcpp::QoS(rclcpp::KeepLast(kStateDepth)).reliable());
  own_state_ =
      create_publisher<RobotState>("state", rclcpp::QoS(rclcpp::KeepLast(kStateDepth)).reliable());
  decisions_ = create_subscription<Decision>(
      "/rovi/decisions", events, [this](const Decision& decision) { on_decision(decision); });
  chunks_ = create_subscription<Payload>("/rovi/downlink/delivered", events,
                                         [this](const Payload& payload) { on_chunk(payload); });
  if (config_.grouped) {
    releases_ = create_subscription<GroupRelease>(
        "/rovi/coordination/releases", events,
        [this](const GroupRelease& release) { on_release(release); });
  }
  receipts_ = create_publisher<RequestReceipt>("/rovi/requests/receipts", events);
  observe_timer_ = create_timer(observation_period, [this] { on_observe(); });
  state_timer_ = create_timer(state_period, [this] { on_state(); });
  settings_ = telemetry::announce_settings(*this);
}

void RobotAgentNode::publish_receipt(std::uint8_t event, core::JobId job, core::TimePoint stamp,
                                     core::TimePoint deadline) {
  RequestReceipt message;
  message.event = event;
  message.robot_id = config_.robot.value();
  message.job_id = job.value();
  message.stamp_ns = stamp.ns();
  message.deadline_ns = deadline.ns();
  receipts_->publish(message);
}

RobotAgent* RobotAgentNode::agent(core::TimePoint now) {
  if (!agent_) {
    agent_ = RobotAgent::create(config_, now);
  }
  if (!agent_) {
    RCLCPP_ERROR_STREAM_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
                                 "cannot start the action buffer at " << now.ns() << " ns");
    return nullptr;
  }
  return &*agent_;
}

void RobotAgentNode::on_observe() {
  const std::optional<core::TimePoint> now = clock_.now();
  if (!now) {
    return;
  }
  RobotAgent* current = agent(*now);
  if (current == nullptr) {
    return;
  }
  if (const std::optional<core::Job> job = current->observe(*now)) {
    requests_->publish(to_request(*job));
    publish_receipt(RequestReceipt::EVENT_OFFERED, job->id, *now,
                    current->receipts().deadline_of(job->id).value_or(core::TimePoint{}));
  }
}

void RobotAgentNode::on_decision(const Decision& decision) {
  const std::optional<core::TimePoint> now = clock_.now();
  if (!now) {
    return;
  }
  const std::optional<core::DecisionStatus> status = status_from(decision.status);
  if (!agent_ || decision.robot_id != config_.robot.value() || !status) {
    return;
  }
  const bool admitted = *status == core::DecisionStatus::kAdmitted;
  if (admitted && !decision.has_reservation) {
    return;
  }
  const core::WorkerId worker{decision.reservation.worker_id};
  if (!agent_->on_decision(core::JobId{decision.job_id}, *status, worker) || !agent_->in_flight()) {
    return;
  }
  const core::Job& job = agent_->in_flight()->job;
  Payload observation;
  observation.kind = Payload::KIND_OBSERVATION;
  observation.job_id = job.id.value();
  observation.robot_id = job.robot.value();
  observation.worker_id = decision.reservation.worker_id;
  observation.bytes = job.upload_bytes.value();
  observation.observation_time_ns = job.observation_time.ns();
  observation.sent_ns = now->ns();
  observations_->publish(observation);
}

void RobotAgentNode::on_release(const GroupRelease& release) {
  const std::optional<core::TimePoint> now = clock_.now();
  if (!now || !agent_ ||
      std::ranges::find(release.robots, config_.robot.value()) == release.robots.end()) {
    return;
  }
  ReleaseNotice notice{.jobs = {},
                       .released = release.released,
                       .release_time = core::TimePoint::from_ns(release.release_time_ns)};
  notice.jobs.reserve(release.jobs.size());
  for (const std::uint64_t job : release.jobs) {
    notice.jobs.emplace_back(job);
  }
  const ReleaseVerdict verdict = agent_->on_release(notice, *now);
  if (verdict == ReleaseVerdict::kMissed || verdict == ReleaseVerdict::kRejected) {
    RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
                                "group " << release.synchronization_group
                                         << " was not released (reason "
                                         << static_cast<int>(release.reason) << ")");
  }
}

void RobotAgentNode::on_chunk(const Payload& payload) {
  const std::optional<core::TimePoint> now = clock_.now();
  if (!now || !agent_ || payload.kind != Payload::KIND_ACTION_CHUNK ||
      payload.robot_id != config_.robot.value()) {
    return;
  }
  const ChunkReceipt receipt{
      .job = core::JobId{payload.job_id},
      .worker = core::WorkerId{payload.worker_id},
      .observation_time = core::TimePoint::from_ns(payload.observation_time_ns)};
  const ChunkVerdict verdict = agent_->on_chunk(receipt, *now);
  if (const std::optional<core::Receipt>& received = agent_->latest_receipt()) {
    publish_receipt(RequestReceipt::EVENT_RECEIVED, received->job, received->at,
                    received->deadline);
  }
  if (verdict != ChunkVerdict::kAccepted && verdict != ChunkVerdict::kHeld) {
    RCLCPP_WARN_STREAM_THROTTLE(
        get_logger(), *get_clock(), kLogThrottleMs,
        "discarded the " << verdict_name(verdict) << " action chunk of job " << payload.job_id);
  }
}

void RobotAgentNode::on_state() {
  const std::optional<core::TimePoint> now = clock_.now();
  if (!now) {
    return;
  }
  RobotAgent* current = agent(*now);
  if (current == nullptr) {
    return;
  }
  current->refresh(*now);
  const RobotState state = to_state(*current, *now);
  states_->publish(state);
  own_state_->publish(state);
}

}  // namespace rovi::robot_agent

RCLCPP_COMPONENTS_REGISTER_NODE(rovi::robot_agent::RobotAgentNode)
