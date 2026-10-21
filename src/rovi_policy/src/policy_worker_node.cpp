#include "rovi_policy/policy_worker_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "rovi_core/units.hpp"
#include "rovi_interfaces/msg/decision.hpp"
#include "rovi_interfaces/msg/inference_record.hpp"
#include "rovi_interfaces/msg/payload.hpp"
#include "rovi_interfaces/msg/policy_server.hpp"
#include "rovi_policy/chunk_check.hpp"
#include "rovi_policy/policy_client.hpp"
#include "rovi_policy/policy_lane.hpp"
#include "rovi_policy/wire.hpp"
#include "rovi_telemetry/run_clock.hpp"

namespace rovi::policy {
namespace {

using rovi_interfaces::msg::Decision;
using rovi_interfaces::msg::InferenceRecord;
using rovi_interfaces::msg::Payload;
using rovi_interfaces::msg::PolicyServer;
using Integers = std::vector<std::int64_t>;

constexpr std::size_t kEventDepth = 100;
constexpr int kWarningThrottleMs = 1'000;

[[noreturn]] void reject(const std::string& name) {
  throw std::invalid_argument{"rovi_policy_worker: invalid parameter " + name};
}

std::int64_t positive(rclcpp::Node& node, const std::string& name) {
  const std::int64_t value = node.declare_parameter<std::int64_t>(name);
  if (value <= 0) {
    reject(name);
  }
  return value;
}

std::chrono::milliseconds milliseconds(rclcpp::Node& node, const std::string& name) {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::nanoseconds{positive(node, name)});
}

core::WorkerId load_id(rclcpp::Node& node) {
  const std::int64_t id = node.declare_parameter<std::int64_t>("worker_id");
  if (id < 0) {
    reject("worker_id");
  }
  return core::WorkerId{static_cast<std::uint64_t>(id)};
}

std::vector<core::ModelClass> load_models(rclcpp::Node& node) {
  std::vector<core::ModelClass> models;
  for (const std::int64_t model : node.declare_parameter<Integers>("model_classes")) {
    if (!std::in_range<std::uint32_t>(model)) {
      reject("model_classes");
    }
    models.emplace_back(static_cast<std::uint32_t>(model));
  }
  if (models.empty()) {
    reject("model_classes");
  }
  return models;
}

PolicyId load_policy(rclcpp::Node& node) {
  const std::optional<PolicyId> policy =
      policy_from_hex(node.declare_parameter<std::string>("policy_id"));
  if (!policy) {
    reject("policy_id");
  }
  return *policy;
}

PolicyLane load_lane(rclcpp::Node& node) {
  std::optional<PolicyLane> lane = PolicyLane::create(
      LaneTiming{.orphan_timeout = core::Duration::from_ns(positive(node, "orphan_timeout_ns")),
                 .max_lead = core::Duration::from_ns(positive(node, "max_lead_ns"))});
  if (!lane) {
    reject("orphan_timeout_ns");
  }
  return std::move(*lane);
}

Endpoint load_endpoint(rclcpp::Node& node) {
  const std::string host = node.declare_parameter<std::string>("server_host");
  const std::int64_t port = node.declare_parameter<std::int64_t>("server_port");
  if (host.empty()) {
    reject("server_host");
  }
  if (!std::in_range<std::uint16_t>(port) || port == 0) {
    reject("server_port");
  }
  return Endpoint{.host = host, .port = static_cast<std::uint16_t>(port)};
}

PolicyClient load_client(rclcpp::Node& node, const Endpoint& endpoint, const PolicyId& policy) {
  const std::string task = node.declare_parameter<std::string>("task");
  if (task.empty()) {
    reject("task");
  }
  return PolicyClient{endpoint, ExpectedServer{.policy = policy, .task = task},
                      ClientTiming{.connect = milliseconds(node, "connect_timeout_ns"),
                                   .reply = milliseconds(node, "reply_timeout_ns")}};
}

std::uint8_t outcome_of(CallStatus status) {
  switch (status) {
    case CallStatus::kOk:
      return InferenceRecord::OUTCOME_OK;
    case CallStatus::kTimeout:
    case CallStatus::kStopped:
      return InferenceRecord::OUTCOME_TIMEOUT;
    case CallStatus::kProtocol:
      return InferenceRecord::OUTCOME_PROTOCOL;
    case CallStatus::kIdentity:
      return InferenceRecord::OUTCOME_IDENTITY;
    case CallStatus::kRemoteError:
      return InferenceRecord::OUTCOME_REMOTE_ERROR;
    case CallStatus::kUnreachable:
      break;
  }
  return InferenceRecord::OUTCOME_UNREACHABLE;
}

}  // namespace

PolicyWorkerNode::PolicyWorkerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("rovi_policy_worker", options),
      clock_{*this},
      id_{load_id(*this)},
      models_{load_models(*this)},
      policy_{load_policy(*this)},
      lane_{load_lane(*this)},
      endpoint_{load_endpoint(*this)},
      client_{load_client(*this, endpoint_, policy_)} {
  const std::int64_t poll_period_ns = positive(*this, "poll_period_ns");
  const rclcpp::QoS events = rclcpp::QoS(rclcpp::KeepLast(kEventDepth)).reliable();
  chunks_ = create_publisher<Payload>("/rovi/downlink/sent", events);
  records_ = create_publisher<InferenceRecord>("/rovi/inference/records", events);
  servers_ = create_publisher<PolicyServer>(
      "/rovi/policy/servers",
      rclcpp::QoS(rclcpp::KeepLast(kEventDepth)).reliable().transient_local());
  decisions_ = create_subscription<Decision>(
      "/rovi/decisions", events, [this](const Decision& decision) { on_decision(decision); });
  observations_ =
      create_subscription<Payload>("/rovi/uplink/delivered", events,
                                   [this](const Payload& payload) { on_observation(payload); });
  poll_timer_ = create_timer(std::chrono::nanoseconds{poll_period_ns}, [this] { on_poll(); });
  settings_ = telemetry::announce_settings(*this);
  caller_ = std::jthread{[this](const std::stop_token& stop) { call(stop); }};
}

void PolicyWorkerNode::on_decision(const Decision& decision) {
  const std::optional<core::TimePoint> now = clock_.now();
  if (!now || decision.status != Decision::STATUS_ADMITTED || !decision.has_reservation ||
      decision.reservation.worker_id != id_.value()) {
    return;
  }
  const bool serves =
      std::ranges::find(models_, core::ModelClass{decision.model_class}) != models_.end();
  const bool assigned =
      serves &&
      lane_.assign(
          LaneJob{.job = core::JobId{decision.job_id},
                  .robot = core::RobotId{decision.robot_id},
                  .planned_start = core::TimePoint::from_ns(decision.reservation.compute_start_ns),
                  .planned_end = core::TimePoint::from_ns(decision.reservation.compute_end_ns),
                  .chunk_bytes = core::Bytes{decision.reservation.download_bytes}},
          *now);
  if (!assigned) {
    RCLCPP_WARN_STREAM_THROTTLE(
        get_logger(), *get_clock(), kWarningThrottleMs,
        "cannot run job " << decision.job_id << " of model " << decision.model_class);
  }
}

void PolicyWorkerNode::on_observation(const Payload& payload) {
  const std::optional<core::TimePoint> now = clock_.now();
  if (!now || payload.kind != Payload::KIND_OBSERVATION || payload.worker_id != id_.value()) {
    return;
  }
  if (payload.joint_positions.empty() || payload.image_rgb8.empty()) {
    RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), kWarningThrottleMs,
                                "observation of job " << payload.job_id << " carries no content");
    return;
  }
  static_cast<void>(lane_.arrive(core::JobId{payload.job_id}, *now,
                                 core::TimePoint::from_ns(payload.observation_time_ns),
                                 Observation{.state = payload.joint_positions,
                                             .width = payload.image_width,
                                             .height = payload.image_height,
                                             .rgb = payload.image_rgb8}));
}

void PolicyWorkerNode::on_poll() {
  const std::optional<core::TimePoint> now = clock_.now();
  if (!now) {
    return;
  }
  lane_.prune(*now);
  std::optional<Finished> finished;
  {
    const std::scoped_lock lock{mutex_};
    finished = std::exchange(finished_, std::nullopt);
  }
  if (finished) {
    static_cast<void>(lane_.finish(finished->call.job.job));
    complete(*finished, *now);
  }
  if (std::optional<Started> started = lane_.start(*now)) {
    Call next{.job = started->job,
              .observation_time = started->observation_time,
              .requested = *now,
              .request = Request{.id = started->job.job.value(),
                                 .observation_ns = started->observation_time.ns(),
                                 .observation = std::move(started->observation)}};
    {
      const std::scoped_lock lock{mutex_};
      pending_ = std::move(next);
    }
    wake_.notify_one();
  }
  if (lane_.discarded() != reported_discards_) {
    reported_discards_ = lane_.discarded();
    RCLCPP_WARN_STREAM_THROTTLE(
        get_logger(), *get_clock(), kWarningThrottleMs,
        "dropped " << reported_discards_ << " jobs that could not start in their slot");
  }
}

void PolicyWorkerNode::complete(const Finished& finished, core::TimePoint now) {
  const LaneJob& job = finished.call.job;
  if (const std::optional<Hello>& server = finished.server) {
    PolicyServer announced;
    announced.worker_id = id_.value();
    announced.connection = finished.connection;
    announced.connected_ns = finished.call.requested.ns();
    announced.host = endpoint_.host;
    announced.port = endpoint_.port;
    announced.policy_id = to_hex(server->policy);
    announced.adapter_id = to_hex(server->adapter);
    announced.policy_type = server->policy_type;
    announced.task = server->task;
    announced.robot_type = server->robot_type;
    announced.device = server->device;
    announced.runtime = server->runtime;
    announced.action_steps = server->action_steps;
    announced.action_dim = server->action_dim;
    announced.state_dim = server->state_dim;
    announced.image_width = server->image_width;
    announced.image_height = server->image_height;
    announced.action_period_ns = server->action_period_ns;
    servers_->publish(announced);
  }
  InferenceRecord record;
  record.job_id = job.job.value();
  record.robot_id = job.robot.value();
  record.worker_id = id_.value();
  record.policy_id = to_hex(policy_);
  record.outcome = outcome_of(finished.result.status);
  record.planned_start_ns = job.planned_start.ns();
  record.planned_end_ns = job.planned_end.ns();
  record.request_ns = finished.call.requested.ns();
  record.response_ns = now.ns();
  record.round_trip_ns = finished.round_trip_ns;
  record.connection = finished.connection;
  if (const std::optional<Response>& response = finished.result.response) {
    record.server_inference_ns = static_cast<std::int64_t>(response->inference_ns);
    record.server_handling_ns = static_cast<std::int64_t>(response->handling_ns);
    record.noise_seed = response->noise_seed;
    const ChunkShape shape{.steps = response->steps, .dim = response->dim};
    if (check_chunk(response->actions, shape, shape, nullptr)) {
      record.outcome = InferenceRecord::OUTCOME_INVALID_CHUNK;
    }
  }
  records_->publish(record);
  if (record.outcome != InferenceRecord::OUTCOME_OK || !finished.result.response) {
    RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), kWarningThrottleMs,
                                "inference of job " << record.job_id << " failed ("
                                                    << status_name(finished.result.status) << ": "
                                                    << finished.result.detail << ")");
    return;
  }
  const Response& response = *finished.result.response;
  Payload chunk;
  chunk.kind = Payload::KIND_ACTION_CHUNK;
  chunk.job_id = job.job.value();
  chunk.robot_id = job.robot.value();
  chunk.worker_id = id_.value();
  chunk.bytes = job.chunk_bytes.value();
  chunk.observation_time_ns = finished.call.observation_time.ns();
  chunk.sent_ns = now.ns();
  chunk.action_steps = response.steps;
  chunk.action_dim = response.dim;
  chunk.actions = response.actions;
  chunk.action_period_ns = finished.action_period_ns;
  chunk.policy_id = record.policy_id;
  chunks_->publish(chunk);
}

void PolicyWorkerNode::call(const std::stop_token& stop) {
  while (true) {
    std::optional<Call> next;
    {
      std::unique_lock lock{mutex_};
      if (!wake_.wait(lock, stop, [this] { return pending_.has_value(); })) {
        return;
      }
      next = std::exchange(pending_, std::nullopt);
    }
    if (!next) {
      continue;
    }
    const auto start = std::chrono::steady_clock::now();
    CallResult result = client_.infer(next->request, stop);
    const std::int64_t round_trip = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::steady_clock::now() - start)
                                        .count();
    const std::uint32_t connection = client_.connections();
    std::optional<Hello> server;
    if (connection != announced_connections_) {
      announced_connections_ = connection;
      server = client_.accepted();
    }
    const std::int64_t period = client_.accepted() ? client_.accepted()->action_period_ns : 0;
    const std::scoped_lock lock{mutex_};
    finished_ = Finished{.call = std::move(*next),
                         .result = std::move(result),
                         .round_trip_ns = round_trip,
                         .action_period_ns = period,
                         .connection = connection,
                         .server = std::move(server)};
  }
}

}  // namespace rovi::policy

RCLCPP_COMPONENTS_REGISTER_NODE(rovi::policy::PolicyWorkerNode)
