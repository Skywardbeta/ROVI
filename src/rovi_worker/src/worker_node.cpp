#include "rovi_worker/worker_node.hpp"

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
#include "rovi_core/units.hpp"
#include "rovi_interfaces/msg/decision.hpp"
#include "rovi_interfaces/msg/payload.hpp"
#include "rovi_telemetry/run_clock.hpp"
#include "rovi_worker/worker_sim.hpp"

namespace rovi::worker {
namespace {

using rovi_interfaces::msg::Decision;
using rovi_interfaces::msg::Payload;
using Integers = std::vector<std::int64_t>;

constexpr std::size_t kEventDepth = 1'000;
constexpr int kWarningThrottleMs = 1'000;

[[noreturn]] void reject(const std::string& name) {
  throw std::invalid_argument{"rovi_worker: invalid parameter " + name};
}

std::int64_t positive(rclcpp::Node& node, const std::string& name) {
  const std::int64_t value = node.declare_parameter<std::int64_t>(name);
  if (value <= 0) {
    reject(name);
  }
  return value;
}

core::WorkerId load_id(rclcpp::Node& node) {
  const std::int64_t id = node.declare_parameter<std::int64_t>("worker_id");
  if (id < 0) {
    reject("worker_id");
  }
  return core::WorkerId{static_cast<std::uint64_t>(id)};
}

WorkerSim load_sim(rclcpp::Node& node) {
  const Integers models = node.declare_parameter<Integers>("model_classes");
  const Integers durations = node.declare_parameter<Integers>("durations_ns");
  if (models.size() != durations.size()) {
    reject("model_classes");
  }
  std::vector<core::ExecutionProfile> profiles;
  for (std::size_t index = 0; index < models.size(); ++index) {
    if (!std::in_range<std::uint32_t>(models.at(index))) {
      reject("model_classes");
    }
    profiles.push_back(core::ExecutionProfile{
        .model = core::ModelClass{static_cast<std::uint32_t>(models.at(index))},
        .duration = core::Duration::from_ns(durations.at(index))});
  }
  std::optional<WorkerSim> sim = WorkerSim::create(
      profiles,
      WorkerTiming{.slowdown = core::Duration::from_ns(
                       node.declare_parameter<std::int64_t>("slowdown_ns", 0)),
                   .orphan_timeout = core::Duration::from_ns(positive(node, "orphan_timeout_ns")),
                   .max_lead = core::Duration::from_ns(positive(node, "max_lead_ns"))});
  if (!sim) {
    reject("durations_ns");
  }
  return std::move(*sim);
}

}  // namespace

WorkerNode::WorkerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("rovi_worker", options),
      clock_{*this},
      id_{load_id(*this)},
      sim_{load_sim(*this)} {
  const std::int64_t poll_period_ns = positive(*this, "poll_period_ns");
  const rclcpp::QoS events = rclcpp::QoS(rclcpp::KeepLast(kEventDepth)).reliable();
  chunks_ = create_publisher<Payload>("/rovi/downlink/sent", events);
  decisions_ = create_subscription<Decision>(
      "/rovi/decisions", events, [this](const Decision& decision) { on_decision(decision); });
  observations_ =
      create_subscription<Payload>("/rovi/uplink/delivered", events,
                                   [this](const Payload& payload) { on_observation(payload); });
  poll_timer_ = create_timer(std::chrono::nanoseconds{poll_period_ns}, [this] { on_poll(); });
  settings_ = telemetry::announce_settings(*this);
}

void WorkerNode::on_decision(const Decision& decision) {
  const std::optional<core::TimePoint> now = clock_.now();
  if (!now) {
    return;
  }
  if (decision.status != Decision::STATUS_ADMITTED || !decision.has_reservation ||
      decision.reservation.worker_id != id_.value()) {
    return;
  }
  const bool assigned = sim_.assign(
      Assignment{.job = core::JobId{decision.job_id},
                 .robot = core::RobotId{decision.robot_id},
                 .model = core::ModelClass{decision.model_class},
                 .planned_start = core::TimePoint::from_ns(decision.reservation.compute_start_ns),
                 .chunk_bytes = core::Bytes{decision.reservation.download_bytes}},
      *now);
  if (!assigned) {
    RCLCPP_WARN_STREAM_THROTTLE(
        get_logger(), *get_clock(), kWarningThrottleMs,
        "cannot execute job " << decision.job_id << " of model " << decision.model_class);
  }
}

void WorkerNode::on_observation(const Payload& payload) {
  const std::optional<core::TimePoint> now = clock_.now();
  if (!now) {
    return;
  }
  if (payload.kind == Payload::KIND_OBSERVATION && payload.worker_id == id_.value()) {
    sim_.arrive(core::JobId{payload.job_id}, *now,
                core::TimePoint::from_ns(payload.observation_time_ns));
  }
}

void WorkerNode::on_poll() {
  const std::optional<core::TimePoint> now = clock_.now();
  if (!now) {
    return;
  }
  for (const Completion& completion : sim_.advance(*now)) {
    Payload chunk;
    chunk.kind = Payload::KIND_ACTION_CHUNK;
    chunk.job_id = completion.job.value();
    chunk.robot_id = completion.robot.value();
    chunk.worker_id = id_.value();
    chunk.bytes = completion.chunk_bytes.value();
    chunk.observation_time_ns = completion.observation_time.ns();
    chunk.sent_ns = now->ns();
    chunks_->publish(chunk);
  }
  if (sim_.discarded() != reported_discards_) {
    reported_discards_ = sim_.discarded();
    RCLCPP_WARN_STREAM_THROTTLE(
        get_logger(), *get_clock(), kWarningThrottleMs,
        "discarded " << reported_discards_ << " unmatched or unrepresentable jobs");
  }
}

}  // namespace rovi::worker

RCLCPP_COMPONENTS_REGISTER_NODE(rovi::worker::WorkerNode)
