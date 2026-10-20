#include "rovi_scheduler/scheduler_node.hpp"

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
#include "rovi_core/scheduler.hpp"
#include "rovi_core/units.hpp"
#include "rovi_core/window.hpp"
#include "rovi_core/wireless_profile.hpp"
#include "rovi_interfaces/msg/decision.hpp"
#include "rovi_interfaces/msg/job_request.hpp"
#include "rovi_interfaces/msg/payload.hpp"
#include "rovi_interfaces/msg/reservation.hpp"
#include "rovi_interfaces/msg/run_metadata.hpp"
#include "rovi_interfaces/msg/scheduler_cycle.hpp"
#include "rovi_scheduler/config.hpp"
#include "rovi_scheduler/conversions.hpp"
#include "rovi_scheduler/job_book.hpp"
#include "rovi_telemetry/run_clock.hpp"

namespace rovi::scheduler {
namespace {

using rovi_interfaces::msg::Decision;
using rovi_interfaces::msg::JobRequest;
using rovi_interfaces::msg::Payload;
using rovi_interfaces::msg::RunMetadata;
using rovi_interfaces::msg::SchedulerCycle;
using Integers = std::vector<std::int64_t>;

constexpr std::size_t kEventDepth = 1'000;
constexpr int kLogThrottleMs = 1'000;

template <typename T>
T required(std::optional<T> value, const std::string& name) {
  if (!value) {
    throw std::invalid_argument{"rovi_scheduler: invalid parameter " + name};
  }
  return std::move(*value);
}

std::vector<core::ServiceSegment> load_segments(rclcpp::Node& node, const std::string& link) {
  return required(
      segments_from(SegmentArrays{
          .first_slots = node.declare_parameter<Integers>(link + ".first_slots"),
          .rates_bps = node.declare_parameter<Integers>(link + ".rates_bps"),
          .reductions_permille = node.declare_parameter<Integers>(link + ".reductions_permille")}),
      link);
}

core::Duration duration_parameter(rclcpp::Node& node, const std::string& name) {
  return core::Duration::from_ns(node.declare_parameter<std::int64_t>(name));
}

SchedulerConfig load_config(rclcpp::Node& node) {
  SchedulerConfig config{
      .policy = required(policy_from(node.declare_parameter<std::string>("policy")), "policy"),
      .cycle_period = duration_parameter(node, "cycle_period_ns"),
      .release_tolerance = duration_parameter(node, "release_tolerance_ns"),
      .horizon_slots = node.declare_parameter<std::int64_t>("horizon_slots"),
      .max_pending_jobs = node.declare_parameter<std::int64_t>("max_pending_jobs"),
      .profile =
          core::ServiceProfile{.slot_length = duration_parameter(node, "slot_length_ns"),
                               .uplink = load_segments(node, "uplink"),
                               .downlink = load_segments(node, "downlink"),
                               .uplink_delay = duration_parameter(node, "uplink_delay_ns"),
                               .downlink_delay = duration_parameter(node, "downlink_delay_ns"),
                               .dispatch_lead = duration_parameter(node, "dispatch_lead_ns")},
      .workers =
          required(workers_from(WorkerArrays{
                       .ids = node.declare_parameter<Integers>("workers.ids"),
                       .model_classes = node.declare_parameter<Integers>("workers.model_classes"),
                       .durations_ns = node.declare_parameter<Integers>("workers.durations_ns")}),
                   "workers")};
  if (node.declare_parameter<bool>("medium.enabled", false)) {
    config.profile.medium = load_segments(node, "medium");
  }
  if (!is_valid(config)) {
    throw std::invalid_argument{"rovi_scheduler: inconsistent scheduler configuration"};
  }
  return config;
}

RunIdentity load_run_identity(rclcpp::Node& node) {
  const std::int64_t seed = node.declare_parameter<std::int64_t>("run.seed", 0);
  if (seed < 0) {
    throw std::invalid_argument{"rovi_scheduler: invalid parameter run.seed"};
  }
  return RunIdentity{
      .run_id = node.declare_parameter<std::string>("run.id", ""),
      .seed = static_cast<std::uint64_t>(seed),
      .git_revision = node.declare_parameter<std::string>("run.git_revision", "unknown"),
      .network_profile_id =
          node.declare_parameter<std::string>("run.network_profile_id", "unspecified")};
}

std::uint32_t count_status(const std::vector<core::JobDecision>& decisions,
                           core::DecisionStatus status) {
  return static_cast<std::uint32_t>(std::ranges::count_if(
      decisions,
      [status](const core::JobDecision& decision) { return decision.decision.status == status; }));
}

}  // namespace

SchedulerNode::SchedulerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("rovi_scheduler", options),
      clock_{*this},
      config_{load_config(*this)},
      run_{load_run_identity(*this)},
      book_{BookLimits{.release_tolerance = config_.release_tolerance,
                       .max_pending = static_cast<std::size_t>(config_.max_pending_jobs)}} {
  const rclcpp::QoS events = rclcpp::QoS(rclcpp::KeepLast(kEventDepth)).reliable();
  decisions_ = create_publisher<Decision>("/rovi/decisions", events);
  cycles_ = create_publisher<SchedulerCycle>("/rovi/scheduler/cycles", events);
  metadata_ = create_publisher<RunMetadata>(
      "/rovi/run_metadata", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  requests_ = create_subscription<JobRequest>(
      "/rovi/job_requests", events, [this](const JobRequest& request) { on_request(request); });
  delivered_ =
      create_subscription<Payload>("/rovi/downlink/delivered", events,
                                   [this](const Payload& payload) { on_delivered(payload); });
  cycle_timer_ =
      create_timer(std::chrono::nanoseconds{config_.cycle_period.ns()}, [this] { on_cycle(); });
  settings_ = telemetry::announce_settings(*this);
}

void SchedulerNode::on_request(const JobRequest& request) {
  const std::optional<core::TimePoint> tick = clock_.now();
  if (!tick) {
    return;
  }
  const core::TimePoint now = *tick;
  const core::Job job = to_job(request);
  if (const std::optional<core::JobDecision> rejection =
          book_.submit(job, now, request.synchronization_group_size)) {
    publish_notice(Notice{.job = job, .decision = *rejection}, nullptr, now);
  }
}

void SchedulerNode::on_delivered(const Payload& payload) {
  if (!clock_.now()) {
    return;
  }
  if (payload.kind == Payload::KIND_ACTION_CHUNK) {
    static_cast<void>(book_.complete(core::JobId{payload.job_id}));
  }
}

void SchedulerNode::on_cycle() {
  const std::optional<core::TimePoint> tick = clock_.now();
  if (!tick) {
    return;
  }
  const core::TimePoint now = *tick;
  if (!slide_window(now) || !window_) {
    return;
  }
  Window& window = *window_;
  for (const Notice& notice : book_.expire(now)) {
    publish_notice(notice, nullptr, now);
  }
  const std::vector<core::Job> released = book_.released(now);
  if (released.empty()) {
    return;
  }
  const auto started = std::chrono::steady_clock::now();
  core::ScheduleResult result =
      core::schedule(window.snapshot, released, book_.completed_for(released), now, config_.policy);
  const auto runtime = std::chrono::steady_clock::now() - started;
  ++cycle_;
  for (const Notice& notice : book_.settle(result.decisions)) {
    publish_notice(notice, &result.snapshot, now);
  }
  SchedulerCycle telemetry;
  telemetry.cycle = cycle_;
  telemetry.now_ns = now.ns();
  telemetry.window_start_ns =
      result.snapshot.uplink.slot_start(core::SlotIndex{0}).value_or(now).ns();
  telemetry.window_end_ns = result.snapshot.uplink.horizon_end().ns();
  telemetry.pending = static_cast<std::uint32_t>(released.size());
  telemetry.admitted = count_status(result.decisions, core::DecisionStatus::kAdmitted);
  telemetry.deferred = count_status(result.decisions, core::DecisionStatus::kDeferred);
  telemetry.rejected = count_status(result.decisions, core::DecisionStatus::kRejected);
  telemetry.decision_digest = result.digest;
  telemetry.runtime_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(runtime).count();
  cycles_->publish(telemetry);
  window.snapshot = std::move(result.snapshot);
}

bool SchedulerNode::slide_window(core::TimePoint now) {
  if (!window_) {
    std::optional<core::ResourceSnapshot> opened =
        core::open_window(config_.profile, config_.workers, now, config_.horizon_slots);
    if (!opened) {
      RCLCPP_ERROR_STREAM_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
                                   "cannot open the resource window at " << now.ns() << " ns");
      return false;
    }
    window_ = Window{.epoch = now, .snapshot = std::move(*opened)};
    publish_metadata(now);
    return true;
  }
  std::optional<core::ResourceSnapshot> advanced =
      core::advance_window(window_->snapshot, config_.profile, window_->epoch, now);
  if (!advanced) {
    RCLCPP_ERROR_STREAM_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
                                 "cannot advance the resource window to " << now.ns() << " ns");
    return false;
  }
  window_->snapshot = std::move(*advanced);
  return true;
}

void SchedulerNode::publish_notice(const Notice& notice, const core::ResourceSnapshot* snapshot,
                                   core::TimePoint now) {
  Decision message = to_decision(
      notice.job, notice.decision.decision,
      DecisionContext{.cycle = cycle_, .decided = now, .source = config_.profile.source});
  if (notice.decision.reservation && snapshot != nullptr) {
    std::optional<rovi_interfaces::msg::Reservation> reservation =
        to_reservation(*notice.decision.reservation, *snapshot);
    if (reservation) {
      message.has_reservation = true;
      message.reservation = *reservation;
    } else {
      RCLCPP_ERROR_STREAM_THROTTLE(
          get_logger(), *get_clock(), kLogThrottleMs,
          "cannot express the reservation of job " << notice.job.id.value() << " on ROS time");
    }
  }
  decisions_->publish(message);
}

void SchedulerNode::publish_metadata(core::TimePoint epoch) {
  RunMetadata message;
  message.interface_version = RunMetadata::INTERFACE_VERSION;
  message.run_id = run_.run_id;
  message.seed = run_.seed;
  message.config_digest = config_digest(config_);
  message.git_revision = run_.git_revision;
  message.network_profile_id = run_.network_profile_id;
  message.service_source = to_source(config_.profile.source);
  message.policy = to_policy(config_.policy);
  message.epoch_ns = epoch.ns();
  message.slot_length_ns = config_.profile.slot_length.ns();
  message.horizon_slots = config_.horizon_slots;
  metadata_->publish(message);
}

}  // namespace rovi::scheduler

RCLCPP_COMPONENTS_REGISTER_NODE(rovi::scheduler::SchedulerNode)
