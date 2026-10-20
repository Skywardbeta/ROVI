#include "rovi_network/link_node.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "rovi_core/units.hpp"
#include "rovi_interfaces/msg/decision.hpp"
#include "rovi_interfaces/msg/payload.hpp"
#include "rovi_interfaces/msg/reservation_violation.hpp"
#include "rovi_interfaces/msg/service_source.hpp"
#include "rovi_network/logical_link.hpp"
#include "rovi_telemetry/run_clock.hpp"

namespace rovi::network {
namespace {

using rovi_interfaces::msg::Decision;
using rovi_interfaces::msg::Payload;
using rovi_interfaces::msg::ReservationViolation;
using rovi_interfaces::msg::ServiceSource;

constexpr std::size_t kEventDepth = 1'000;
constexpr int kWarningThrottleMs = 1'000;

constexpr bool mirrors(Violation violation, std::uint8_t wire) {
  return static_cast<std::uint8_t>(violation) == wire;
}

static_assert(mirrors(Violation::kLateSend, ReservationViolation::REASON_LATE_SEND));
static_assert(mirrors(Violation::kSizeMismatch, ReservationViolation::REASON_SIZE_MISMATCH));
static_assert(mirrors(Violation::kIdentityMismatch,
                      ReservationViolation::REASON_IDENTITY_MISMATCH));
static_assert(mirrors(Violation::kDuplicate, ReservationViolation::REASON_DUPLICATE));
static_assert(mirrors(Violation::kUnleased, ReservationViolation::REASON_UNLEASED));
static_assert(mirrors(Violation::kWrongDirection, ReservationViolation::REASON_WRONG_DIRECTION));

core::Duration positive_duration(rclcpp::Node& node, const std::string& name) {
  const std::int64_t value = node.declare_parameter<std::int64_t>(name);
  if (value <= 0) {
    throw std::invalid_argument{"rovi_link: parameter " + name + " must be positive"};
  }
  return core::Duration::from_ns(value);
}

Lease lease_between(std::int64_t service_start_ns, std::int64_t arrival_ns,
                    const Authorization& authorized) {
  return Lease{.service_start = core::TimePoint::from_ns(service_start_ns),
               .arrival = core::TimePoint::from_ns(arrival_ns),
               .authorized = authorized};
}

Authorization claimed_by(const Payload& payload) {
  return Authorization{.robot = core::RobotId{payload.robot_id},
                       .worker = core::WorkerId{payload.worker_id},
                       .bytes = core::Bytes{payload.bytes}};
}

ReservationViolation to_violation(const Refusal& refusal) {
  ReservationViolation message;
  message.job_id = refusal.job.value();
  message.robot_id = refusal.claimed.robot.value();
  message.worker_id = refusal.claimed.worker.value();
  message.direction = refusal.direction == Direction::kUplink
                          ? ReservationViolation::DIRECTION_UPLINK
                          : ReservationViolation::DIRECTION_DOWNLINK;
  message.reason = static_cast<std::uint8_t>(refusal.reason);
  message.bytes = refusal.claimed.bytes.value();
  message.sent_ns = refusal.sent.ns();
  message.detected_ns = refusal.detected.ns();
  message.has_lease = refusal.lease.has_value();
  if (refusal.lease) {
    message.service_start_ns = refusal.lease->service_start.ns();
    message.arrival_ns = refusal.lease->arrival.ns();
    message.authorized_bytes = refusal.lease->authorized.bytes.value();
  }
  message.service_source.value = ServiceSource::LOGICAL;
  return message;
}

}  // namespace

LinkNode::LinkNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("rovi_link", options),
      clock_{*this},
      link_{LinkTiming{.orphan_timeout = positive_duration(*this, "orphan_timeout_ns"),
                       .max_lead = positive_duration(*this, "max_lead_ns")}} {
  const core::Duration poll_period = positive_duration(*this, "poll_period_ns");
  const rclcpp::QoS events = rclcpp::QoS(rclcpp::KeepLast(kEventDepth)).reliable();
  uplink_delivered_ = create_publisher<Payload>("/rovi/uplink/delivered", events);
  downlink_delivered_ = create_publisher<Payload>("/rovi/downlink/delivered", events);
  violations_ = create_publisher<ReservationViolation>("/rovi/link/violations", events);
  decisions_ = create_subscription<Decision>(
      "/rovi/decisions", events, [this](const Decision& decision) { on_decision(decision); });
  uplink_sent_ = create_subscription<Payload>(
      "/rovi/uplink/sent", events,
      [this](const Payload& payload) { on_sent(payload, Direction::kUplink); });
  downlink_sent_ = create_subscription<Payload>(
      "/rovi/downlink/sent", events,
      [this](const Payload& payload) { on_sent(payload, Direction::kDownlink); });
  poll_timer_ = create_timer(std::chrono::nanoseconds{poll_period.ns()}, [this] { on_poll(); });
  settings_ = telemetry::announce_settings(*this);
}

void LinkNode::on_decision(const Decision& decision) {
  const std::optional<core::TimePoint> now = clock_.now();
  if (!now) {
    return;
  }
  if (decision.status != Decision::STATUS_ADMITTED || !decision.has_reservation) {
    return;
  }
  const core::JobId job{decision.job_id};
  const auto& reservation = decision.reservation;
  const core::RobotId robot{decision.robot_id};
  const core::WorkerId worker{reservation.worker_id};
  const bool uplink = link_.lease(
      job, Direction::kUplink,
      lease_between(
          reservation.upload_start_ns, reservation.observation_ready_ns,
          {.robot = robot, .worker = worker, .bytes = core::Bytes{reservation.upload_bytes}}),
      *now);
  const bool downlink = link_.lease(
      job, Direction::kDownlink,
      lease_between(
          reservation.download_start_ns, reservation.delivery_ns,
          {.robot = robot, .worker = worker, .bytes = core::Bytes{reservation.download_bytes}}),
      *now);
  if (!uplink || !downlink) {
    RCLCPP_WARN_STREAM_THROTTLE(
        get_logger(), *get_clock(), kWarningThrottleMs,
        "ignored an invalid or repeated reservation for job " << decision.job_id);
  }
}

void LinkNode::on_sent(const Payload& payload, Direction direction) {
  const std::optional<core::TimePoint> now = clock_.now();
  if (!now) {
    return;
  }
  const core::JobId job{payload.job_id};
  const std::uint8_t expected =
      direction == Direction::kUplink ? Payload::KIND_OBSERVATION : Payload::KIND_ACTION_CHUNK;
  if (payload.kind != expected) {
    report(Refusal{.job = job,
                   .direction = direction,
                   .reason = Violation::kWrongDirection,
                   .claimed = claimed_by(payload),
                   .sent = *now,
                   .detected = *now,
                   .lease = std::nullopt});
    return;
  }
  link_.send(job, direction, claimed_by(payload), payload, *now);
}

void LinkNode::on_poll() {
  const std::optional<core::TimePoint> now = clock_.now();
  if (!now) {
    return;
  }
  LogicalLink<Payload>::Events events = link_.deliver(*now);
  for (auto& delivery : events.deliveries) {
    Payload& payload = delivery.payload;
    payload.delivered_ns = now->ns();
    payload.service_source.value = ServiceSource::LOGICAL;
    (delivery.direction == Direction::kUplink ? uplink_delivered_ : downlink_delivered_)
        ->publish(payload);
  }
  for (const Refusal& refusal : events.refusals) {
    report(refusal);
  }
  if (link_.unused_leases() != reported_unused_) {
    reported_unused_ = link_.unused_leases();
    RCLCPP_WARN_STREAM_THROTTLE(
        get_logger(), *get_clock(), kWarningThrottleMs,
        reported_unused_ << " reserved leases expired unused; their service stays spent");
  }
}

void LinkNode::report(const Refusal& refusal) {
  violations_->publish(to_violation(refusal));
  RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), kWarningThrottleMs,
      "refused the " << (refusal.direction == Direction::kUplink ? "uplink" : "downlink")
                     << " payload of job " << refusal.job.value() << " (violation "
                     << static_cast<int>(refusal.reason) << ")");
}

}  // namespace rovi::network

RCLCPP_COMPONENTS_REGISTER_NODE(rovi::network::LinkNode)
