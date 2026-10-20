#include "rovi_coordination/coordinator_node.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
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

#include "rovi_coordination/group.hpp"
#include "rovi_coordination/group_assembler.hpp"
#include "rovi_coordination/group_releaser.hpp"
#include "rovi_core/admission.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/units.hpp"
#include "rovi_interfaces/msg/decision.hpp"
#include "rovi_interfaces/msg/group_release.hpp"
#include "rovi_interfaces/msg/job_request.hpp"
#include "rovi_interfaces/msg/payload.hpp"
#include "rovi_scheduler/conversions.hpp"
#include "rovi_telemetry/run_clock.hpp"

namespace rovi::coordination {
namespace {

using rovi_interfaces::msg::Decision;
using rovi_interfaces::msg::GroupRelease;
using rovi_interfaces::msg::JobRequest;
using rovi_interfaces::msg::Payload;

constexpr std::size_t kEventDepth = 1'000;
constexpr int kLogThrottleMs = 1'000;
constexpr std::int64_t kMaximumWorkcells = 1'000;
constexpr std::int64_t kMaximumTrackedGroups = 1'024;
constexpr std::int64_t kShortestPeriodNs = 1'000'000;
constexpr std::int64_t kLongestPeriodNs = 1'000'000'000;

[[noreturn]] void reject(const std::string& name) {
  throw std::invalid_argument{"coordinator: invalid parameter " + name};
}

std::int64_t bounded(rclcpp::Node& node, const std::string& name, std::int64_t low,
                     std::int64_t high) {
  const std::int64_t value = node.declare_parameter<std::int64_t>(name);
  if (value < low || value > high) {
    reject(name);
  }
  return value;
}

GroupAssembler load_assembler(rclcpp::Node& node) {
  const std::int64_t workcells = bounded(node, "workcells", 1, kMaximumWorkcells);
  const std::int64_t team =
      bounded(node, "robots_per_workcell", 1, static_cast<std::int64_t>(kMaximumWorkcellSize));
  const core::Duration skew = core::Duration::from_ns(
      bounded(node, "maximum_result_ready_skew_ns", 0, std::numeric_limits<std::int64_t>::max()));
  std::vector<Workcell> cells;
  for (std::int64_t cell = 0; cell < workcells; ++cell) {
    Workcell workcell{.id = static_cast<std::uint32_t>(cell), .members = {}, .skew = skew};
    for (std::int64_t member = 0; member < team; ++member) {
      workcell.members.emplace_back(static_cast<std::uint32_t>((cell * team) + member));
    }
    cells.push_back(std::move(workcell));
  }
  std::optional<GroupAssembler> assembler = GroupAssembler::create(std::move(cells));
  if (!assembler) {
    reject("workcells or robots_per_workcell");
  }
  return std::move(*assembler);
}

}  // namespace

CoordinatorNode::CoordinatorNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("rovi_coordinator", options),
      clock_{*this},
      assembler_{load_assembler(*this)},
      releaser_{
          static_cast<std::size_t>(bounded(*this, "max_tracked_groups", 1, kMaximumTrackedGroups)),
          core::Duration::from_ns(bounded(*this, "expiry_grace_ns", 0, kLongestPeriodNs))} {
  const std::int64_t expiry_period =
      bounded(*this, "expiry_period_ns", kShortestPeriodNs, kLongestPeriodNs);
  const rclcpp::QoS events = rclcpp::QoS(rclcpp::KeepLast(kEventDepth)).reliable();
  requests_ = create_publisher<JobRequest>("/rovi/job_requests", events);
  releases_ = create_publisher<GroupRelease>("/rovi/coordination/releases", events);
  proposal_subscription_ =
      create_subscription<JobRequest>("/rovi/coordination/proposals", events,
                                      [this](const JobRequest& request) { on_proposal(request); });
  decisions_ = create_subscription<Decision>(
      "/rovi/decisions", events, [this](const Decision& decision) { on_decision(decision); });
  deliveries_ = create_subscription<Payload>(
      "/rovi/downlink/delivered", events, [this](const Payload& payload) { on_delivery(payload); });
  expiry_timer_ = create_timer(std::chrono::nanoseconds{expiry_period}, [this] { on_expiry(); });
  settings_ = telemetry::announce_settings(*this);
}

void CoordinatorNode::on_proposal(const JobRequest& request) {
  const std::optional<core::TimePoint> now = clock_.now();
  if (!now) {
    return;
  }
  const core::Job job = scheduler::to_job(request);
  const ProposalOutcome outcome = assembler_.propose(job);
  if (outcome.status == ProposalStatus::kUnknownRobot ||
      outcome.status == ProposalStatus::kAlreadyGrouped ||
      outcome.status == ProposalStatus::kMalformed) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
                         "ignored proposal %lu from robot %u", request.job_id, request.robot_id);
    return;
  }
  proposals_.insert_or_assign(job.robot, request);
  if (outcome.group) {
    submit(*outcome.group, *now);
  }
}

void CoordinatorNode::submit(const AssembledGroup& group, core::TimePoint now) {
  if (!releaser_.track(group)) {
    ReleaseOutcome refused{.workcell = group.workcell,
                           .group = group.group,
                           .released = false,
                           .reason = core::ReasonCode::kQueueFull,
                           .release_time = now,
                           .skew = core::Duration{},
                           .jobs = {},
                           .robots = {}};
    for (const core::Job& member : group.members) {
      refused.jobs.push_back(member.id);
      refused.robots.push_back(member.robot);
      proposals_.erase(member.robot);
    }
    publish(refused);
    return;
  }
  for (const core::Job& member : group.members) {
    const auto proposal = proposals_.find(member.robot);
    if (proposal == proposals_.end()) {
      continue;
    }
    JobRequest request = proposal->second;
    proposals_.erase(proposal);
    request.has_synchronization_group = true;
    request.synchronization_group = group.group.value();
    request.maximum_result_ready_skew_ns = member.maximum_result_ready_skew.ns();
    request.synchronization_group_size = static_cast<std::uint32_t>(group.members.size());
    request.has_task_deadline = member.task_deadline.has_value();
    request.task_deadline_ns = member.task_deadline.value_or(core::TimePoint{}).ns();
    requests_->publish(request);
  }
}

void CoordinatorNode::on_decision(const Decision& decision) {
  if (!clock_.now()) {
    return;
  }
  if (decision.status == Decision::STATUS_REJECTED) {
    releaser_.on_decision(core::JobId{decision.job_id}, core::DecisionStatus::kRejected,
                          decision.reason == Decision::REASON_DUPLICATE_JOB
                              ? core::ReasonCode::kDuplicateJob
                              : core::ReasonCode::kNone);
  }
}

void CoordinatorNode::on_delivery(const Payload& payload) {
  if (!clock_.now()) {
    return;
  }
  if (payload.kind != Payload::KIND_ACTION_CHUNK) {
    return;
  }
  if (const std::optional<ReleaseOutcome> outcome = releaser_.on_delivery(
          core::JobId{payload.job_id}, core::TimePoint::from_ns(payload.delivered_ns))) {
    publish(*outcome);
  }
}

void CoordinatorNode::on_expiry() {
  const std::optional<core::TimePoint> tick = clock_.now();
  if (!tick) {
    return;
  }
  const core::TimePoint now = *tick;
  for (const AssembledGroup& dropped : assembler_.expire(now)) {
    ReleaseOutcome outcome{.workcell = dropped.workcell,
                           .group = dropped.group,
                           .released = false,
                           .reason = core::ReasonCode::kGroupMemberUnavailable,
                           .release_time = now,
                           .skew = core::Duration{},
                           .jobs = {},
                           .robots = {}};
    for (const core::Job& member : dropped.members) {
      outcome.jobs.push_back(member.id);
      outcome.robots.push_back(member.robot);
      proposals_.erase(member.robot);
    }
    publish(outcome);
  }
  for (const ReleaseOutcome& outcome : releaser_.expire(now)) {
    publish(outcome);
  }
}

void CoordinatorNode::publish(const ReleaseOutcome& outcome) {
  GroupRelease release;
  release.workcell = outcome.workcell;
  release.synchronization_group = outcome.group.value();
  release.released = outcome.released;
  release.reason = scheduler::to_reason(outcome.reason);
  release.release_time_ns = outcome.release_time.ns();
  release.result_ready_skew_ns = outcome.skew.ns();
  for (const core::JobId job : outcome.jobs) {
    release.jobs.push_back(job.value());
  }
  for (const core::RobotId robot : outcome.robots) {
    release.robots.push_back(robot.value());
  }
  releases_->publish(release);
}

}  // namespace rovi::coordination

RCLCPP_COMPONENTS_REGISTER_NODE(rovi::coordination::CoordinatorNode)
