#include "rovi_scheduler/conversions.hpp"

#include <algorithm>
#include <cstdint>
#include <optional>
#include <vector>

#include "rovi_core/admission.hpp"
#include "rovi_core/capacity_calendar.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/scheduler.hpp"
#include "rovi_core/units.hpp"
#include "rovi_interfaces/msg/decision.hpp"
#include "rovi_interfaces/msg/job_request.hpp"
#include "rovi_interfaces/msg/reservation.hpp"
#include "rovi_interfaces/msg/run_metadata.hpp"
#include "rovi_interfaces/msg/service_source.hpp"

namespace rovi::scheduler {
namespace {

using rovi_interfaces::msg::Decision;

template <typename Strong>
std::vector<Strong> strong_values(const std::vector<std::uint64_t>& values) {
  std::vector<Strong> result(values.size(), Strong{0});
  std::ranges::transform(values, result.begin(), [](std::uint64_t value) { return Strong{value}; });
  return result;
}

std::optional<std::int64_t> total_bytes(const core::Transfer& transfer) {
  std::int64_t total = 0;
  const bool overflow =
      std::ranges::any_of(transfer.slices, [&total](const core::ServiceSlice& slice) {
        return __builtin_add_overflow(total, slice.bytes.value(), &total);
      });
  return overflow ? std::nullopt : std::optional{total};
}

}  // namespace

core::Job to_job(const rovi_interfaces::msg::JobRequest& request) {
  return core::Job{
      .id = core::JobId{request.job_id},
      .release_time = core::TimePoint::from_ns(request.release_time_ns),
      .observation_time = core::TimePoint::from_ns(request.observation_time_ns),
      .buffer_empty_time = core::TimePoint::from_ns(request.buffer_empty_time_ns),
      .safety_margin = core::Duration::from_ns(request.safety_margin_ns),
      .task_deadline = request.has_task_deadline
                           ? std::optional{core::TimePoint::from_ns(request.task_deadline_ns)}
                           : std::nullopt,
      .maximum_result_age =
          request.has_maximum_result_age
              ? std::optional{core::Duration::from_ns(request.maximum_result_age_ns)}
              : std::nullopt,
      .upload_bytes = core::Bytes{request.upload_bytes},
      .action_chunk_bytes = core::Bytes{request.action_chunk_bytes},
      .model = core::ModelClass{request.model_class},
      .eligible_workers = strong_values<core::WorkerId>(request.eligible_workers),
      .robot = core::RobotId{request.robot_id},
      .predecessors = strong_values<core::JobId>(request.predecessors),
      .synchronization_group = request.has_synchronization_group
                                   ? std::optional{core::GroupId{request.synchronization_group}}
                                   : std::nullopt,
      .maximum_result_ready_skew = core::Duration::from_ns(request.maximum_result_ready_skew_ns),
      .sequence = request.sequence};
}

std::uint8_t to_status(core::DecisionStatus status) {
  switch (status) {
    case core::DecisionStatus::kRejected:
      return Decision::STATUS_REJECTED;
    case core::DecisionStatus::kDeferred:
      return Decision::STATUS_DEFERRED;
    case core::DecisionStatus::kAdmitted:
      return Decision::STATUS_ADMITTED;
  }
  return Decision::STATUS_REJECTED;
}

std::uint8_t to_reason(core::ReasonCode reason) {
  switch (reason) {
    case core::ReasonCode::kNone:
      return Decision::REASON_NONE;
    case core::ReasonCode::kInvalidSnapshot:
      return Decision::REASON_INVALID_SNAPSHOT;
    case core::ReasonCode::kInvalidPayloadSize:
      return Decision::REASON_INVALID_PAYLOAD_SIZE;
    case core::ReasonCode::kInvalidTiming:
      return Decision::REASON_INVALID_TIMING;
    case core::ReasonCode::kDuplicateJob:
      return Decision::REASON_DUPLICATE_JOB;
    case core::ReasonCode::kArithmeticOverflow:
      return Decision::REASON_ARITHMETIC_OVERFLOW;
    case core::ReasonCode::kNoEligibleWorker:
      return Decision::REASON_NO_ELIGIBLE_WORKER;
    case core::ReasonCode::kPredecessorIncomplete:
      return Decision::REASON_PREDECESSOR_INCOMPLETE;
    case core::ReasonCode::kUplinkServiceUnavailable:
      return Decision::REASON_UPLINK_SERVICE_UNAVAILABLE;
    case core::ReasonCode::kNoFeasibleChain:
      return Decision::REASON_NO_FEASIBLE_CHAIN;
    case core::ReasonCode::kDeadlineMiss:
      return Decision::REASON_DEADLINE_MISS;
    case core::ReasonCode::kStaleResult:
      return Decision::REASON_STALE_RESULT;
    case core::ReasonCode::kGroupMemberUnavailable:
      return Decision::REASON_GROUP_MEMBER_UNAVAILABLE;
    case core::ReasonCode::kGroupSkewExceeded:
      return Decision::REASON_GROUP_SKEW_EXCEEDED;
    case core::ReasonCode::kQueueFull:
      return Decision::REASON_QUEUE_FULL;
    case core::ReasonCode::kMalformedRequest:
      return Decision::REASON_MALFORMED_REQUEST;
  }
  return Decision::REASON_NONE;
}

std::uint8_t to_policy(core::Policy policy) {
  using rovi_interfaces::msg::RunMetadata;
  switch (policy) {
    case core::Policy::kLeastSlack:
      return RunMetadata::POLICY_LEAST_SLACK;
    case core::Policy::kEarliestDeadline:
      return RunMetadata::POLICY_EARLIEST_DEADLINE;
    case core::Policy::kFirstInFirstOut:
      return RunMetadata::POLICY_FIRST_IN_FIRST_OUT;
  }
  return RunMetadata::POLICY_LEAST_SLACK;
}

rovi_interfaces::msg::ServiceSource to_source(core::ServiceSource source) {
  using rovi_interfaces::msg::ServiceSource;
  ServiceSource message;
  switch (source) {
    case core::ServiceSource::kLogical:
      message.value = ServiceSource::LOGICAL;
      break;
    case core::ServiceSource::kEnforced:
      message.value = ServiceSource::ENFORCED;
      break;
    case core::ServiceSource::kMeasured:
      message.value = ServiceSource::MEASURED;
      break;
    case core::ServiceSource::kReplayed:
      message.value = ServiceSource::REPLAYED;
      break;
  }
  return message;
}

rovi_interfaces::msg::Decision to_decision(const core::Job& job, const core::Decision& decision,
                                           const DecisionContext& context) {
  Decision message;
  message.job_id = job.id.value();
  message.robot_id = job.robot.value();
  message.model_class = job.model.value();
  message.cycle = context.cycle;
  message.decided_ns = context.decided.ns();
  message.status = to_status(decision.status);
  message.reason = to_reason(decision.reason);
  message.service_source = to_source(context.source);
  message.has_reservation = false;
  return message;
}

std::optional<rovi_interfaces::msg::Reservation> to_reservation(
    const core::ChainReservation& reservation, const core::ResourceSnapshot& snapshot) {
  if (reservation.upload.slices.empty() || reservation.download.slices.empty()) {
    return std::nullopt;
  }
  const std::optional<core::TimePoint> upload_start =
      snapshot.uplink.slot_start(reservation.upload.slices.front().slot);
  const std::optional<core::TimePoint> download_start =
      snapshot.downlink.slot_start(reservation.download.slices.front().slot);
  const std::optional<core::TimePoint> observation_ready =
      core::checked_add(reservation.upload.completion, snapshot.uplink_delay);
  const std::optional<std::int64_t> upload_bytes = total_bytes(reservation.upload);
  const std::optional<std::int64_t> download_bytes = total_bytes(reservation.download);
  if (!upload_start || !download_start || !observation_ready || !upload_bytes || !download_bytes) {
    return std::nullopt;
  }
  rovi_interfaces::msg::Reservation message;
  message.worker_id = reservation.worker.value();
  message.upload_bytes = *upload_bytes;
  message.upload_start_ns = upload_start->ns();
  message.upload_completion_ns = reservation.upload.completion.ns();
  message.observation_ready_ns = observation_ready->ns();
  message.compute_start_ns = reservation.compute.start.ns();
  message.compute_end_ns = reservation.compute.end.ns();
  message.download_bytes = *download_bytes;
  message.download_start_ns = download_start->ns();
  message.download_completion_ns = reservation.download.completion.ns();
  message.delivery_ns = reservation.delivery.ns();
  message.deadline_ns = reservation.deadline.ns();
  message.slack_ns = reservation.slack.ns();
  return message;
}

}  // namespace rovi::scheduler
