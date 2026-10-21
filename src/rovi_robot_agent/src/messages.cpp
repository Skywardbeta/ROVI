#include "rovi_robot_agent/messages.hpp"

#include <algorithm>
#include <cstdint>
#include <optional>
#include <vector>

#include "rovi_core/admission.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/receipt.hpp"
#include "rovi_core/units.hpp"
#include "rovi_interfaces/msg/decision.hpp"
#include "rovi_interfaces/msg/job_request.hpp"
#include "rovi_interfaces/msg/robot_state.hpp"
#include "rovi_robot_agent/robot_agent.hpp"

namespace rovi::robot_agent {
namespace {

template <typename Strong>
std::vector<std::uint64_t> raw_values(const std::vector<Strong>& values) {
  std::vector<std::uint64_t> result(values.size());
  std::ranges::transform(values, result.begin(), [](Strong value) { return value.value(); });
  return result;
}

}  // namespace

rovi_interfaces::msg::JobRequest to_request(const core::Job& job) {
  rovi_interfaces::msg::JobRequest request;
  request.job_id = job.id.value();
  request.robot_id = job.robot.value();
  request.sequence = job.sequence;
  request.release_time_ns = job.release_time.ns();
  request.observation_time_ns = job.observation_time.ns();
  request.buffer_empty_time_ns = job.buffer_empty_time.ns();
  request.safety_margin_ns = job.safety_margin.ns();
  request.has_task_deadline = job.task_deadline.has_value();
  request.task_deadline_ns = job.task_deadline.value_or(core::TimePoint{}).ns();
  request.has_maximum_result_age = job.maximum_result_age.has_value();
  request.maximum_result_age_ns = job.maximum_result_age.value_or(core::Duration{}).ns();
  request.upload_bytes = job.upload_bytes.value();
  request.action_chunk_bytes = job.action_chunk_bytes.value();
  request.model_class = job.model.value();
  request.eligible_workers = raw_values(job.eligible_workers);
  request.predecessors = raw_values(job.predecessors);
  request.has_synchronization_group = job.synchronization_group.has_value();
  request.synchronization_group = job.synchronization_group.value_or(core::GroupId{0}).value();
  request.maximum_result_ready_skew_ns = job.maximum_result_ready_skew.ns();
  return request;
}

std::optional<core::DecisionStatus> status_from(std::uint8_t status) {
  using rovi_interfaces::msg::Decision;
  switch (status) {
    case Decision::STATUS_REJECTED:
      return core::DecisionStatus::kRejected;
    case Decision::STATUS_DEFERRED:
      return core::DecisionStatus::kDeferred;
    case Decision::STATUS_ADMITTED:
      return core::DecisionStatus::kAdmitted;
    default:
      return std::nullopt;
  }
}

rovi_interfaces::msg::RobotState to_state(const RobotAgent& agent, core::TimePoint now) {
  using rovi_interfaces::msg::RobotState;
  const Counters& counters = agent.counters();
  const std::optional<core::Duration> remaining = core::elapsed(now, agent.buffer_empty_time());
  RobotState state;
  state.robot_id = agent.robot().value();
  state.stamp_ns = now.ns();
  state.mode =
      agent.mode() == Mode::kExecuting ? RobotState::MODE_EXECUTING : RobotState::MODE_FALLBACK;
  state.buffer_empty_time_ns = agent.buffer_empty_time().ns();
  state.buffer_remaining_ns = std::max<std::int64_t>(remaining.value_or(core::Duration{}).ns(), 0);
  state.has_job_in_flight = agent.in_flight().has_value();
  state.job_in_flight = agent.in_flight() ? agent.in_flight()->job.id.value() : 0U;
  state.jobs_submitted = counters.jobs_submitted;
  state.jobs_rejected = counters.jobs_rejected;
  state.chunks_accepted = counters.chunks_accepted;
  state.chunks_late = counters.chunks_late;
  state.chunks_stale = counters.chunks_stale;
  state.chunks_missed = counters.chunks_missed;
  state.fallback_entries = counters.fallback_entries;
  state.chunks_invalid = counters.chunks_invalid;
  const core::ReceiptSummary receipts = agent.receipts().summary(now);
  state.requests_offered = receipts.offered;
  state.receipts_on_time = receipts.on_time;
  state.receipts_late = receipts.late;
  state.requests_missing = receipts.missing;
  state.requests_pending = receipts.pending;
  return state;
}

}  // namespace rovi::robot_agent
