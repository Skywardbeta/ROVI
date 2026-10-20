#include "rovi_scheduler/conversions.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "rovi_core/admission.hpp"
#include "rovi_core/capacity_calendar.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/scheduler.hpp"
#include "rovi_core/units.hpp"
#include "rovi_core/window.hpp"
#include "rovi_core/wireless_profile.hpp"
#include "rovi_interfaces/msg/decision.hpp"
#include "rovi_interfaces/msg/job_request.hpp"
#include "rovi_interfaces/msg/reservation.hpp"
#include "rovi_interfaces/msg/run_metadata.hpp"
#include "rovi_interfaces/msg/service_source.hpp"

namespace rovi::scheduler {
namespace {

using rovi_interfaces::msg::Decision;
using rovi_interfaces::msg::RunMetadata;
using rovi_interfaces::msg::ServiceSource;

constexpr std::int64_t kNsPerMs = 1'000'000;

core::TimePoint at_ms(std::int64_t count) { return core::TimePoint::from_ns(count * kNsPerMs); }

core::Duration ms(std::int64_t count) { return core::Duration::from_ns(count * kNsPerMs); }

rovi_interfaces::msg::JobRequest full_request() {
  rovi_interfaces::msg::JobRequest request;
  request.job_id = 42;
  request.robot_id = 7;
  request.sequence = 9;
  request.release_time_ns = 100;
  request.observation_time_ns = 90;
  request.buffer_empty_time_ns = 900;
  request.safety_margin_ns = 50;
  request.has_task_deadline = true;
  request.task_deadline_ns = 800;
  request.has_maximum_result_age = true;
  request.maximum_result_age_ns = 400;
  request.upload_bytes = 1'000;
  request.action_chunk_bytes = 200;
  request.model_class = 3;
  request.eligible_workers = {1, 2};
  request.predecessors = {40, 41};
  request.has_synchronization_group = true;
  request.synchronization_group = 5;
  request.maximum_result_ready_skew_ns = 30;
  return request;
}

TEST(Conversions, JobRequestMapsEveryField) {
  const core::Job expected{.id = core::JobId{42},
                           .release_time = core::TimePoint::from_ns(100),
                           .observation_time = core::TimePoint::from_ns(90),
                           .buffer_empty_time = core::TimePoint::from_ns(900),
                           .safety_margin = core::Duration::from_ns(50),
                           .task_deadline = core::TimePoint::from_ns(800),
                           .maximum_result_age = core::Duration::from_ns(400),
                           .upload_bytes = core::Bytes{1'000},
                           .action_chunk_bytes = core::Bytes{200},
                           .model = core::ModelClass{3},
                           .eligible_workers = {core::WorkerId{1}, core::WorkerId{2}},
                           .robot = core::RobotId{7},
                           .predecessors = {core::JobId{40}, core::JobId{41}},
                           .synchronization_group = core::GroupId{5},
                           .maximum_result_ready_skew = core::Duration::from_ns(30),
                           .sequence = 9};

  EXPECT_EQ(to_job(full_request()), expected);
}

TEST(Conversions, AbsentOptionalFieldsStayAbsentWhateverTheirValues) {
  rovi_interfaces::msg::JobRequest request = full_request();
  request.has_task_deadline = false;
  request.has_maximum_result_age = false;
  request.has_synchronization_group = false;

  const core::Job job = to_job(request);

  EXPECT_FALSE(job.task_deadline.has_value());
  EXPECT_FALSE(job.maximum_result_age.has_value());
  EXPECT_FALSE(job.synchronization_group.has_value());
}

TEST(Conversions, StatusReasonPolicyAndSourceMatchTheMessageConstants) {
  EXPECT_EQ(to_status(core::DecisionStatus::kRejected), Decision::STATUS_REJECTED);
  EXPECT_EQ(to_status(core::DecisionStatus::kDeferred), Decision::STATUS_DEFERRED);
  EXPECT_EQ(to_status(core::DecisionStatus::kAdmitted), Decision::STATUS_ADMITTED);

  EXPECT_EQ(to_reason(core::ReasonCode::kNone), Decision::REASON_NONE);
  EXPECT_EQ(to_reason(core::ReasonCode::kInvalidSnapshot), Decision::REASON_INVALID_SNAPSHOT);
  EXPECT_EQ(to_reason(core::ReasonCode::kInvalidPayloadSize),
            Decision::REASON_INVALID_PAYLOAD_SIZE);
  EXPECT_EQ(to_reason(core::ReasonCode::kInvalidTiming), Decision::REASON_INVALID_TIMING);
  EXPECT_EQ(to_reason(core::ReasonCode::kDuplicateJob), Decision::REASON_DUPLICATE_JOB);
  EXPECT_EQ(to_reason(core::ReasonCode::kArithmeticOverflow), Decision::REASON_ARITHMETIC_OVERFLOW);
  EXPECT_EQ(to_reason(core::ReasonCode::kNoEligibleWorker), Decision::REASON_NO_ELIGIBLE_WORKER);
  EXPECT_EQ(to_reason(core::ReasonCode::kPredecessorIncomplete),
            Decision::REASON_PREDECESSOR_INCOMPLETE);
  EXPECT_EQ(to_reason(core::ReasonCode::kUplinkServiceUnavailable),
            Decision::REASON_UPLINK_SERVICE_UNAVAILABLE);
  EXPECT_EQ(to_reason(core::ReasonCode::kNoFeasibleChain), Decision::REASON_NO_FEASIBLE_CHAIN);
  EXPECT_EQ(to_reason(core::ReasonCode::kDeadlineMiss), Decision::REASON_DEADLINE_MISS);
  EXPECT_EQ(to_reason(core::ReasonCode::kStaleResult), Decision::REASON_STALE_RESULT);
  EXPECT_EQ(to_reason(core::ReasonCode::kGroupMemberUnavailable),
            Decision::REASON_GROUP_MEMBER_UNAVAILABLE);
  EXPECT_EQ(to_reason(core::ReasonCode::kGroupSkewExceeded), Decision::REASON_GROUP_SKEW_EXCEEDED);
  EXPECT_EQ(to_reason(core::ReasonCode::kQueueFull), Decision::REASON_QUEUE_FULL);
  EXPECT_EQ(to_reason(core::ReasonCode::kMalformedRequest), Decision::REASON_MALFORMED_REQUEST);

  EXPECT_EQ(to_policy(core::Policy::kLeastSlack), RunMetadata::POLICY_LEAST_SLACK);
  EXPECT_EQ(to_policy(core::Policy::kEarliestDeadline), RunMetadata::POLICY_EARLIEST_DEADLINE);
  EXPECT_EQ(to_policy(core::Policy::kFirstInFirstOut), RunMetadata::POLICY_FIRST_IN_FIRST_OUT);

  EXPECT_EQ(to_source(core::ServiceSource::kLogical).value, ServiceSource::LOGICAL);
  EXPECT_EQ(to_source(core::ServiceSource::kEnforced).value, ServiceSource::ENFORCED);
  EXPECT_EQ(to_source(core::ServiceSource::kMeasured).value, ServiceSource::MEASURED);
  EXPECT_EQ(to_source(core::ServiceSource::kReplayed).value, ServiceSource::REPLAYED);
}

TEST(Conversions, DecisionCarriesJobIdentityCycleTimeAndSourceWithoutAReservation) {
  const core::Job job = to_job(full_request());
  const core::Decision deferral{.status = core::DecisionStatus::kDeferred,
                                .reason = core::ReasonCode::kPredecessorIncomplete};

  const Decision message = to_decision(job, deferral,
                                       DecisionContext{.cycle = 12,
                                                       .decided = core::TimePoint::from_ns(555),
                                                       .source = core::ServiceSource::kMeasured});

  EXPECT_EQ(message.job_id, 42U);
  EXPECT_EQ(message.robot_id, 7U);
  EXPECT_EQ(message.model_class, 3U);
  EXPECT_EQ(message.cycle, 12U);
  EXPECT_EQ(message.decided_ns, 555);
  EXPECT_EQ(message.status, Decision::STATUS_DEFERRED);
  EXPECT_EQ(message.reason, Decision::REASON_PREDECESSOR_INCOMPLETE);
  EXPECT_EQ(message.service_source.value, ServiceSource::MEASURED);
  EXPECT_FALSE(message.has_reservation);
}

core::ServiceSegment rate(std::int64_t bits_per_second) {
  return core::ServiceSegment{.first_slot = 0,
                              .rate = core::BitsPerSecond{bits_per_second},
                              .reduction = core::Permille{0}};
}

core::ResourceSnapshot reference_window() {
  const core::ServiceProfile profile{.slot_length = ms(1),
                                     .uplink = {rate(1'000'000'000)},
                                     .downlink = {rate(400'000'000)},
                                     .uplink_delay = ms(2),
                                     .downlink_delay = ms(3)};
  const std::vector<core::WorkerSpec> workers{core::WorkerSpec{
      .id = core::WorkerId{1},
      .profiles = {core::ExecutionProfile{.model = core::ModelClass{1}, .duration = ms(10)}}}};
  return core::open_window(profile, workers, at_ms(0), 100).value();
}

core::Job reference_job() {
  return core::Job{.id = core::JobId{1},
                   .release_time = at_ms(0),
                   .observation_time = at_ms(0),
                   .buffer_empty_time = at_ms(50),
                   .safety_margin = ms(5),
                   .task_deadline = std::nullopt,
                   .maximum_result_age = ms(40),
                   .upload_bytes = core::Bytes{250'000},
                   .action_chunk_bytes = core::Bytes{100'000},
                   .model = core::ModelClass{1},
                   .eligible_workers = {core::WorkerId{1}}};
}

TEST(Conversions, ReservationReportsStageTimesFromTheSnapshotGeometry) {
  const core::AdmissionResult admission =
      core::admit(reference_window(), reference_job(), at_ms(0));
  ASSERT_TRUE(admission.reservation.has_value());

  const std::optional<rovi_interfaces::msg::Reservation> message =
      to_reservation(admission.reservation.value(), admission.snapshot);

  ASSERT_TRUE(message.has_value());
  EXPECT_EQ(message->worker_id, 1U);
  EXPECT_EQ(message->upload_bytes, 250'000);
  EXPECT_EQ(message->upload_start_ns, 0);
  EXPECT_EQ(message->upload_completion_ns, 2 * kNsPerMs);
  EXPECT_EQ(message->observation_ready_ns, 4 * kNsPerMs);
  EXPECT_EQ(message->compute_start_ns, 4 * kNsPerMs);
  EXPECT_EQ(message->compute_end_ns, 14 * kNsPerMs);
  EXPECT_EQ(message->download_bytes, 100'000);
  EXPECT_EQ(message->download_start_ns, 14 * kNsPerMs);
  EXPECT_EQ(message->download_completion_ns, 16 * kNsPerMs);
  EXPECT_EQ(message->delivery_ns, 19 * kNsPerMs);
  EXPECT_EQ(message->deadline_ns, 40 * kNsPerMs);
  EXPECT_EQ(message->slack_ns, 21 * kNsPerMs);
}

TEST(Conversions, ReservationRejectsSlicesOutsideTheSnapshotOrUnrepresentableTimes) {
  const core::ResourceSnapshot window = reference_window();
  const core::ChainReservation valid =
      core::admit(window, reference_job(), at_ms(0)).reservation.value();
  core::ChainReservation no_upload = valid;
  no_upload.upload.slices.clear();
  core::ChainReservation no_download = valid;
  no_download.download.slices.clear();
  core::ChainReservation beyond_horizon = valid;
  beyond_horizon.download.slices.front().slot = core::SlotIndex{1'000};
  core::ChainReservation unrepresentable_ready = valid;
  unrepresentable_ready.upload.completion =
      core::TimePoint::from_ns(std::numeric_limits<std::int64_t>::max());
  core::ChainReservation overflowing_bytes = valid;
  overflowing_bytes.upload.slices.front().bytes =
      core::Bytes{std::numeric_limits<std::int64_t>::max()};

  EXPECT_EQ(to_reservation(no_upload, window), std::nullopt);
  EXPECT_EQ(to_reservation(no_download, window), std::nullopt);
  EXPECT_EQ(to_reservation(beyond_horizon, window), std::nullopt);
  EXPECT_EQ(to_reservation(unrepresentable_ready, window), std::nullopt);
  EXPECT_EQ(to_reservation(overflowing_bytes, window), std::nullopt);
}

}  // namespace
}  // namespace rovi::scheduler
