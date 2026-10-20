#include "rovi_core/admission.hpp"

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <optional>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "rovi_core/capacity_calendar.hpp"
#include "rovi_core/interval_calendar.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/units.hpp"

#include "printers.hpp"  // IWYU pragma: keep
#include "scenario.hpp"
#include "test_support.hpp"

namespace rovi::core {
namespace {

ChainReservation chain_through(std::uint64_t worker_id, std::int64_t upload_slot, Interval compute,
                               std::int64_t download_slot) {
  return ChainReservation{.job = JobId{1},
                          .worker = WorkerId{worker_id},
                          .upload = transfer_in_slots(upload_slot, kUplinkBytesPerSlot),
                          .compute = compute,
                          .download = transfer_in_slots(download_slot, kDownlinkBytesPerSlot),
                          .deadline = at_ms(100),
                          .slack = ms(0),
                          .delivery = at_ms(download_slot + 2)};
}

ChainReservation reference_chain() { return chain_through(0, 0, window_ms(2, 7), 7); }

void expect_not_admitted(const ResourceSnapshot& snapshot, const Job& job, TimePoint now,
                         Decision expected) {
  const AdmissionResult result = admit(snapshot, job, now);

  EXPECT_EQ(result.decision, expected);
  EXPECT_FALSE(result.reservation.has_value());
  EXPECT_EQ(result.snapshot, snapshot);
}

void expect_admitted_on_worker_zero(std::vector<Worker> workers) {
  Job job = reference_job();
  job.eligible_workers = {WorkerId{1}, WorkerId{0}};

  const AdmissionResult result = admit(empty_snapshot(std::move(workers)), job, at_ms(0));

  ASSERT_EQ(result.decision, admitted());
  EXPECT_EQ(result.reservation.value().worker, WorkerId{0});
}

TEST(SingleChainAdmission, AdmitsChainIntoEmptyCalendars) {
  const ResourceSnapshot snapshot = empty_snapshot({worker(0, ms(5))});

  const AdmissionResult result = admit(snapshot, reference_job(), at_ms(0));

  ASSERT_EQ(result.decision, admitted());
  ASSERT_TRUE(result.reservation.has_value());
  const ChainReservation& chain = result.reservation.value();
  EXPECT_EQ(chain.job, JobId{1});
  EXPECT_EQ(chain.worker, WorkerId{0});
  EXPECT_EQ(chain.upload, (Transfer{.slices = {{.slot = SlotIndex{0}, .bytes = Bytes{125'000}},
                                               {.slot = SlotIndex{1}, .bytes = Bytes{125'000}}},
                                    .completion = at_ms(2)}));
  EXPECT_EQ(chain.compute, window_ms(2, 7));
  EXPECT_EQ(chain.download, (Transfer{.slices = {{.slot = SlotIndex{7}, .bytes = Bytes{50'000}},
                                                 {.slot = SlotIndex{8}, .bytes = Bytes{50'000}}},
                                      .completion = at_ms(9)}));
  EXPECT_EQ(chain.deadline, at_ms(40));
  EXPECT_EQ(chain.slack, ms(31));
  EXPECT_EQ(reserved_bytes(result.snapshot.uplink),
            expected_reserved({{0, 125'000}, {1, 125'000}}));
  EXPECT_EQ(reserved_bytes(result.snapshot.downlink),
            expected_reserved({{7, 50'000}, {8, 50'000}}));
  ASSERT_EQ(result.snapshot.workers.size(), 1U);
  EXPECT_EQ(result.snapshot.workers.front().lane.intervals(),
            (std::vector<Interval>{window_ms(2, 7)}));
  EXPECT_EQ(reserved_bytes(snapshot.uplink), expected_reserved({}));
}

TEST(SingleChainAdmission, UploadStartsNoEarlierThanNow) {
  const ResourceSnapshot snapshot = empty_snapshot({worker(0, ms(5))});

  const AdmissionResult result = admit(snapshot, reference_job(), at_ms(3));

  ASSERT_EQ(result.decision, admitted());
  const ChainReservation& chain = result.reservation.value();
  EXPECT_EQ(chain.upload, transfer_in_slots(3, kUplinkBytesPerSlot));
  EXPECT_EQ(chain.compute, window_ms(5, 10));
  EXPECT_EQ(chain.download, transfer_in_slots(10, kDownlinkBytesPerSlot));
  EXPECT_EQ(chain.slack, ms(28));
}

TEST(SingleChainAdmission, SecondAdmissionUsesResidualCapacityAndFreeLaneTime) {
  const ResourceSnapshot first =
      admit(empty_snapshot({worker(0, ms(5))}), reference_job(), at_ms(0)).snapshot;
  Job second_job = reference_job();
  second_job.id = JobId{2};

  const AdmissionResult second = admit(first, second_job, at_ms(0));

  ASSERT_EQ(second.decision, admitted());
  const ChainReservation& chain = second.reservation.value();
  EXPECT_EQ(chain.upload, transfer_in_slots(2, kUplinkBytesPerSlot));
  EXPECT_EQ(chain.compute, window_ms(7, 12));
  EXPECT_EQ(chain.download, transfer_in_slots(12, kDownlinkBytesPerSlot));
  EXPECT_EQ(chain.slack, ms(26));
  EXPECT_EQ(reserved_bytes(second.snapshot.uplink),
            expected_reserved({{0, 125'000}, {1, 125'000}, {2, 125'000}, {3, 125'000}}));
  EXPECT_EQ(reserved_bytes(second.snapshot.downlink),
            expected_reserved({{7, 50'000}, {8, 50'000}, {12, 50'000}, {13, 50'000}}));
  EXPECT_EQ(second.snapshot.workers.front().lane.intervals(),
            (std::vector<Interval>{window_ms(2, 7), window_ms(7, 12)}));
}

TEST(SingleChainAdmission, SelectsWorkerWithEarliestDelivery) {
  Job job = reference_job();
  job.eligible_workers = {WorkerId{0}, WorkerId{1}};
  const ResourceSnapshot snapshot = empty_snapshot({worker(0, ms(5)), worker(1, ms(3))});

  const AdmissionResult result = admit(snapshot, job, at_ms(0));

  ASSERT_EQ(result.decision, admitted());
  const ChainReservation& chain = result.reservation.value();
  EXPECT_EQ(chain.worker, WorkerId{1});
  EXPECT_EQ(chain.compute, window_ms(2, 5));
  EXPECT_EQ(chain.download, transfer_in_slots(5, kDownlinkBytesPerSlot));
  EXPECT_TRUE(result.snapshot.workers.at(0).lane.intervals().empty());
  EXPECT_EQ(result.snapshot.workers.at(1).lane.intervals(),
            (std::vector<Interval>{window_ms(2, 5)}));
}

TEST(SingleChainAdmission, BreaksDeliveryTiesByLowestWorkerIdRegardlessOfListOrder) {
  const ResourceSnapshot snapshot = empty_snapshot({worker(2, ms(5)), worker(1, ms(5))});
  Job higher_id_first = reference_job();
  higher_id_first.eligible_workers = {WorkerId{2}, WorkerId{1}};
  Job lower_id_first = reference_job();
  lower_id_first.eligible_workers = {WorkerId{1}, WorkerId{2}};

  const AdmissionResult from_higher_first = admit(snapshot, higher_id_first, at_ms(0));
  const AdmissionResult from_lower_first = admit(snapshot, lower_id_first, at_ms(0));

  ASSERT_EQ(from_higher_first.decision, admitted());
  ASSERT_EQ(from_lower_first.decision, admitted());
  EXPECT_EQ(from_higher_first.reservation.value().worker, WorkerId{1});
  EXPECT_EQ(from_lower_first.reservation.value().worker, WorkerId{1});
}

TEST(SingleChainAdmission, SkipsIneligibleAndInfeasibleWorkersListedFirst) {
  {
    SCOPED_TRACE("first listed worker is absent from the snapshot");
    expect_admitted_on_worker_zero({worker(0, ms(5))});
  }
  {
    SCOPED_TRACE("first listed worker has no profile for the model");
    expect_admitted_on_worker_zero({worker_for(1, ModelClass{9}, ms(5)), worker(0, ms(5))});
  }
  {
    SCOPED_TRACE("first listed worker has a non-positive execution duration");
    expect_admitted_on_worker_zero({worker(1, ms(0)), worker(0, ms(5))});
  }
  {
    SCOPED_TRACE("first listed worker's compute completion overflows");
    expect_admitted_on_worker_zero({worker(1, d(kMaxNs)), worker(0, ms(5))});
  }
  {
    SCOPED_TRACE("first listed worker's download would start after the horizon");
    expect_admitted_on_worker_zero({worker(1, ms(200)), worker(0, ms(5))});
  }
}

TEST(SingleChainAdmission, RejectsJobWithoutEligibleWorker) {
  const ResourceSnapshot snapshot = empty_snapshot({worker(0, ms(5))});
  Job absent_worker = reference_job();
  absent_worker.eligible_workers = {WorkerId{7}};
  Job unsupported_model = reference_job();
  unsupported_model.model = ModelClass{1};
  const Decision expected = outcome(DecisionStatus::kRejected, ReasonCode::kNoEligibleWorker);

  {
    SCOPED_TRACE("eligible worker absent from snapshot");
    expect_not_admitted(snapshot, absent_worker, at_ms(0), expected);
  }
  {
    SCOPED_TRACE("worker has no execution profile for the model");
    expect_not_admitted(snapshot, unsupported_model, at_ms(0), expected);
  }
}

TEST(SingleChainAdmission, TreatsWorkersWithNonPositiveExecutionDurationAsIneligible) {
  const Decision expected = outcome(DecisionStatus::kRejected, ReasonCode::kNoEligibleWorker);

  {
    SCOPED_TRACE("zero execution duration");
    expect_not_admitted(empty_snapshot({worker(0, ms(0))}), reference_job(), at_ms(0), expected);
  }
  {
    SCOPED_TRACE("negative execution duration");
    expect_not_admitted(empty_snapshot({worker(0, ms(-5))}), reference_job(), at_ms(0), expected);
  }
}

TEST(SingleChainAdmission, DefersWhenUplinkCannotCompleteWithinHorizon) {
  Job job = reference_job();
  job.upload_bytes = Bytes{(kHorizonSlots * kUplinkBytesPerSlot) + 1};

  expect_not_admitted(empty_snapshot({worker(0, ms(5))}), job, at_ms(0),
                      outcome(DecisionStatus::kDeferred, ReasonCode::kUplinkServiceUnavailable));
}

TEST(SingleChainAdmission, DefersWhenNoWorkerCanDeliverWithinHorizon) {
  const Decision expected = outcome(DecisionStatus::kDeferred, ReasonCode::kNoFeasibleChain);

  {
    SCOPED_TRACE("download would start after the horizon");
    expect_not_admitted(empty_snapshot({worker(0, ms(200))}), reference_job(), at_ms(0), expected);
  }
  {
    SCOPED_TRACE("compute completion overflows");
    expect_not_admitted(empty_snapshot({worker(0, d(kMaxNs))}), reference_job(), at_ms(0),
                        expected);
  }
}

TEST(SingleChainAdmission, RejectsDeliveryAfterActionBufferDeadline) {
  const Decision expected = outcome(DecisionStatus::kRejected, ReasonCode::kDeadlineMiss);
  Job buffer_bound = reference_job();
  buffer_bound.maximum_result_age = ms(100);

  {
    SCOPED_TRACE("inference too slow for the deadline");
    expect_not_admitted(empty_snapshot({worker(0, ms(60))}), buffer_bound, at_ms(0), expected);
  }
  {
    SCOPED_TRACE("current time already past the deadline");
    expect_not_admitted(empty_snapshot({worker(0, ms(5))}), buffer_bound, at_ms(46), expected);
  }
}

TEST(SingleChainAdmission, RejectsStaleResultDespiteRemainingActionBuffer) {
  Job fresh_for_eight_ms = reference_job();
  fresh_for_eight_ms.maximum_result_age = ms(8);

  expect_not_admitted(empty_snapshot({worker(0, ms(5))}), fresh_for_eight_ms, at_ms(0),
                      outcome(DecisionStatus::kRejected, ReasonCode::kStaleResult));
}

TEST(SingleChainAdmission, AppliesUplinkAndDownlinkPathDelays) {
  ResourceSnapshot snapshot = empty_snapshot({worker(0, ms(5))});
  snapshot.uplink_delay = ms(1);
  snapshot.downlink_delay = ms(2);

  const AdmissionResult result = admit(snapshot, reference_job(), at_ms(0));

  ASSERT_EQ(result.decision, admitted());
  const ChainReservation& chain = result.reservation.value();
  EXPECT_EQ(chain.upload.completion, at_ms(2));
  EXPECT_EQ(chain.compute, window_ms(3, 8));
  EXPECT_EQ(chain.download, transfer_in_slots(8, kDownlinkBytesPerSlot));
  EXPECT_EQ(chain.delivery, at_ms(12));
  EXPECT_EQ(chain.slack, ms(28));
}

TEST(SingleChainAdmission, LeavesTheDispatchLeadBeforeEveryReservedStage) {
  ResourceSnapshot snapshot = empty_snapshot({worker(0, ms(5))});
  snapshot.dispatch_lead = ms(1);

  const AdmissionResult result = admit(snapshot, reference_job(), at_ms(0));

  ASSERT_EQ(result.decision, admitted());
  const ChainReservation& chain = result.reservation.value();
  EXPECT_EQ(chain.upload, transfer_in_slots(1, kUplinkBytesPerSlot));
  EXPECT_EQ(chain.compute, window_ms(4, 9));
  EXPECT_EQ(chain.download, transfer_in_slots(10, kDownlinkBytesPerSlot));
  EXPECT_EQ(chain.delivery, at_ms(12));
  EXPECT_EQ(chain.slack, ms(28));
}

TEST(SingleChainAdmission, StartsEachNetworkStageAtTheFirstSlotBoundaryAfterTheDispatchLead) {
  ResourceSnapshot snapshot = empty_snapshot({worker(0, ms(5))});
  snapshot.dispatch_lead = d(1);

  const AdmissionResult result = admit(snapshot, reference_job(), at_ms(0));

  ASSERT_EQ(result.decision, admitted());
  const ChainReservation& chain = result.reservation.value();
  EXPECT_EQ(chain.upload, transfer_in_slots(1, kUplinkBytesPerSlot));
  EXPECT_EQ(chain.compute,
            (Interval{.start = t((3 * kNsPerMs) + 1), .end = t((8 * kNsPerMs) + 1)}));
  EXPECT_EQ(chain.download, transfer_in_slots(9, kDownlinkBytesPerSlot));
}

TEST(SingleChainAdmission, SharedMediumLimitsCombinedUplinkAndDownlinkService) {
  ResourceSnapshot snapshot = empty_snapshot({worker(0, ms(5))});
  snapshot.shared_medium = uniform_calendar(60'000);

  const AdmissionResult result = admit(snapshot, reference_job(), at_ms(0));

  ASSERT_EQ(result.decision, admitted());
  const ChainReservation& chain = result.reservation.value();
  EXPECT_EQ(chain.upload.completion, at_ms(5));
  EXPECT_EQ(chain.compute, window_ms(5, 10));
  EXPECT_EQ(chain.delivery, at_ms(12));
  EXPECT_EQ(reserved_bytes(result.snapshot.shared_medium.value()),
            expected_reserved({{0, 60'000},
                               {1, 60'000},
                               {2, 60'000},
                               {3, 60'000},
                               {4, 10'000},
                               {10, 50'000},
                               {11, 50'000}}));
}

TEST(SingleChainAdmission, RejectsInconsistentResourceSnapshot) {
  const Decision expected = outcome(DecisionStatus::kRejected, ReasonCode::kInvalidSnapshot);
  ResourceSnapshot negative_delay = empty_snapshot({worker(0, ms(5))});
  negative_delay.downlink_delay = d(-1);
  ResourceSnapshot negative_lead = empty_snapshot({worker(0, ms(5))});
  negative_lead.dispatch_lead = d(-1);
  ResourceSnapshot short_medium = empty_snapshot({worker(0, ms(5))});
  short_medium.shared_medium = uniform_calendar(60'000, 50);
  ResourceSnapshot mismatched_links = empty_snapshot({worker(0, ms(5))});
  mismatched_links.downlink = uniform_calendar(kDownlinkBytesPerSlot, 50);

  for (const ResourceSnapshot& snapshot :
       {negative_delay, negative_lead, short_medium, mismatched_links}) {
    expect_not_admitted(snapshot, reference_job(), at_ms(0), expected);
  }
}

TEST(SingleChainAdmission, RejectsNegativeResultReadySkew) {
  Job job = reference_job();
  job.maximum_result_ready_skew = d(-1);

  expect_not_admitted(empty_snapshot({worker(0, ms(5))}), job, at_ms(0),
                      outcome(DecisionStatus::kRejected, ReasonCode::kInvalidTiming));
}

TEST(SingleChainAdmission, AdmitsDeliveryExactlyAtEffectiveDeadline) {
  const AdmissionResult result =
      admit(empty_snapshot({worker(0, ms(36))}), reference_job(), at_ms(0));

  ASSERT_EQ(result.decision, admitted());
  EXPECT_EQ(result.reservation.value().download.completion, at_ms(40));
  EXPECT_EQ(result.reservation.value().slack, ms(0));
}

TEST(SingleChainAdmission, RejectsNonPositivePayloadSizes) {
  const ResourceSnapshot snapshot = empty_snapshot({worker(0, ms(5))});
  const Decision expected = outcome(DecisionStatus::kRejected, ReasonCode::kInvalidPayloadSize);
  Job zero_upload = reference_job();
  zero_upload.upload_bytes = Bytes{0};
  Job negative_upload = reference_job();
  negative_upload.upload_bytes = Bytes{-1};
  Job zero_chunk = reference_job();
  zero_chunk.action_chunk_bytes = Bytes{0};
  Job negative_chunk = reference_job();
  negative_chunk.action_chunk_bytes = Bytes{-1};

  for (const Job& job : {zero_upload, negative_upload, zero_chunk, negative_chunk}) {
    SCOPED_TRACE(::testing::Message()
                 << "upload " << job.upload_bytes << ", chunk " << job.action_chunk_bytes);
    expect_not_admitted(snapshot, job, at_ms(0), expected);
  }
}

TEST(SingleChainAdmission, RejectsInconsistentJobTiming) {
  const ResourceSnapshot snapshot = empty_snapshot({worker(0, ms(5))});
  const Decision expected = outcome(DecisionStatus::kRejected, ReasonCode::kInvalidTiming);
  Job negative_margin = reference_job();
  negative_margin.safety_margin = d(-1);
  Job zero_result_age = reference_job();
  zero_result_age.maximum_result_age = ms(0);
  Job negative_result_age = reference_job();
  negative_result_age.maximum_result_age = ms(-1);
  Job observed_after_release = reference_job();
  observed_after_release.observation_time = at_ms(1);
  Job released_after_now = reference_job();
  released_after_now.release_time = at_ms(1);

  {
    SCOPED_TRACE("negative safety margin");
    expect_not_admitted(snapshot, negative_margin, at_ms(0), expected);
  }
  {
    SCOPED_TRACE("zero maximum result age");
    expect_not_admitted(snapshot, zero_result_age, at_ms(0), expected);
  }
  {
    SCOPED_TRACE("negative maximum result age");
    expect_not_admitted(snapshot, negative_result_age, at_ms(0), expected);
  }
  {
    SCOPED_TRACE("observation after release");
    expect_not_admitted(snapshot, observed_after_release, at_ms(1), expected);
  }
  {
    SCOPED_TRACE("release after now");
    expect_not_admitted(snapshot, released_after_now, at_ms(0), expected);
  }
}

TEST(SingleChainAdmission, AcceptsZeroSafetyMarginAndReleaseAtNow) {
  Job job = reference_job();
  job.safety_margin = ms(0);
  job.release_time = at_ms(1);

  const AdmissionResult result = admit(empty_snapshot({worker(0, ms(5))}), job, at_ms(1));

  EXPECT_EQ(result.decision, admitted());
}

TEST(SingleChainAdmission, RejectsArithmeticOverflowInDeadlineOrSlack) {
  const ResourceSnapshot snapshot = empty_snapshot({worker(0, ms(5))});
  Job deadline_underflow = reference_job();
  deadline_underflow.buffer_empty_time = t(kMinNs);
  Job slack_underflow = reference_job();
  slack_underflow.task_deadline = t(kMinNs);
  const Decision expected = outcome(DecisionStatus::kRejected, ReasonCode::kArithmeticOverflow);

  {
    SCOPED_TRACE("effective deadline underflows");
    expect_not_admitted(snapshot, deadline_underflow, at_ms(0), expected);
  }
  {
    SCOPED_TRACE("slack underflows");
    expect_not_admitted(snapshot, slack_underflow, at_ms(0), expected);
  }
  {
    SCOPED_TRACE("dispatch lead overflows");
    ResourceSnapshot unbounded_lead = snapshot;
    unbounded_lead.dispatch_lead = d(kMaxNs);
    expect_not_admitted(unbounded_lead, reference_job(), at_ms(1), expected);
  }
}

TEST(ChainCommit, AppliesEveryReservationToANewSnapshot) {
  const ResourceSnapshot snapshot = empty_snapshot({worker(0, ms(5))});

  const ResourceSnapshot committed = commit(snapshot, reference_chain()).value();

  EXPECT_EQ(reserved_bytes(committed.uplink), expected_reserved({{0, 125'000}, {1, 125'000}}));
  EXPECT_EQ(reserved_bytes(committed.downlink), expected_reserved({{7, 50'000}, {8, 50'000}}));
  EXPECT_EQ(committed.workers.at(0).lane.intervals(), (std::vector<Interval>{window_ms(2, 7)}));
  EXPECT_EQ(snapshot, empty_snapshot({worker(0, ms(5))}));
}

TEST(ChainCommit, RejectsChainConflictingOnAnySingleResource) {
  const ResourceSnapshot with_reference =
      commit(empty_snapshot({worker(0, ms(5))}), reference_chain()).value();
  const ResourceSnapshot busy =
      commit(with_reference, chain_through(0, 30, window_ms(60, 65), 70)).value();

  EXPECT_TRUE(commit(busy, chain_through(0, 20, window_ms(40, 45), 50)).has_value());
  {
    SCOPED_TRACE("uplink slots already reserved");
    EXPECT_EQ(commit(busy, chain_through(0, 0, window_ms(40, 45), 50)), std::nullopt);
  }
  {
    SCOPED_TRACE("downlink slots already reserved");
    EXPECT_EQ(commit(busy, chain_through(0, 20, window_ms(40, 45), 70)), std::nullopt);
  }
  {
    SCOPED_TRACE("compute lane already reserved");
    EXPECT_EQ(commit(busy, chain_through(0, 20, window_ms(60, 65), 80)), std::nullopt);
  }
  {
    SCOPED_TRACE("worker absent from snapshot");
    EXPECT_EQ(commit(busy, chain_through(9, 20, window_ms(40, 45), 50)), std::nullopt);
  }
}

TEST(ChainCommit, RejectsChainWhoseDeliveryDoesNotFollowDownload) {
  ChainReservation early_delivery = reference_chain();
  early_delivery.delivery = at_ms(8);

  EXPECT_EQ(commit(empty_snapshot({worker(0, ms(5))}), early_delivery), std::nullopt);
}

TEST(ChainCommit, RejectsChainExceedingSharedMedium) {
  ResourceSnapshot snapshot = empty_snapshot({worker(0, ms(5)), worker(1, ms(5))});
  snapshot.uplink = uniform_calendar(2 * kUplinkBytesPerSlot);
  snapshot.shared_medium = uniform_calendar(kUplinkBytesPerSlot + kDownlinkBytesPerSlot);
  const ResourceSnapshot committed = commit(snapshot, reference_chain()).value();
  ChainReservation same_slots_other_worker = reference_chain();
  same_slots_other_worker.worker = WorkerId{1};
  same_slots_other_worker.download = transfer_in_slots(20, kDownlinkBytesPerSlot);
  same_slots_other_worker.delivery = at_ms(22);

  EXPECT_EQ(commit(committed, same_slots_other_worker), std::nullopt);
  EXPECT_EQ(reserved_bytes(committed.shared_medium.value()),
            expected_reserved({{0, 125'000}, {1, 125'000}, {7, 50'000}, {8, 50'000}}));
}

TEST(ChainCommit, RejectsChainWhoseStagesAreOutOfOrder) {
  const ResourceSnapshot snapshot = empty_snapshot({worker(0, ms(5))});
  ChainReservation empty_download = reference_chain();
  empty_download.download = Transfer{.slices = {}, .completion = at_ms(9)};

  {
    SCOPED_TRACE("compute starts before upload completes");
    EXPECT_EQ(commit(snapshot, chain_through(0, 0, window_ms(1, 6), 7)), std::nullopt);
  }
  {
    SCOPED_TRACE("download starts before compute ends");
    EXPECT_EQ(commit(snapshot, chain_through(0, 0, window_ms(2, 7), 5)), std::nullopt);
  }
  {
    SCOPED_TRACE("download carries no slices");
    EXPECT_EQ(commit(snapshot, empty_download), std::nullopt);
  }
  {
    SCOPED_TRACE("download starts outside the horizon");
    EXPECT_EQ(commit(snapshot, chain_through(0, 0, window_ms(2, 7), 200)), std::nullopt);
  }
}

TEST(ChainCommit, RejectsChainThatLeavesLessThanTheDispatchLeadBetweenStages) {
  ResourceSnapshot snapshot = empty_snapshot({worker(0, ms(5))});
  snapshot.dispatch_lead = ms(1);

  {
    SCOPED_TRACE("compute starts within the lead after the observation arrives");
    EXPECT_EQ(commit(snapshot, chain_through(0, 0, window_ms(2, 7), 8)), std::nullopt);
  }
  {
    SCOPED_TRACE("download starts within the lead after compute ends");
    EXPECT_EQ(commit(snapshot, chain_through(0, 0, window_ms(3, 8), 8)), std::nullopt);
  }
  EXPECT_TRUE(commit(snapshot, chain_through(0, 0, window_ms(3, 8), 9)).has_value());
}

// Endpoint budgets: a robot's per-slot cap binds the sum of its own requests' service.

std::vector<std::pair<std::int64_t, std::int64_t>> slices_of(const Transfer& transfer) {
  std::vector<std::pair<std::int64_t, std::int64_t>> slices;
  slices.reserve(transfer.slices.size());
  std::ranges::transform(
      transfer.slices, std::back_inserter(slices),
      [](const ServiceSlice& slice) { return std::pair{slice.slot.value(), slice.bytes.value()}; });
  return slices;
}

TEST(EndpointBudget, CapsARobotsServicePerSlotBelowTheSharedLink) {
  ResourceSnapshot snapshot = empty_snapshot({worker(0, ms(5))});
  snapshot.endpoints = {EndpointBudget{.robot = RobotId{0},
                                       .uplink = uniform_calendar(50'000),
                                       .downlink = uniform_calendar(25'000)}};
  const AdmissionResult result = admit(snapshot, reference_job(), at_ms(0));
  ASSERT_EQ(result.decision, admitted());
  const ChainReservation& chain = result.reservation.value();
  EXPECT_EQ(chain.robot, RobotId{0});
  EXPECT_EQ(chain.upload.completion, at_ms(5));
  EXPECT_EQ(chain.compute, window_ms(5, 10));
  EXPECT_EQ(chain.download.completion, at_ms(14));
  EXPECT_EQ(reserved_bytes(result.snapshot.endpoints.front().uplink.value()),
            expected_reserved({{0, 50'000}, {1, 50'000}, {2, 50'000}, {3, 50'000}, {4, 50'000}}));
  EXPECT_EQ(reserved_bytes(result.snapshot.endpoints.front().downlink.value()),
            expected_reserved({{10, 25'000}, {11, 25'000}, {12, 25'000}, {13, 25'000}}));
}

// The robot may take 60,000 bytes per slot of the 125,000-byte link. Its second request gets only
// what its first left in slot 1, while another robot's request uses the link freely.
TEST(EndpointBudget, IsSharedAcrossOneRobotsRequestsButNotOtherRobots) {
  ResourceSnapshot snapshot = empty_snapshot({worker(0, ms(1))});
  snapshot.endpoints = {EndpointBudget{
      .robot = RobotId{0}, .uplink = uniform_calendar(60'000), .downlink = std::nullopt}};
  Job first = job_numbered(1);
  Job second = job_numbered(2);
  Job other = job_numbered(3);
  first.upload_bytes = second.upload_bytes = other.upload_bytes = Bytes{100'000};
  other.robot = RobotId{1};
  AdmissionResult result = admit(snapshot, first, at_ms(0));
  ASSERT_EQ(result.decision, admitted());
  EXPECT_EQ(slices_of(result.reservation->upload),
            (std::vector<std::pair<std::int64_t, std::int64_t>>{{0, 60'000}, {1, 40'000}}));
  result = admit(result.snapshot, second, at_ms(0));
  ASSERT_EQ(result.decision, admitted());
  EXPECT_EQ(
      slices_of(result.reservation->upload),
      (std::vector<std::pair<std::int64_t, std::int64_t>>{{1, 20'000}, {2, 60'000}, {3, 20'000}}));
  result = admit(result.snapshot, other, at_ms(0));
  ASSERT_EQ(result.decision, admitted());
  EXPECT_EQ(slices_of(result.reservation->upload),
            (std::vector<std::pair<std::int64_t, std::int64_t>>{{0, 65'000}, {1, 35'000}}));
}

TEST(EndpointBudget, RejectsBudgetsThatDoNotMatchTheLinksOrRepeatARobot) {
  ResourceSnapshot snapshot = empty_snapshot({worker(0, ms(5))});
  snapshot.endpoints = {EndpointBudget{
      .robot = RobotId{0},
      .uplink = CapacityCalendar::create(at_ms(0), ms(2), std::vector<Bytes>(10, Bytes{1'000})),
      .downlink = std::nullopt}};
  EXPECT_FALSE(is_consistent(snapshot));
  snapshot.endpoints = {
      EndpointBudget{
          .robot = RobotId{0}, .uplink = uniform_calendar(1'000), .downlink = std::nullopt},
      EndpointBudget{.robot = RobotId{0}, .uplink = std::nullopt, .downlink = std::nullopt}};
  EXPECT_FALSE(is_consistent(snapshot));
}

}  // namespace
}  // namespace rovi::core
