#include "rovi_core/scheduler.hpp"

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "rovi_core/admission.hpp"
#include "rovi_core/audit.hpp"
#include "rovi_core/capacity_calendar.hpp"
#include "rovi_core/interval_calendar.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/units.hpp"
#include "rovi_core/wireless_profile.hpp"

#include "printers.hpp"  // IWYU pragma: keep
#include "scenario.hpp"
#include "test_support.hpp"

namespace rovi::core {
namespace {

const JobDecision& decision_for(const ScheduleResult& result, std::uint64_t id) {
  const auto found = std::ranges::find(result.decisions, JobId{id}, &JobDecision::job);
  if (found == result.decisions.end()) {
    throw std::out_of_range{"no decision for job"};
  }
  return *found;
}

Job member(std::uint64_t id, std::uint64_t worker_id, Duration skew) {
  Job job = job_numbered(id);
  job.eligible_workers = {WorkerId{worker_id}};
  job.synchronization_group = GroupId{1};
  job.maximum_result_ready_skew = skew;
  return job;
}

ScheduleResult run(const ResourceSnapshot& snapshot, const std::vector<Job>& jobs,
                   const std::vector<JobId>& completed = {}, Policy policy = Policy::kLeastSlack) {
  return schedule(snapshot, jobs, completed, at_ms(0), policy);
}

TEST(Scheduler, AdmitsIndependentJobsWithoutComputeOverlapOrOverbooking) {
  const ResourceSnapshot snapshot = empty_snapshot({worker(0, ms(5))});
  const std::vector<Job> jobs = {job_numbered(1), job_numbered(2), job_numbered(3)};

  const ScheduleResult result = run(snapshot, jobs);

  ASSERT_EQ(result.decisions.size(), 3U);
  for (const JobDecision& decision : result.decisions) {
    EXPECT_EQ(decision.decision, admitted());
  }
  EXPECT_EQ(result.snapshot.workers.front().lane.intervals(),
            (std::vector<Interval>{window_ms(2, 7), window_ms(7, 12), window_ms(12, 17)}));
  EXPECT_EQ(
      reserved_bytes(result.snapshot.uplink),
      expected_reserved(
          {{0, 125'000}, {1, 125'000}, {2, 125'000}, {3, 125'000}, {4, 125'000}, {5, 125'000}}));
  EXPECT_EQ(
      reserved_bytes(result.snapshot.downlink),
      expected_reserved(
          {{7, 50'000}, {8, 50'000}, {12, 50'000}, {13, 50'000}, {17, 50'000}, {18, 50'000}}));
  EXPECT_TRUE(audit_schedule(snapshot, jobs, {}, result).empty());
}

TEST(Scheduler, LeastSlackAdmitsTightJobThatArrivalOrderRejects) {
  const ResourceSnapshot snapshot =
      empty_snapshot({worker_with(0, {{.model = ModelClass{0}, .duration = ms(5)},
                                      {.model = ModelClass{1}, .duration = ms(20)}})});
  Job loose_long = job_numbered(1);
  loose_long.buffer_empty_time = at_ms(100);
  loose_long.maximum_result_age = ms(100);
  loose_long.model = ModelClass{1};
  Job tight_short = job_numbered(2);
  tight_short.buffer_empty_time = at_ms(30);
  tight_short.maximum_result_age = ms(100);
  const std::vector<Job> jobs = {loose_long, tight_short};

  const ScheduleResult least_slack = run(snapshot, jobs, {}, Policy::kLeastSlack);
  const ScheduleResult earliest_deadline = run(snapshot, jobs, {}, Policy::kEarliestDeadline);
  const ScheduleResult arrival_order = run(snapshot, jobs, {}, Policy::kFirstInFirstOut);

  EXPECT_EQ(decision_for(least_slack, 1).decision, admitted());
  EXPECT_EQ(decision_for(least_slack, 2).decision, admitted());
  EXPECT_EQ(decision_for(least_slack, 2).reservation.value().compute, window_ms(2, 7));
  EXPECT_EQ(decision_for(earliest_deadline, 2).decision, admitted());
  EXPECT_EQ(decision_for(arrival_order, 1).decision, admitted());
  EXPECT_EQ(decision_for(arrival_order, 2).decision,
            outcome(DecisionStatus::kRejected, ReasonCode::kDeadlineMiss));
  for (const ScheduleResult* result : {&least_slack, &earliest_deadline, &arrival_order}) {
    EXPECT_TRUE(audit_schedule(snapshot, jobs, {}, *result).empty());
  }
}

TEST(Scheduler, BreaksEqualSlackTiesBySequenceIndependentOfInputOrder) {
  const ResourceSnapshot snapshot = empty_snapshot({worker(0, ms(5))});

  const ScheduleResult reversed = run(snapshot, {job_numbered(2), job_numbered(1)});
  const ScheduleResult ordered = run(snapshot, {job_numbered(1), job_numbered(2)});

  EXPECT_EQ(decision_for(reversed, 1).reservation.value().compute, window_ms(2, 7));
  EXPECT_EQ(decision_for(reversed, 2).reservation.value().compute, window_ms(7, 12));
  EXPECT_EQ(reversed.decisions, ordered.decisions);
  EXPECT_EQ(reversed.snapshot, ordered.snapshot);
  EXPECT_EQ(reversed.digest, ordered.digest);
}

TEST(Scheduler, DefersJobsUntilEveryPredecessorHasActuallyCompleted) {
  const ResourceSnapshot snapshot = empty_snapshot({worker(0, ms(5))});
  Job dependent = job_numbered(2);
  dependent.predecessors = {JobId{1}};

  const ScheduleResult same_cycle = run(snapshot, {job_numbered(1), dependent});
  const ScheduleResult after_completion = run(snapshot, {dependent}, {JobId{1}});

  EXPECT_EQ(decision_for(same_cycle, 1).decision, admitted());
  EXPECT_EQ(decision_for(same_cycle, 2).decision,
            outcome(DecisionStatus::kDeferred, ReasonCode::kPredecessorIncomplete));
  EXPECT_EQ(decision_for(after_completion, 2).decision, admitted());
}

TEST(Scheduler, RejectsEveryJobSharingADuplicatedId) {
  const ResourceSnapshot snapshot = empty_snapshot({worker(0, ms(5))});

  const ScheduleResult result = run(snapshot, {job_numbered(7), job_numbered(7), job_numbered(8)});

  ASSERT_EQ(result.decisions.size(), 3U);
  EXPECT_EQ(result.decisions.at(0).decision,
            outcome(DecisionStatus::kRejected, ReasonCode::kDuplicateJob));
  EXPECT_EQ(result.decisions.at(1).decision,
            outcome(DecisionStatus::kRejected, ReasonCode::kDuplicateJob));
  EXPECT_EQ(decision_for(result, 8).decision, admitted());
}

TEST(Scheduler, AdmitsCooperativeGroupAtomically) {
  const ResourceSnapshot snapshot =
      empty_snapshot({worker(0, ms(5)), worker(1, ms(5)), worker(2, ms(5))});
  const std::vector<Job> group = {member(1, 0, ms(10)), member(2, 1, ms(10)), member(3, 2, ms(10))};

  const ScheduleResult result = run(snapshot, group);

  for (const JobDecision& decision : result.decisions) {
    EXPECT_EQ(decision.decision, admitted());
  }
  EXPECT_EQ(decision_for(result, 1).reservation.value().delivery, at_ms(9));
  EXPECT_EQ(decision_for(result, 3).reservation.value().delivery, at_ms(13));
  EXPECT_TRUE(audit_schedule(snapshot, group, {}, result).empty());
}

TEST(Scheduler, AdmitsNoMemberOfAGroupWhenAnyMemberCannotBeAdmitted) {
  const ResourceSnapshot snapshot =
      empty_snapshot({worker(0, ms(5)), worker(1, ms(5)), worker(2, ms(60))});
  const std::vector<Job> group = {member(1, 0, ms(100)), member(2, 1, ms(100)),
                                  member(3, 2, ms(100))};

  const ScheduleResult result = run(snapshot, group);

  EXPECT_EQ(decision_for(result, 1).decision,
            outcome(DecisionStatus::kDeferred, ReasonCode::kGroupMemberUnavailable));
  EXPECT_EQ(decision_for(result, 2).decision,
            outcome(DecisionStatus::kDeferred, ReasonCode::kGroupMemberUnavailable));
  EXPECT_EQ(decision_for(result, 3).decision,
            outcome(DecisionStatus::kRejected, ReasonCode::kStaleResult));
  EXPECT_EQ(result.snapshot, snapshot);
}

TEST(Scheduler, DefersWholeGroupWhenAMemberWaitsOnAPredecessor) {
  const ResourceSnapshot snapshot =
      empty_snapshot({worker(0, ms(5)), worker(1, ms(5)), worker(2, ms(5))});
  Job blocked = member(2, 1, ms(100));
  blocked.predecessors = {JobId{99}};

  const ScheduleResult result =
      run(snapshot, {member(1, 0, ms(100)), blocked, member(3, 2, ms(100))});

  EXPECT_EQ(decision_for(result, 1).decision,
            outcome(DecisionStatus::kDeferred, ReasonCode::kGroupMemberUnavailable));
  EXPECT_EQ(decision_for(result, 2).decision,
            outcome(DecisionStatus::kDeferred, ReasonCode::kPredecessorIncomplete));
  EXPECT_EQ(decision_for(result, 3).decision,
            outcome(DecisionStatus::kDeferred, ReasonCode::kGroupMemberUnavailable));
  EXPECT_EQ(result.snapshot, snapshot);
}

TEST(Scheduler, EnforcesBoundedResultReadySkewWithinAGroup) {
  const ResourceSnapshot snapshot = empty_snapshot({worker(0, ms(5)), worker(1, ms(15))});

  const ScheduleResult tight = run(snapshot, {member(1, 0, ms(5)), member(2, 1, ms(5))});
  const ScheduleResult loose = run(snapshot, {member(1, 0, ms(20)), member(2, 1, ms(20))});

  const Decision skew_exceeded = outcome(DecisionStatus::kDeferred, ReasonCode::kGroupSkewExceeded);
  EXPECT_EQ(decision_for(tight, 1).decision, skew_exceeded);
  EXPECT_EQ(decision_for(tight, 2).decision, skew_exceeded);
  EXPECT_EQ(tight.snapshot, snapshot);
  EXPECT_EQ(decision_for(loose, 1).reservation.value().delivery, at_ms(9));
  EXPECT_EQ(decision_for(loose, 2).reservation.value().delivery, at_ms(21));
}

TEST(Scheduler, HandoverOutageDelaysUploadUntilServiceReturns) {
  ResourceSnapshot snapshot = empty_snapshot({worker(0, ms(5))});
  snapshot.uplink =
      build_service_calendar(TimePoint{}, ms(1), kHorizonSlots,
                             {{.first_slot = 0, .rate = BitsPerSecond{0}},
                              {.first_slot = 5, .rate = BitsPerSecond{1'000'000'000}}})
          .value();
  ResourceSnapshot lost = snapshot;
  lost.uplink =
      build_service_calendar(TimePoint{}, ms(1), kHorizonSlots, {{.first_slot = 0}}).value();

  const ScheduleResult recovered = run(snapshot, {job_numbered(1)});
  const ScheduleResult outage = run(lost, {job_numbered(1)});

  EXPECT_EQ(decision_for(recovered, 1).reservation.value().upload,
            transfer_in_slots(5, kUplinkBytesPerSlot));
  EXPECT_EQ(decision_for(recovered, 1).reservation.value().delivery, at_ms(14));
  EXPECT_EQ(decision_for(outage, 1).decision,
            outcome(DecisionStatus::kDeferred, ReasonCode::kUplinkServiceUnavailable));
}

TEST(Scheduler, SharedMediumCouplesUplinkAndDownlinkAcrossJobs) {
  const ResourceSnapshot independent = empty_snapshot({worker(0, ms(1))});
  ResourceSnapshot coupled = independent;
  coupled.shared_medium = uniform_calendar(kUplinkBytesPerSlot);
  const std::vector<Job> jobs = {job_numbered(1), job_numbered(2)};

  const ScheduleResult without_medium = run(independent, jobs);
  const ScheduleResult with_medium = run(coupled, jobs);

  EXPECT_EQ(without_medium.snapshot.uplink.reserved(SlotIndex{3}).value() +
                without_medium.snapshot.downlink.reserved(SlotIndex{3}).value(),
            175'000);
  for (std::int64_t slot = 0; slot < kHorizonSlots; ++slot) {
    EXPECT_LE(with_medium.snapshot.uplink.reserved(SlotIndex{slot}).value() +
                  with_medium.snapshot.downlink.reserved(SlotIndex{slot}).value(),
              kUplinkBytesPerSlot);
  }
  EXPECT_EQ(decision_for(with_medium, 2).reservation.value().upload.completion, at_ms(5));
  EXPECT_TRUE(audit_schedule(coupled, jobs, {}, with_medium).empty());
}

TEST(Scheduler, RejectsEveryJobForAnInconsistentSnapshot) {
  ResourceSnapshot snapshot = empty_snapshot({worker(0, ms(5))});
  snapshot.uplink_delay = d(-1);

  const ScheduleResult result = run(snapshot, {job_numbered(1), job_numbered(2)});

  for (const JobDecision& decision : result.decisions) {
    EXPECT_EQ(decision.decision, outcome(DecisionStatus::kRejected, ReasonCode::kInvalidSnapshot));
  }
  EXPECT_EQ(result.snapshot, snapshot);
}

TEST(Scheduler, DecisionDigestIsDeterministicAndSensitiveToDecisions) {
  const ResourceSnapshot snapshot = empty_snapshot({worker(0, ms(5))});
  const std::vector<Job> jobs = {job_numbered(1), job_numbered(2)};
  std::vector<Job> heavier = jobs;
  heavier.at(1).upload_bytes = Bytes{260'000};

  const ScheduleResult first = run(snapshot, jobs);
  const ScheduleResult second = run(snapshot, jobs);
  const ScheduleResult changed = run(snapshot, heavier);

  EXPECT_EQ(first.digest, second.digest);
  EXPECT_EQ(first.digest, decision_digest(first.decisions));
  EXPECT_NE(first.digest, changed.digest);
}

TEST(Scheduler, ReportsTheServiceSourceOfTheSnapshot) {
  ResourceSnapshot enforced = empty_snapshot({worker(0, ms(5))});
  enforced.source = ServiceSource::kEnforced;

  EXPECT_EQ(run(empty_snapshot({worker(0, ms(5))}), {job_numbered(1)}).source,
            ServiceSource::kLogical);
  EXPECT_EQ(run(enforced, {job_numbered(1)}).source, ServiceSource::kEnforced);
}

}  // namespace
}  // namespace rovi::core
