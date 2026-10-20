#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "rovi_core/cohort.hpp"
#include "rovi_core/comparison.hpp"
#include "rovi_core/engine.hpp"
#include "rovi_core/interval_calendar.hpp"
#include "rovi_core/receipt.hpp"
#include "rovi_core/units.hpp"
#include "rovi_core/wireless_profile.hpp"

namespace rovi::core {
namespace {

constexpr std::int64_t kMillisecond = 1'000'000;

TimePoint at_ms(std::int64_t ms) { return TimePoint::from_ns(ms * kMillisecond); }
constexpr Duration ms(std::int64_t value) { return Duration::from_ns(value * kMillisecond); }

ServiceSegment rate(std::int64_t first_slot, std::int64_t bps) {
  return ServiceSegment{
      .first_slot = first_slot, .rate = BitsPerSecond{bps}, .reduction = Permille{0}};
}

// 10,000 bytes per 1 ms slot in each direction, one 10 ms lane, no fixed delays.
Environment environment() {
  return Environment{.slot_length = Duration::from_ns(kMillisecond),
                     .slot_count = 100,
                     .uplink = {rate(0, 80'000'000)},
                     .downlink = {rate(0, 80'000'000)},
                     .uplink_delay = {},
                     .downlink_delay = {},
                     .compute_planned = Duration::from_ns(10 * kMillisecond),
                     .compute_actual = Duration::from_ns(10 * kMillisecond)};
}

Request request(std::uint64_t id, std::uint32_t robot, std::int64_t release_ms,
                std::int64_t deadline_ms, std::int64_t upload = 100'000) {
  return Request{.id = JobId{id},
                 .robot = RobotId{robot},
                 .release = at_ms(release_ms),
                 .deadline = at_ms(deadline_ms),
                 .upload = Bytes{upload},
                 .response = Bytes{10'000}};
}

RunResult run(ComparisonPolicy policy, const std::vector<Request>& cohort,
              const RecoveryRules& recovery, const Environment& setting = environment()) {
  std::optional<RunResult> result = run_comparison(policy, setting, cohort, recovery);
  EXPECT_TRUE(result.has_value());
  return result ? std::move(*result) : RunResult{};
}

constexpr RecoveryRules kStrict{};
constexpr RecoveryRules kBestEffort{.best_effort = true, .compute_bound = ms(10)};

// A's reservation covers 0 to 10 ms but A only sends at 5 ms; B's
// covers 10 to 20 ms. A takes the 5 to 10 ms its lost reservation left unused, waits through B's,
// and finishes at 25 ms, while B completes exactly as planned at 20 ms.
TEST(Recovery, ALateUploadUsesOnlyServiceTheCommitmentsLeaveUnused) {
  Request late = request(0, 0, 0, 60);
  late.send_delay = Duration::from_ns(5 * kMillisecond);
  const std::vector<Request> cohort{late, request(1, 1, 0, 60)};

  const RunResult strict = run(ComparisonPolicy::kLinkedLeastSlack, cohort, kStrict);
  EXPECT_TRUE(strict.outcomes.at(0).lease_missed);
  EXPECT_EQ(strict.outcomes.at(0).progress.upload_left, Bytes{100'000});
  EXPECT_EQ(strict.outcomes.at(0).outcome, ReceiptOutcome::kMissing);

  const RunResult recovered = run(ComparisonPolicy::kLinkedLeastSlack, cohort, kBestEffort);
  EXPECT_TRUE(recovered.outcomes.at(0).best_effort);
  EXPECT_EQ(recovered.outcomes.at(0).progress.ready, at_ms(25));
  EXPECT_EQ(recovered.outcomes.at(1).progress.ready, at_ms(20));
  EXPECT_EQ(recovered.outcomes.at(1).progress.receipt, strict.outcomes.at(1).progress.receipt);
  ASSERT_TRUE(recovered.outcomes.at(1).plan.has_value());
  EXPECT_EQ(recovered.outcomes.at(1).progress.receipt, recovered.outcomes.at(1).plan->delivery);
  EXPECT_EQ(recovered.outcomes.at(0).progress.compute_start, at_ms(30));
  EXPECT_EQ(recovered.outcomes.at(0).progress.receipt, at_ms(41));
  EXPECT_EQ(recovered.outcomes.at(0).outcome, ReceiptOutcome::kOnTime);
  EXPECT_EQ(count_recovery(recovered), (RecoveryCounts{.entered = 1, .delivered = 1}));
  EXPECT_EQ(pair_runs(strict, recovered),
            (Pairing{.offered = 2, .recovered = 1, .harmed = 0, .unresolved = 0}));
}

// Sent at 5 ms into an uplink that is out from 10 to 20 ms, A gets
// 50,000 bytes before the outage and the rest after it, and completes its upload at 25 ms.
TEST(Recovery, ALateUploadWaitsOutAnOutageInsteadOfShiftingThroughIt) {
  Environment outage = environment();
  outage.uplink = {rate(0, 80'000'000), rate(10, 0), rate(20, 80'000'000)};
  Request late = request(0, 0, 0, 60);
  late.send_delay = Duration::from_ns(5 * kMillisecond);
  const RunResult recovered = run(ComparisonPolicy::kLinkedLeastSlack, {late}, kBestEffort, outage);
  EXPECT_TRUE(recovered.outcomes.at(0).lease_missed);
  EXPECT_EQ(recovered.outcomes.at(0).progress.ready, at_ms(25));
}

// Each robot may send 5,000 bytes per slot. B's capped upload ends at
// 10 ms, so its compute is protected from 10 to 20 ms. A cannot meet its 5 ms deadline and is
// served best effort; its observation is ready at 2 ms, but a 10 ms compute then would run into
// B's interval, so it waits for the lane gap at 20 ms and B runs exactly as planned.
TEST(Recovery, BestEffortComputeNeverTakesAProtectedFutureInterval) {
  Environment capped = environment();
  capped.endpoint_uplink = Bytes{5'000};
  const std::vector<Request> cohort{request(0, 0, 0, 5, 10'000), request(1, 1, 0, 60, 50'000)};
  const RunResult result = run(ComparisonPolicy::kLinkedLeastSlack, cohort, kBestEffort, capped);
  EXPECT_EQ(result.outcomes.at(0).commitment, Commitment::kRejected);
  EXPECT_EQ(result.outcomes.at(0).progress.ready, at_ms(2));
  EXPECT_EQ(result.outcomes.at(0).progress.compute_start, at_ms(20));
  EXPECT_EQ(result.outcomes.at(0).outcome, ReceiptOutcome::kLate);
  EXPECT_EQ(result.outcomes.at(1).progress.compute_start, at_ms(10));
  ASSERT_TRUE(result.outcomes.at(1).plan.has_value());
  EXPECT_EQ(result.outcomes.at(1).progress.receipt, result.outcomes.at(1).plan->delivery);
}

// A's 5 ms deadline cannot be met, so it is served best effort until robot 0 releases a newer
// observation at 3 ms, which supersedes it; its original deadline still makes it a known miss.
TEST(Recovery, ANewerObservationSupersedesBestEffortWorkWithoutRewritingItsDeadline) {
  const std::vector<Request> cohort{request(0, 0, 0, 5), request(1, 0, 3, 90)};
  const RecoveryRules superseding{.best_effort = true, .supersede = true};
  const RunResult result = run(ComparisonPolicy::kLinkedLeastSlack, cohort, superseding);
  EXPECT_EQ(result.outcomes.at(0).dropped, DropReason::kSuperseded);
  EXPECT_EQ(result.outcomes.at(0).progress.dropped, at_ms(3));
  EXPECT_EQ(result.outcomes.at(0).progress.upload_left, Bytes{70'000});
  EXPECT_EQ(result.outcomes.at(0).outcome, ReceiptOutcome::kMissing);
  EXPECT_EQ(result.cohort.at(0).deadline, at_ms(5));
  EXPECT_EQ(count_recovery(result), (RecoveryCounts{.entered = 1, .superseded = 1}));

  const RunResult kept = run(ComparisonPolicy::kLinkedLeastSlack, cohort, kBestEffort);
  EXPECT_FALSE(kept.outcomes.at(0).dropped.has_value());
  EXPECT_EQ(kept.outcomes.at(0).outcome, ReceiptOutcome::kLate);
}

TEST(Recovery, BestEffortWorkExpiresAfterItsMaximumAge) {
  const RecoveryRules aging{.best_effort = true,
                            .maximum_age = Duration::from_ns(4 * kMillisecond)};
  const RunResult result = run(ComparisonPolicy::kLinkedLeastSlack, {request(0, 0, 0, 5)}, aging);
  EXPECT_EQ(result.outcomes.at(0).dropped, DropReason::kExpired);
  EXPECT_EQ(result.outcomes.at(0).progress.dropped, at_ms(5));
  EXPECT_EQ(result.outcomes.at(0).progress.upload_left, Bytes{50'000});
}

TEST(Recovery, AFullBacklogRefusesFurtherBestEffortWork) {
  const RecoveryRules bounded{.best_effort = true, .compute_bound = ms(10), .backlog = 1};
  const RunResult result =
      run(ComparisonPolicy::kLinkedLeastSlack, {request(0, 0, 0, 5), request(1, 1, 0, 6)}, bounded);
  EXPECT_FALSE(result.outcomes.at(0).dropped.has_value());
  EXPECT_EQ(result.outcomes.at(1).dropped, DropReason::kBacklogFull);
  EXPECT_EQ(result.outcomes.at(1).progress.upload_left, Bytes{100'000});
  EXPECT_EQ(count_recovery(result),
            (RecoveryCounts{.entered = 2, .delivered = 1, .backlog_full = 1}));
}

// The declared rules are common: the independent control drops superseded work too.
TEST(Recovery, TheIndependentControlFollowsTheSameDeclaredRules) {
  const std::vector<Request> cohort{request(0, 0, 0, 5), request(1, 0, 3, 90)};
  const RecoveryRules superseding{.best_effort = true, .supersede = true};
  const RunResult result = run(ComparisonPolicy::kIndependentEarliestDeadline, cohort, superseding);
  EXPECT_EQ(result.outcomes.at(0).dropped, DropReason::kSuperseded);
  EXPECT_EQ(result.recovery, superseding);
}

// A is ready at 2 ms while C computes from 1 to 11 ms; robot 0's newer request at 5 ms supersedes A
// as it waits, so the lane must skip it once free.
TEST(Recovery, IndependentComputeSkipsWorkDroppedWhileItWaitsForTheLane) {
  const std::vector<Request> cohort{request(0, 0, 0, 30, 10'000), request(1, 1, 0, 20, 10'000),
                                    request(2, 0, 5, 90)};
  const RecoveryRules superseding{.supersede = true};
  const RunResult result = run(ComparisonPolicy::kIndependentEarliestDeadline, cohort, superseding);
  EXPECT_EQ(result.outcomes.at(1).progress.compute_start, at_ms(1));
  EXPECT_EQ(result.outcomes.at(0).progress.ready, at_ms(2));
  EXPECT_EQ(result.outcomes.at(0).dropped, DropReason::kSuperseded);
  EXPECT_FALSE(result.outcomes.at(0).progress.compute_start.has_value());
}

// Both directions share 10,000 bytes per slot. B uploads in slot 0, computes from 1 to 11 ms, and
// downloads in slot 11. A is served best effort; it must leave slot 11's medium to B's download,
// so it gets 100,000 bytes in slots 1 to 10 and the rest in slots 12 to 21.
TEST(Recovery, ABestEffortUploadLeavesTheMediumACommittedDownloadNeeds) {
  Environment coupled = environment();
  coupled.medium = std::vector<ServiceSegment>{rate(0, 80'000'000)};
  const std::vector<Request> cohort{request(0, 0, 0, 5, 200'000), request(1, 1, 0, 60, 10'000)};
  const RunResult result = run(ComparisonPolicy::kLinkedLeastSlack, cohort, kBestEffort, coupled);
  ASSERT_TRUE(result.outcomes.at(1).plan.has_value());
  EXPECT_EQ(result.outcomes.at(1).progress.receipt, at_ms(12));
  EXPECT_EQ(result.outcomes.at(1).progress.receipt, result.outcomes.at(1).plan->delivery);
  EXPECT_TRUE(result.outcomes.at(0).best_effort);
  EXPECT_EQ(result.outcomes.at(0).progress.ready, at_ms(22));
}

// A's reservation uploads from 5 ms, but A only sends at 10 ms; robot 0's newer observation was
// released at 3 ms. A is dropped as superseded the moment it loses its lease, not a slot later.
TEST(Recovery, WorkThatLosesItsLeaseAfterANewerObservationIsSupersededAtOnce) {
  Request late = request(0, 0, 0, 90);
  late.send_delay = Duration::from_ns(10 * kMillisecond);
  const std::vector<Request> cohort{late, request(1, 1, 0, 20, 50'000), request(2, 0, 3, 90)};
  const RecoveryRules superseding{.best_effort = true, .supersede = true};
  const RunResult result = run(ComparisonPolicy::kLinkedLeastSlack, cohort, superseding);
  EXPECT_EQ(result.outcomes.at(0).commitment, Commitment::kAdmitted);
  EXPECT_TRUE(result.outcomes.at(0).lease_missed);
  EXPECT_EQ(result.outcomes.at(0).dropped, DropReason::kSuperseded);
  EXPECT_EQ(result.outcomes.at(0).progress.dropped, at_ms(5));
  EXPECT_EQ(result.outcomes.at(0).progress.upload_left, Bytes{100'000});
  EXPECT_EQ(count_recovery(result), (RecoveryCounts{.entered = 1, .superseded = 1}));
}

// B's reservation computes from 30 to 40 ms. A is ready at 20 ms, but its
// declared 20 ms bound would run into B's interval, so it starts at 40 ms.
TEST(Recovery, ARecoveryBoundedAt20MsWaitsOutTheProtectedInterval) {
  Environment capped = environment();
  capped.endpoint_uplink = Bytes{5'000};
  const std::vector<Request> cohort{request(1, 1, 0, 60, 150'000), request(0, 0, 19, 20, 5'000)};
  const RecoveryRules bounded{.best_effort = true, .compute_bound = ms(20)};
  const RunResult result = run(ComparisonPolicy::kLinkedLeastSlack, cohort, bounded, capped);
  ASSERT_TRUE(result.outcomes.at(0).plan.has_value());
  EXPECT_EQ(result.outcomes.at(0).plan->compute, (Interval{.start = at_ms(30), .end = at_ms(40)}));
  EXPECT_EQ(result.outcomes.at(0).progress.receipt, result.outcomes.at(0).plan->delivery);
  EXPECT_EQ(result.outcomes.at(1).progress.ready, at_ms(20));
  EXPECT_EQ(result.outcomes.at(1).progress.compute_start, at_ms(40));
  EXPECT_TRUE(result.outcomes.at(1).unreserved_compute);
}

TEST(Recovery, WithoutADeclaredBoundRecoveryComputeIsDeferred) {
  Environment capped = environment();
  capped.endpoint_uplink = Bytes{5'000};
  const std::vector<Request> cohort{request(1, 1, 0, 60, 150'000), request(0, 0, 19, 20, 5'000)};
  const RunResult result =
      run(ComparisonPolicy::kLinkedLeastSlack, cohort, {.best_effort = true}, capped);
  EXPECT_EQ(result.outcomes.at(1).progress.ready, at_ms(20));
  EXPECT_FALSE(result.outcomes.at(1).progress.compute_start.has_value());
  EXPECT_FALSE(result.outcomes.at(1).unreserved_compute);
}

TEST(Recovery, RefusesABoundShorterThanThePlannedCompute) {
  EXPECT_FALSE(run_comparison(ComparisonPolicy::kLinkedLeastSlack, environment(),
                              {request(0, 0, 0, 60)}, {.best_effort = true, .compute_bound = ms(9)})
                   .has_value());
}

// Planned 8 ms, actual and bound 9 ms. B's reservation computes from 10 to 18 ms and runs to 19 ms.
// A is ready at 2 ms, but 9 ms from then reaches B's interval, so A waits for the lane at 19 ms.
TEST(Recovery, RecoveryComputeIsFittedWithTheDeclaredBoundNotThePlannedTime) {
  Environment longer = environment();
  longer.compute_planned = ms(8);
  longer.compute_actual = ms(9);
  longer.endpoint_uplink = Bytes{5'000};
  const std::vector<Request> cohort{request(0, 0, 0, 1, 10'000), request(1, 1, 0, 60, 50'000)};
  const RecoveryRules bounded{.best_effort = true, .compute_bound = ms(9)};
  const RunResult result = run(ComparisonPolicy::kLinkedLeastSlack, cohort, bounded, longer);
  ASSERT_TRUE(result.outcomes.at(1).plan.has_value());
  EXPECT_EQ(result.outcomes.at(1).plan->compute, (Interval{.start = at_ms(10), .end = at_ms(18)}));
  EXPECT_EQ(result.outcomes.at(0).progress.compute_start, at_ms(19));
}

// Planned 9 ms, actual and bound 9.3 ms. Strict delivers B at its 22 ms deadline. A is ready at
// 2.5 ms, and fitting 9 ms there would push B's compute late; with the bound, A waits for B.
TEST(Recovery, RecoveryNeverTurnsAStrictOnTimeReservationLate) {
  Environment longer = environment();
  longer.compute_planned = ms(9);
  longer.compute_actual = Duration::from_ns(9'300'000);
  longer.uplink_delay = Duration::from_ns(500'000);
  longer.endpoint_uplink = Bytes{5'000};
  const std::vector<Request> cohort{request(0, 0, 0, 1, 10'000), request(1, 1, 0, 22, 55'000)};
  const RecoveryRules bounded{.best_effort = true, .compute_bound = longer.compute_actual};
  const RunResult strict = run(ComparisonPolicy::kLinkedLeastSlack, cohort, kStrict, longer);
  const RunResult recovered = run(ComparisonPolicy::kLinkedLeastSlack, cohort, bounded, longer);
  EXPECT_EQ(strict.outcomes.at(1).progress.receipt, at_ms(22));
  EXPECT_EQ(recovered.outcomes.at(1).progress.receipt, at_ms(22));
  EXPECT_EQ(recovered.outcomes.at(0).progress.compute_start, TimePoint::from_ns(20'800'000));
  const std::optional<Pairing> pairing = pair_runs(strict, recovered);
  ASSERT_TRUE(pairing.has_value());
  EXPECT_EQ(pairing->harmed, 0U);
}

// A sends at 5 ms and is ready at 16 ms; B is ready at 13 ms. A's earlier deadline must not hold
// the free lane for an input that has not arrived.
TEST(Recovery, AReadyRecoveryIsNotHeldBehindOneWhoseInputArrivesLater) {
  Environment delayed = environment();
  delayed.uplink_delay = ms(10);
  Request late = request(0, 0, 0, 1, 10'000);
  late.send_delay = ms(5);
  const std::vector<Request> cohort{late, request(1, 1, 2, 4, 10'000)};
  const RunResult result = run(ComparisonPolicy::kLinkedLeastSlack, cohort, kBestEffort, delayed);
  EXPECT_EQ(result.outcomes.at(0).progress.ready, at_ms(16));
  EXPECT_EQ(result.outcomes.at(1).progress.ready, at_ms(13));
  EXPECT_EQ(result.outcomes.at(1).progress.compute_start, at_ms(13));
  EXPECT_EQ(result.outcomes.at(0).progress.compute_start, at_ms(23));
}

// The same, with A expiring at 15 ms while it waits: the lane must not then be offered at 13 ms.
TEST(Recovery, ExpiringWorkThatWaitsForItsInputCannotAbortTheRun) {
  Environment delayed = environment();
  delayed.uplink_delay = ms(10);
  Request late = request(0, 0, 0, 1, 10'000);
  late.send_delay = ms(5);
  const std::vector<Request> cohort{late, request(1, 1, 2, 4, 10'000)};
  const RecoveryRules aging{.best_effort = true, .compute_bound = ms(10), .maximum_age = ms(14)};
  const RunResult result = run(ComparisonPolicy::kLinkedLeastSlack, cohort, aging, delayed);
  EXPECT_EQ(result.outcomes.at(0).dropped, DropReason::kExpired);
  EXPECT_EQ(result.outcomes.at(0).progress.dropped, at_ms(15));
  EXPECT_EQ(result.outcomes.at(1).progress.compute_start, at_ms(13));
}

TEST(Recovery, StrictExecutionRemainsTheDefaultControl) {
  const std::vector<Request> cohort{request(0, 0, 0, 5), request(1, 1, 0, 60)};
  const RunResult defaulted = run(ComparisonPolicy::kLinkedLeastSlack, cohort, RecoveryRules{});
  EXPECT_EQ(defaulted.recovery, kStrict);
  EXPECT_FALSE(defaulted.outcomes.at(0).best_effort);
  EXPECT_EQ(defaulted.outcomes.at(0).progress.upload_left, Bytes{100'000});
  EXPECT_EQ(count_recovery(defaulted), RecoveryCounts{});
}

}  // namespace
}  // namespace rovi::core
