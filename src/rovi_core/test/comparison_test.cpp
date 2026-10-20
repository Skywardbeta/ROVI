#include "rovi_core/comparison.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <map>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "rovi_core/admission.hpp"
#include "rovi_core/capacity_calendar.hpp"
#include "rovi_core/cohort.hpp"
#include "rovi_core/engine.hpp"
#include "rovi_core/interval_calendar.hpp"
#include "rovi_core/receipt.hpp"
#include "rovi_core/units.hpp"
#include "rovi_core/wireless_profile.hpp"

namespace rovi::core {
namespace {

constexpr std::int64_t kMillisecond = 1'000'000;
constexpr std::int64_t kSlotBytes = 10'000;

TimePoint at_ms(std::int64_t ms) { return TimePoint::from_ns(ms * kMillisecond); }
Duration ms(std::int64_t value) { return Duration::from_ns(value * kMillisecond); }

// 80 Mbps over 1 ms slots is 10,000 bytes per slot: a 100,000-byte upload takes 10 slots and a
// 10,000-byte response one slot, around one 10 ms compute lane, with no fixed delays.
Environment environment(std::int64_t slots = 100) {
  const std::vector<ServiceSegment> link{
      {.first_slot = 0, .rate = BitsPerSecond{80'000'000}, .reduction = Permille{0}}};
  return Environment{.slot_length = ms(1),
                     .slot_count = slots,
                     .uplink = link,
                     .downlink = link,
                     .uplink_delay = {},
                     .downlink_delay = {},
                     .compute_planned = ms(10),
                     .compute_actual = ms(10)};
}

Request request(std::uint32_t robot, std::int64_t release_ms, std::int64_t deadline_ms) {
  return Request{.id = JobId{std::uint64_t{robot} << 32U},
                 .robot = RobotId{robot},
                 .release = at_ms(release_ms),
                 .deadline = at_ms(deadline_ms),
                 .upload = Bytes{100'000},
                 .response = Bytes{10'000}};
}

RunResult run(ComparisonPolicy policy, const std::vector<Request>& cohort,
              const Environment& setting = environment()) {
  std::optional<RunResult> result = run_comparison(policy, setting, cohort);
  EXPECT_TRUE(result.has_value()) << name_of(policy);
  return result ? std::move(*result) : RunResult{};
}

std::vector<std::optional<TimePoint>> receipts(const RunResult& result) {
  std::vector<std::optional<TimePoint>> times;
  times.reserve(result.outcomes.size());
  std::ranges::transform(result.outcomes, std::back_inserter(times),
                         [](const RequestOutcome& outcome) { return outcome.progress.receipt; });
  return times;
}

std::vector<Event> before(const std::vector<Event>& events, TimePoint cutoff) {
  std::vector<Event> kept;
  std::ranges::copy_if(events, std::back_inserter(kept),
                       [cutoff](const Event& event) { return event.time < cutoff; });
  return kept;
}

// Rebuilds every stage from the raw trace of the test environment, independently of the engine.
void expect_resource_laws(const RunResult& result) {
  const std::size_t count = result.cohort.size();
  std::vector<std::int64_t> uploaded(count, 0);
  std::vector<std::int64_t> responded(count, 0);
  std::vector<std::optional<TimePoint>> ready(count);
  std::vector<std::optional<TimePoint>> produced(count);
  std::vector<std::optional<TimePoint>> received(count);
  std::vector<bool> dropped(count, false);
  std::map<std::pair<EventKind, std::int64_t>, std::int64_t> per_slot;
  std::optional<TimePoint> lane_free;
  for (const Event& event : result.events) {
    ASSERT_LT(event.request, count);
    const Request& offered = result.cohort.at(event.request);
    const std::int64_t slot = event.time.ns() / kMillisecond;
    EXPECT_TRUE(!dropped.at(event.request) || event.kind == EventKind::kComputeEnd)
        << "a dropped request was served";
    switch (event.kind) {
      case EventKind::kUploadService:
        EXPECT_EQ(event.time.ns() % kMillisecond, 0);
        EXPECT_GE(event.time, offered.release);
        EXPECT_FALSE(ready.at(event.request).has_value());
        uploaded.at(event.request) += event.bytes.value();
        EXPECT_LE(uploaded.at(event.request), offered.upload.value());
        per_slot[{event.kind, slot}] += event.bytes.value();
        EXPECT_LE((per_slot[{event.kind, slot}]), kSlotBytes);
        break;
      case EventKind::kUploadComplete:
        EXPECT_EQ(uploaded.at(event.request), offered.upload.value());
        ready.at(event.request) = event.time;
        break;
      case EventKind::kComputeStart:
        ASSERT_TRUE(ready.at(event.request).has_value());
        EXPECT_GE(event.time, *ready.at(event.request));
        EXPECT_TRUE(!lane_free || event.time >= *lane_free);
        lane_free = TimePoint::from_ns(event.time.ns() + (10 * kMillisecond));
        break;
      case EventKind::kComputeEnd:
        EXPECT_EQ(lane_free, event.time);
        produced.at(event.request) = event.time;
        break;
      case EventKind::kResponseService:
        ASSERT_TRUE(produced.at(event.request).has_value());
        EXPECT_EQ(event.time.ns() % kMillisecond, 0);
        EXPECT_GE(event.time, *produced.at(event.request));
        EXPECT_FALSE(received.at(event.request).has_value());
        responded.at(event.request) += event.bytes.value();
        EXPECT_LE(responded.at(event.request), offered.response.value());
        per_slot[{event.kind, slot}] += event.bytes.value();
        EXPECT_LE((per_slot[{event.kind, slot}]), kSlotBytes);
        break;
      case EventKind::kReceipt:
        EXPECT_EQ(responded.at(event.request), offered.response.value());
        received.at(event.request) = event.time;
        break;
      case EventKind::kDropped:
        EXPECT_FALSE(received.at(event.request).has_value());
        dropped.at(event.request) = true;
        break;
    }
  }
  for (std::size_t index = 0; index < count; ++index) {
    EXPECT_EQ(received.at(index), result.outcomes.at(index).progress.receipt) << index;
  }
}

TEST(Comparison, NamesPoliciesKeepsLegacyAliasesAndDeclaresCapabilities) {
  for (const ComparisonPolicy policy : kComparisonPolicies) {
    EXPECT_EQ(policy_named(name_of(policy)), policy);
  }
  EXPECT_EQ(policy_named("least_slack"), ComparisonPolicy::kLinkedLeastSlack);
  EXPECT_EQ(policy_named("earliest_deadline"), ComparisonPolicy::kLinkedEarliestDeadline);
  EXPECT_EQ(policy_named("first_in_first_out"), ComparisonPolicy::kLinkedFirstInFirstOut);
  EXPECT_FALSE(policy_named("optimal").has_value());
  const Capabilities linked{
      .cross_stage_planning = true, .atomic_admission = true, .capacity_lookahead = true};
  EXPECT_EQ(capabilities_of(ComparisonPolicy::kLinkedLeastSlack), linked);
  EXPECT_EQ(capabilities_of(ComparisonPolicy::kLinkedFirstInFirstOut), linked);
  EXPECT_EQ(capabilities_of(ComparisonPolicy::kIndependentEarliestDeadline), Capabilities{});
  EXPECT_EQ(capabilities_of(ComparisonPolicy::kIndependentEqualShare), Capabilities{});
}

// Four robots release together with 40 ms deadlines. Independent stages serve them in id order and
// receive at 21, 31, 41, and 51 ms. The linked plan admits the first two and rejects the others
// before serving them, because their earliest delivery would be 41 ms.
TEST(Comparison, ABurstOfFourHasHandDerivedLinkedAndIndependentOutcomes) {
  const std::vector<Request> cohort{request(0, 0, 40), request(1, 0, 40), request(2, 0, 40),
                                    request(3, 0, 40)};
  const RunResult independent = run(ComparisonPolicy::kIndependentEarliestDeadline, cohort);
  const RunResult linked = run(ComparisonPolicy::kLinkedLeastSlack, cohort);

  EXPECT_EQ(receipts(independent),
            (std::vector<std::optional<TimePoint>>{at_ms(21), at_ms(31), at_ms(41), at_ms(51)}));
  EXPECT_EQ(summarize(independent),
            (ReceiptSummary{.offered = 4, .on_time = 2, .late = 2, .missing = 0, .pending = 0}));
  EXPECT_EQ(receipts(linked), (std::vector<std::optional<TimePoint>>{at_ms(21), at_ms(31),
                                                                     std::nullopt, std::nullopt}));
  EXPECT_EQ(summarize(linked),
            (ReceiptSummary{.offered = 4, .on_time = 2, .late = 0, .missing = 2, .pending = 0}));

  const auto late = static_cast<std::size_t>(ReceiptOutcome::kLate);
  const auto missing = static_cast<std::size_t>(ReceiptOutcome::kMissing);
  const auto on_time = static_cast<std::size_t>(ReceiptOutcome::kOnTime);
  const std::optional<std::array<Work, 4>> independent_work = work_by_outcome(independent);
  const std::optional<std::array<Work, 4>> linked_work = work_by_outcome(linked);
  ASSERT_TRUE(independent_work && linked_work);
  const Work two{.upload = Bytes{200'000},
                 .compute = ms(20),
                 .committed_compute = ms(20),
                 .response = Bytes{20'000}};
  EXPECT_EQ(independent_work->at(late), two);
  EXPECT_EQ(independent_work->at(on_time), two);
  EXPECT_EQ(linked_work->at(missing), Work{});
  EXPECT_EQ(linked_work->at(on_time), two);

  EXPECT_EQ(count_commitments(independent), (CommitmentCounts{.none = 4}));
  EXPECT_EQ(count_commitments(linked), (CommitmentCounts{.admitted = 2, .rejected = 2}));
  EXPECT_EQ(plan_deviation(linked), (PlanDeviation{.planned = 2, .delivered_as_planned = 2}));
  EXPECT_EQ(plan_deviation(independent), PlanDeviation{});

  const std::optional<Latency> latency = receipt_latency(independent);
  ASSERT_TRUE(latency.has_value());
  EXPECT_EQ(latency->received, 4U);
  EXPECT_EQ(latency->minimum, ms(21));
  EXPECT_EQ(latency->median, ms(31));
  EXPECT_EQ(latency->p95, ms(51));
  EXPECT_EQ(latency->maximum, ms(51));
  const std::map<RobotId, ReceiptSummary> robots = summarize_by_robot(linked);
  ASSERT_EQ(robots.size(), 4U);
  EXPECT_EQ(robots.at(RobotId{0}).on_time, 1U);
  EXPECT_EQ(robots.at(RobotId{3}).missing, 1U);
}

// A has a 15 ms deadline that no chain can meet; B has 30 ms. Independent stages serve A first by
// deadline, so both arrive late. The linked plan rejects A without serving it and delivers B at
// 21 ms, so B is recovered.
TEST(Comparison, IndependentStagesServeADoomedRequestThatLinkedPlanningRejects) {
  const std::vector<Request> cohort{request(0, 0, 15), request(1, 0, 30)};
  const RunResult independent = run(ComparisonPolicy::kIndependentEarliestDeadline, cohort);
  const RunResult linked = run(ComparisonPolicy::kLinkedLeastSlack, cohort);

  EXPECT_EQ(receipts(independent), (std::vector<std::optional<TimePoint>>{at_ms(21), at_ms(31)}));
  EXPECT_EQ(independent.outcomes.at(0).progress.upload_left, Bytes{0});
  EXPECT_EQ(independent.outcomes.at(0).commitment, Commitment::kNone);
  EXPECT_EQ(linked.outcomes.at(0).commitment, Commitment::kRejected);
  EXPECT_EQ(linked.outcomes.at(0).progress.upload_left, Bytes{100'000});
  EXPECT_EQ(receipts(linked), (std::vector<std::optional<TimePoint>>{std::nullopt, at_ms(21)}));
  EXPECT_EQ(pair_runs(independent, linked),
            (Pairing{.offered = 2, .recovered = 1, .harmed = 0, .unresolved = 0}));
  EXPECT_EQ(pair_runs(linked, independent),
            (Pairing{.offered = 2, .recovered = 0, .harmed = 1, .unresolved = 0}));
}

// The planner budgets 12 ms of compute that actually takes 10 ms. The linked plan would deliver at
// 23 ms against a 22 ms deadline and rejects the request; independent stages deliver at 21 ms.
TEST(Comparison, AConservativePlanCanHarmARequestThatIndependentStagesDeliver) {
  Environment setting = environment();
  setting.compute_planned = ms(12);
  const std::vector<Request> cohort{request(0, 0, 22)};
  const RunResult independent =
      run(ComparisonPolicy::kIndependentEarliestDeadline, cohort, setting);
  const RunResult linked = run(ComparisonPolicy::kLinkedLeastSlack, cohort, setting);
  EXPECT_EQ(receipts(independent), (std::vector<std::optional<TimePoint>>{at_ms(21)}));
  EXPECT_EQ(linked.outcomes.at(0).commitment, Commitment::kRejected);
  EXPECT_EQ(pair_runs(independent, linked),
            (Pairing{.offered = 1, .recovered = 0, .harmed = 1, .unresolved = 0}));
}

// Compute planned at 10 ms takes 12 ms, so the response exists at 22 ms, after its 20 ms downlink
// lease. The lease is lost rather than shifted, and the request receives no unreserved service.
TEST(Comparison, AnAdmittedPlanLosesItsLeaseWhenComputeOverrunsAndGetsNoFreeService) {
  Environment setting = environment();
  setting.compute_actual = ms(12);
  const std::vector<Request> cohort{request(0, 0, 40)};
  const RunResult linked = run(ComparisonPolicy::kLinkedLeastSlack, cohort, setting);
  const RequestOutcome& outcome = linked.outcomes.at(0);
  EXPECT_EQ(outcome.commitment, Commitment::kAdmitted);
  ASSERT_TRUE(outcome.plan.has_value());
  EXPECT_EQ(outcome.plan->delivery, at_ms(21));
  EXPECT_TRUE(outcome.lease_missed);
  EXPECT_EQ(outcome.progress.produced, at_ms(22));
  EXPECT_EQ(outcome.progress.response_left, Bytes{10'000});
  EXPECT_EQ(outcome.outcome, ReceiptOutcome::kMissing);
  EXPECT_EQ(plan_deviation(linked), (PlanDeviation{.planned = 1, .undelivered = 1}));
  EXPECT_EQ(count_commitments(linked), (CommitmentCounts{.admitted = 1, .leases_missed = 1}));

  const RunResult independent =
      run(ComparisonPolicy::kIndependentEarliestDeadline, cohort, setting);
  EXPECT_EQ(receipts(independent), (std::vector<std::optional<TimePoint>>{at_ms(23)}));
}

// Planned at 10 ms each, the computes take 14 ms: the first ends at 24 ms, after its 20 ms lease,
// and pushes the second from its planned 20 ms start to 24 ms, after which its 30 ms lease is lost.
TEST(Comparison, AnOverrunMovesTheNextPlannedComputeAndBothLeasesAreLost) {
  Environment setting = environment();
  setting.compute_actual = ms(14);
  const std::vector<Request> cohort{request(0, 0, 60), request(1, 0, 60)};
  const RunResult linked = run(ComparisonPolicy::kLinkedLeastSlack, cohort, setting);
  EXPECT_EQ(linked.outcomes.at(1).progress.compute_start, at_ms(24));
  EXPECT_EQ(plan_deviation(linked),
            (PlanDeviation{.planned = 2, .undelivered = 2, .compute_moved = 1}));
  EXPECT_EQ(count_commitments(linked), (CommitmentCounts{.admitted = 2, .leases_missed = 2}));
}

TEST(Comparison, EqualSharingSplitsEachLinkSlotAndChangesActualCompletion) {
  const std::vector<Request> cohort{request(0, 0, 60), request(1, 0, 60)};
  const RunResult deadline = run(ComparisonPolicy::kIndependentEarliestDeadline, cohort);
  const RunResult shared = run(ComparisonPolicy::kIndependentEqualShare, cohort);
  EXPECT_EQ(receipts(deadline), (std::vector<std::optional<TimePoint>>{at_ms(21), at_ms(31)}));
  EXPECT_EQ(receipts(shared), (std::vector<std::optional<TimePoint>>{at_ms(31), at_ms(41)}));
  EXPECT_EQ(shared.outcomes.at(0).progress.ready, at_ms(20));
  EXPECT_EQ(shared.outcomes.at(1).progress.ready, at_ms(20));
}

// The future request has the earliest deadline and arrives during the others' uploads, so any
// policy that read it early would change their service or plans before its release.
TEST(Comparison, NoPolicySeesARequestBeforeItsRelease) {
  const std::vector<Request> present{request(0, 0, 60), request(1, 0, 70)};
  const std::vector<Request> with_future{request(0, 0, 60), request(1, 0, 70), request(2, 5, 30)};
  for (const ComparisonPolicy policy : kComparisonPolicies) {
    const RunResult first = run(policy, present);
    const RunResult second = run(policy, with_future);
    EXPECT_EQ(before(first.events, at_ms(5)), before(second.events, at_ms(5))) << name_of(policy);
    for (std::size_t index = 0; index < present.size(); ++index) {
      EXPECT_EQ(first.outcomes.at(index).plan, second.outcomes.at(index).plan) << name_of(policy);
    }
  }
}

TEST(Comparison, EveryPolicyRepeatsExactlyAndObeysTheResourceLawsUnderLoad) {
  const std::optional<std::vector<Request>> cohort =
      make_cohort(CohortSpec{.robots = 6,
                             .requests_per_robot = 4,
                             .period = ms(40),
                             .phase_spread = ms(15),
                             .relative_deadline = ms(45),
                             .upload = Bytes{30'000},
                             .response = Bytes{10'000},
                             .seed = 11});
  ASSERT_TRUE(cohort.has_value());
  const Environment setting = environment(400);
  std::map<ComparisonPolicy, std::uint64_t> digests;
  for (const ComparisonPolicy policy : kComparisonPolicies) {
    const RunResult first = run(policy, *cohort, setting);
    const RunResult again = run(policy, *cohort, setting);
    EXPECT_EQ(first.events, again.events) << name_of(policy);
    EXPECT_EQ(first.digest, again.digest) << name_of(policy);
    expect_resource_laws(first);
    const ReceiptSummary summary = summarize(first);
    EXPECT_EQ(summary.offered, cohort->size()) << name_of(policy);
    EXPECT_EQ(summary.on_time + summary.late + summary.missing + summary.pending, summary.offered)
        << name_of(policy);
    const PlanDeviation deviation = plan_deviation(first);
    EXPECT_EQ(deviation.delivered_as_planned + deviation.delivered_earlier +
                  deviation.delivered_later + deviation.undelivered,
              deviation.planned)
        << name_of(policy);
    digests[policy] = first.digest;
  }
  EXPECT_NE(digests.at(ComparisonPolicy::kLinkedLeastSlack),
            digests.at(ComparisonPolicy::kIndependentEarliestDeadline));
}

// With a 15 ms horizon, B (12 ms deadline) is a known miss while A (30 ms deadline) is still
// unresolved, so the on-time ratio stays undefined and pairing marks A unresolved. Neither chain
// fits the planner's 15-slot calendars, so the linked policy defers both without deciding.
TEST(Comparison, KeepsUnresolvedRequestsPendingAtTheHorizon) {
  const std::vector<Request> cohort{request(0, 0, 30), request(1, 0, 12)};
  for (const ComparisonPolicy policy :
       {ComparisonPolicy::kIndependentEarliestDeadline, ComparisonPolicy::kLinkedLeastSlack}) {
    const RunResult result = run(policy, cohort, environment(15));
    EXPECT_EQ(summarize(result),
              (ReceiptSummary{.offered = 2, .on_time = 0, .late = 0, .missing = 1, .pending = 1}))
        << name_of(policy);
    EXPECT_FALSE(summarize(result).on_time_ratio().has_value());
    EXPECT_EQ(pair_runs(result, result),
              (Pairing{.offered = 2, .recovered = 0, .harmed = 0, .unresolved = 1}));
  }
  const RunResult linked = run(ComparisonPolicy::kLinkedLeastSlack, cohort, environment(15));
  EXPECT_EQ(count_commitments(linked), (CommitmentCounts{.undecided = 2}));
}

TEST(Comparison, RefusesToPairRunsOverDifferentCohortsOrHorizons) {
  const RunResult first = run(ComparisonPolicy::kIndependentEarliestDeadline, {request(0, 0, 60)});
  const RunResult second = run(ComparisonPolicy::kIndependentEarliestDeadline, {request(1, 0, 60)});
  EXPECT_FALSE(pair_runs(first, second).has_value());
  const RunResult longer =
      run(ComparisonPolicy::kIndependentEarliestDeadline, {request(0, 0, 60)}, environment(120));
  EXPECT_FALSE(pair_runs(first, longer).has_value());
}

std::size_t bucket(ReceiptOutcome outcome) { return static_cast<std::size_t>(outcome); }

// A run assembled by hand, to reach metric edges that real runs rarely produce.
RunResult by_hand(std::vector<Request> cohort, const std::vector<Progress>& progress,
                  TimePoint horizon = TimePoint::from_ns(INT64_MAX)) {
  RunResult result{.policy = ComparisonPolicy::kIndependentEarliestDeadline,
                   .cohort = std::move(cohort),
                   .outcomes = {},
                   .events = {},
                   .horizon = horizon,
                   .digest = 0};
  for (const Progress& state : progress) {
    result.outcomes.push_back(RequestOutcome{.outcome = ReceiptOutcome::kMissing,
                                             .commitment = Commitment::kNone,
                                             .plan = std::nullopt,
                                             .lease_missed = false,
                                             .progress = state});
  }
  return result;
}

Progress waiting(Bytes upload_left, Bytes response_left) {
  return Progress{.upload_left = upload_left,
                  .response_left = response_left,
                  .ready = std::nullopt,
                  .compute_start = std::nullopt,
                  .produced = std::nullopt,
                  .receipt = std::nullopt};
}

Progress computed(std::int64_t start_ns, std::int64_t end_ns) {
  return Progress{.upload_left = Bytes{0},
                  .response_left = Bytes{10'000},
                  .ready = TimePoint::from_ns(start_ns),
                  .compute_start = TimePoint::from_ns(start_ns),
                  .produced = TimePoint::from_ns(end_ns),
                  .receipt = std::nullopt};
}

// The lane runs from 10 to 20 ms but the run stops at 15 ms, so only 5 ms of it was observed.
TEST(Comparison, WorkCountsOnlyLaneTimeElapsedByTheHorizon) {
  const RunResult result =
      run(ComparisonPolicy::kIndependentEarliestDeadline, {request(0, 0, 30)}, environment(15));
  ASSERT_EQ(result.outcomes.at(0).progress.compute_start, at_ms(10));
  ASSERT_EQ(result.outcomes.at(0).outcome, ReceiptOutcome::kPending);
  const std::optional<std::array<Work, 4>> work = work_by_outcome(result);
  ASSERT_TRUE(work.has_value());
  EXPECT_EQ(work->at(bucket(ReceiptOutcome::kPending)).compute, ms(5));
  EXPECT_EQ(work->at(bucket(ReceiptOutcome::kPending)).committed_compute, ms(10));
}

// The response arrives at 31 ms, after the 21 ms horizon, so its planned delivery is not observed.
TEST(Comparison, AReceiptAfterTheHorizonIsNotCountedAsDelivered) {
  Environment setting = environment(21);
  setting.downlink_delay = ms(10);
  const RunResult linked = run(ComparisonPolicy::kLinkedLeastSlack, {request(0, 0, 100)}, setting);
  ASSERT_EQ(linked.outcomes.at(0).progress.receipt, at_ms(31));
  ASSERT_EQ(linked.outcomes.at(0).outcome, ReceiptOutcome::kPending);
  EXPECT_EQ(plan_deviation(linked), (PlanDeviation{.planned = 1, .undelivered = 1}));
}

TEST(Comparison, AReceiptExactlyAtTheHorizonAndTheDeadlineIsOnTime) {
  for (const ComparisonPolicy policy : kComparisonPolicies) {
    const RunResult result = run(policy, {request(0, 0, 21)}, environment(21));
    EXPECT_EQ(result.outcomes.at(0).outcome, ReceiptOutcome::kOnTime) << name_of(policy);
  }
}

// One-byte payloads with quarter-millisecond delays and half-millisecond compute: each stage
// starts as soon as its input exists, and the response waits for the next slot boundary.
TEST(Comparison, FractionalDelaysAndComputeRoundServiceUpToTheNextBoundary) {
  Environment setting = environment(8);
  setting.uplink_delay = Duration::from_ns(kMillisecond / 4);
  setting.downlink_delay = Duration::from_ns(kMillisecond / 4);
  setting.compute_planned = Duration::from_ns(kMillisecond / 2);
  setting.compute_actual = Duration::from_ns(kMillisecond / 2);
  Request tiny = request(0, 0, 4);
  tiny.upload = Bytes{1};
  tiny.response = Bytes{1};
  for (const ComparisonPolicy policy : kComparisonPolicies) {
    const RunResult result = run(policy, {tiny}, setting);
    const Progress& progress = result.outcomes.at(0).progress;
    EXPECT_EQ(progress.ready, TimePoint::from_ns(kMillisecond + (kMillisecond / 4)));
    EXPECT_EQ(progress.compute_start, TimePoint::from_ns(kMillisecond + (kMillisecond / 4)));
    EXPECT_EQ(progress.produced, TimePoint::from_ns(kMillisecond + (3 * kMillisecond / 4)));
    EXPECT_EQ(progress.receipt, TimePoint::from_ns((3 * kMillisecond) + (kMillisecond / 4)));
  }
}

// Three bytes of a small upload were served and none of a maximal one, so the total is 3 bytes,
// and no intermediate sum may pass INT64_MAX on the way there.
TEST(Comparison, WorkIsDifferencedPerRequestBeforeItIsSummed) {
  Request small = request(0, 0, 60);
  small.upload = Bytes{3};
  Request huge = request(1, 0, 60);
  huge.upload = Bytes{INT64_MAX};
  const RunResult result = by_hand(
      {small, huge}, {waiting(Bytes{0}, small.response), waiting(huge.upload, huge.response)});
  const std::optional<std::array<Work, 4>> work = work_by_outcome(result);
  ASSERT_TRUE(work.has_value());
  EXPECT_EQ(work->at(bucket(ReceiptOutcome::kMissing)).upload, Bytes{3});
}

TEST(Comparison, RefusesWorkTotalsThatCannotBeRepresented) {
  Request maximal = request(0, 0, 60);
  maximal.upload = Bytes{INT64_MAX};
  Request small = request(1, 0, 60);
  small.upload = Bytes{1};
  EXPECT_FALSE(work_by_outcome(by_hand({maximal, small}, {waiting(Bytes{0}, maximal.response),
                                                          waiting(Bytes{0}, small.response)}))
                   .has_value());
  EXPECT_FALSE(work_by_outcome(by_hand({request(0, 0, 60), request(1, 0, 60)},
                                       {computed(0, 6'100'000'000'000'000'000),
                                        computed(0, 4'000'000'000'000'000'000)}))
                   .has_value());
}

// These fail in a plain build too: wrapping arithmetic would return a value instead of refusing.
TEST(Comparison, RefusesEachUnrepresentableAmountWithoutRelyingOnSanitizers) {
  const Request small = request(0, 0, 60);
  EXPECT_FALSE(
      work_by_outcome(by_hand({small}, {waiting(Bytes{INT64_MIN}, small.response)})).has_value());
  Request maximal = request(1, 0, 60);
  maximal.response = Bytes{INT64_MAX};
  EXPECT_FALSE(work_by_outcome(by_hand({small, maximal},
                                       {waiting(Bytes{0}, Bytes{0}), waiting(Bytes{0}, Bytes{0})}))
                   .has_value());
  const std::optional<std::array<Work, 4>> observed_only = work_by_outcome(
      by_hand({request(0, 0, 60), request(1, 0, 60)},
              {computed(0, 6'100'000'000'000'000'000), computed(0, 4'000'000'000'000'000'000)},
              TimePoint::from_ns(1'000'000'000'000'000'000)));
  EXPECT_FALSE(observed_only.has_value()) << "the committed total overflows even though the "
                                             "observed total does not";
}

// Progress the engine cannot produce: more bytes left than offered, or a compute ending before it
// starts.
TEST(Comparison, RefusesInconsistentProgressInsteadOfCountingNegativeWork) {
  const Request offered = request(0, 0, 60);
  EXPECT_FALSE(
      work_by_outcome(by_hand({offered}, {waiting(Bytes{100'001}, offered.response)})).has_value());
  EXPECT_FALSE(work_by_outcome(by_hand({offered}, {computed(20 * kMillisecond, 10 * kMillisecond)}))
                   .has_value());
}

TEST(Comparison, ComputeIsClippedExactlyAtTheHorizonEdges) {
  const auto work_of = [](const Progress& progress) {
    const std::optional<std::array<Work, 4>> work =
        work_by_outcome(by_hand({request(0, 0, 60)}, {progress}, at_ms(20)));
    return work ? work->at(bucket(ReceiptOutcome::kMissing)) : Work{};
  };
  const Work ends_at_horizon = work_of(computed(10 * kMillisecond, 20 * kMillisecond));
  EXPECT_EQ(ends_at_horizon.compute, ms(10));
  EXPECT_EQ(ends_at_horizon.committed_compute, ms(10));
  const Work starts_at_horizon = work_of(computed(20 * kMillisecond, 30 * kMillisecond));
  EXPECT_EQ(starts_at_horizon.compute, Duration{});
  EXPECT_EQ(starts_at_horizon.committed_compute, Duration{});
  const Work starts_after_horizon = work_of(computed(25 * kMillisecond, 30 * kMillisecond));
  EXPECT_EQ(starts_after_horizon.compute, Duration{});
  EXPECT_EQ(starts_after_horizon.committed_compute, Duration{});
  const Work empty = work_of(computed(10 * kMillisecond, 10 * kMillisecond));
  EXPECT_EQ(empty.compute, Duration{});
  EXPECT_EQ(empty.committed_compute, Duration{});
}

TEST(Comparison, AReceiptExactlyAtTheHorizonIsADelivery) {
  const RunResult linked =
      run(ComparisonPolicy::kLinkedLeastSlack, {request(0, 0, 100)}, environment(21));
  ASSERT_EQ(linked.outcomes.at(0).progress.receipt, at_ms(21));
  EXPECT_EQ(plan_deviation(linked), (PlanDeviation{.planned = 1, .delivered_as_planned = 1}));
}

// Each plan delivers at 25 ms. The receipts at 20, 25, and 30 ms are earlier, as planned, and
// later; the one at 45 ms falls after the 40 ms horizon and is not yet a delivery.
TEST(Comparison, DeliveryCategoriesPartitionThePlansAndTrackTheWorstDelay) {
  std::vector<Progress> progress;
  for (const std::int64_t receipt_ms : {20, 25, 30, 45}) {
    Progress state = computed(10 * kMillisecond, 20 * kMillisecond);
    state.receipt = at_ms(receipt_ms);
    progress.push_back(state);
  }
  RunResult result =
      by_hand({request(0, 0, 60), request(1, 0, 60), request(2, 0, 60), request(3, 0, 60)},
              progress, at_ms(40));
  for (RequestOutcome& outcome : result.outcomes) {
    outcome.plan = ChainReservation{.job = JobId{0},
                                    .worker = WorkerId{1},
                                    .upload = Transfer{.slices = {}, .completion = at_ms(10)},
                                    .compute = Interval{.start = at_ms(10), .end = at_ms(20)},
                                    .download = Transfer{.slices = {}, .completion = at_ms(25)},
                                    .deadline = at_ms(60),
                                    .slack = ms(35),
                                    .delivery = at_ms(25)};
  }
  EXPECT_EQ(plan_deviation(result), (PlanDeviation{.planned = 4,
                                                   .delivered_as_planned = 1,
                                                   .delivered_earlier = 1,
                                                   .delivered_later = 1,
                                                   .undelivered = 1,
                                                   .compute_moved = 0,
                                                   .compute_not_started = 0,
                                                   .worst_delivery_delay = ms(5)}));
}

TEST(Comparison, RefusesEndpointCapsOverMoreThanTenMillionRobotSlots) {
  Environment wide = environment(5'000'001);
  wide.endpoint_uplink = Bytes{5'000};
  EXPECT_FALSE(run_comparison(ComparisonPolicy::kIndependentEarliestDeadline, wide,
                              {request(0, 0, 60), request(1, 0, 60)})
                   .has_value());
}

}  // namespace
}  // namespace rovi::core
