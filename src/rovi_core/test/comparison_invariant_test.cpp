#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <map>
#include <numeric>
#include <optional>
#include <random>
#include <tuple>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "rovi_core/cohort.hpp"
#include "rovi_core/comparison.hpp"
#include "rovi_core/engine.hpp"
#include "rovi_core/receipt.hpp"
#include "rovi_core/units.hpp"
#include "rovi_core/wireless_profile.hpp"

namespace rovi::core {
namespace {

constexpr std::int64_t kMillisecond = 1'000'000;
constexpr std::uint64_t kTrials = 300;
// 96 Mbps shared by both directions: narrower than the two links together, so it binds.
constexpr std::int64_t kMediumSlotBytes = 12'000;

// 80 Mbps over 1 ms slots, then an uplink outage from slot 12, 40 Mbps uplink from slot 16, and
// 16 Mbps downlink from slot 17.
std::int64_t uplink_capacity(std::int64_t slot) {
  if (slot < 12) {
    return 10'000;
  }
  return slot < 16 ? 0 : 5'000;
}

std::int64_t downlink_capacity(std::int64_t slot) { return slot < 17 ? 10'000 : 2'000; }

ServiceSegment rate(std::int64_t first_slot, std::int64_t bps) {
  return ServiceSegment{
      .first_slot = first_slot, .rate = BitsPerSecond{bps}, .reduction = Permille{0}};
}

std::vector<ServiceSegment> uplink_profile() {
  return {rate(0, 80'000'000), rate(12, 0), rate(16, 40'000'000)};
}

std::vector<ServiceSegment> downlink_profile() {
  return {rate(0, 80'000'000), rate(17, 16'000'000)};
}

Environment random_environment(std::mt19937_64& generator) {
  const auto draw = [&generator](std::int64_t bound) {
    return static_cast<std::int64_t>(generator() % static_cast<std::uint64_t>(bound));
  };
  return Environment{
      .slot_length = Duration::from_ns(kMillisecond),
      .slot_count = 18 + draw(82),
      .uplink = uplink_profile(),
      .downlink = downlink_profile(),
      .uplink_delay = Duration::from_ns(draw(12 * kMillisecond)),
      .downlink_delay = Duration::from_ns(draw(12 * kMillisecond)),
      .compute_planned = Duration::from_ns(1 + draw(8 * kMillisecond)),
      .compute_actual = Duration::from_ns(1 + draw(8 * kMillisecond)),
      .medium = draw(3) == 0 ? std::optional<std::vector<ServiceSegment>>{{rate(0, 96'000'000)}}
                             : std::nullopt,
      .endpoint_uplink = draw(2) == 0 ? std::optional{Bytes{3'000 + draw(8'000)}} : std::nullopt,
      .endpoint_downlink = draw(2) == 0 ? std::optional{Bytes{1'000 + draw(4'000)}} : std::nullopt};
}

RecoveryRules random_rules(std::mt19937_64& generator, const Environment& environment) {
  const auto draw = [&generator](std::int64_t bound) {
    return static_cast<std::int64_t>(generator() % static_cast<std::uint64_t>(bound));
  };
  return RecoveryRules{
      .best_effort = draw(2) == 0,
      .compute_bound = draw(3) == 0
                           ? std::nullopt
                           : std::optional{Duration::from_ns(environment.compute_planned.ns() +
                                                             draw(3 * kMillisecond))},
      .maximum_age =
          draw(2) == 0
              ? std::optional{Duration::from_ns((2 * kMillisecond) + draw(30 * kMillisecond))}
              : std::nullopt,
      .supersede = draw(2) == 0,
      .backlog =
          draw(2) == 0 ? std::optional{static_cast<std::size_t>(1 + draw(4))} : std::nullopt};
}

std::vector<Request> random_cohort(std::mt19937_64& generator) {
  const auto draw = [&generator](std::int64_t bound) {
    return static_cast<std::int64_t>(generator() % static_cast<std::uint64_t>(bound));
  };
  std::vector<Request> cohort;
  const auto count = static_cast<std::uint32_t>(1 + draw(8));
  for (std::uint32_t robot = 0; robot < count; ++robot) {
    const std::int64_t release = draw(20 * kMillisecond);
    cohort.push_back(
        Request{.id = JobId{robot},
                .robot = RobotId{static_cast<std::uint32_t>(draw(1 + (count / 2)))},
                .release = TimePoint::from_ns(release),
                .deadline = TimePoint::from_ns(release + draw(60 * kMillisecond)),
                .upload = Bytes{1 + draw(25'000)},
                .response = Bytes{1 + draw(12'000)},
                .send_delay = Duration::from_ns(draw(3) == 0 ? draw(4 * kMillisecond) : 0)});
  }
  std::ranges::sort(cohort, {}, &Request::release);
  return cohort;
}

void expect_service_and_lane_laws(const RunResult& result, const Environment& environment) {
  const std::size_t count = result.cohort.size();
  std::vector<std::int64_t> uploaded(count, 0);
  std::vector<std::int64_t> responded(count, 0);
  std::vector<std::pair<TimePoint, TimePoint>> lane;
  std::map<std::pair<EventKind, std::int64_t>, std::int64_t> per_slot;
  std::map<std::tuple<EventKind, RobotId, std::int64_t>, std::int64_t> per_robot;
  std::map<std::int64_t, std::int64_t> per_medium_slot;
  for (const Event& event : result.events) {
    ASSERT_LT(event.request, count);
    const Request& offered = result.cohort.at(event.request);
    const Progress& progress = result.outcomes.at(event.request).progress;
    const std::int64_t slot = event.time.ns() / kMillisecond;
    if (event.kind == EventKind::kUploadService || event.kind == EventKind::kResponseService) {
      EXPECT_EQ(event.time.ns() % kMillisecond, 0);
      EXPECT_LT(event.time, result.horizon);
      EXPECT_TRUE(!progress.dropped || event.time < *progress.dropped) << "served after a drop";
      const bool upload = event.kind == EventKind::kUploadService;
      const std::optional<Bytes>& cap =
          upload ? environment.endpoint_uplink : environment.endpoint_downlink;
      std::int64_t& robot_total = per_robot[{event.kind, offered.robot, slot}];
      robot_total += event.bytes.value();
      EXPECT_TRUE(!cap || robot_total <= cap->value()) << "endpoint cap exceeded";
      std::int64_t& medium_total = per_medium_slot[slot];
      medium_total += event.bytes.value();
      EXPECT_TRUE(!environment.medium || medium_total <= kMediumSlotBytes)
          << "coupled medium exceeded";
      std::int64_t& total = per_slot[{event.kind, slot}];
      total += event.bytes.value();
      if (event.kind == EventKind::kUploadService) {
        EXPECT_GE(event.time.ns(), offered.release.ns() + offered.send_delay.ns());
        EXPECT_LE(total, uplink_capacity(slot));
        uploaded.at(event.request) += event.bytes.value();
      } else {
        ASSERT_TRUE(progress.produced.has_value());
        EXPECT_GE(event.time, *progress.produced);
        EXPECT_LE(total, downlink_capacity(slot));
        responded.at(event.request) += event.bytes.value();
      }
    } else if (event.kind == EventKind::kUploadComplete) {
      ASSERT_TRUE(progress.ready.has_value());
      EXPECT_EQ(progress.ready->ns(), event.time.ns() + environment.uplink_delay.ns());
    } else if (event.kind == EventKind::kComputeStart) {
      ASSERT_TRUE(progress.ready && progress.produced);
      EXPECT_GE(event.time, *progress.ready);
      EXPECT_EQ(progress.produced->ns() - event.time.ns(), environment.compute_actual.ns());
      lane.emplace_back(event.time, *progress.produced);
    }
  }
  std::ranges::sort(lane);
  for (std::size_t index = 1; index < lane.size(); ++index) {
    EXPECT_GE(lane.at(index).first, lane.at(index - 1).second);
  }
  for (std::size_t index = 0; index < count; ++index) {
    const Progress& progress = result.outcomes.at(index).progress;
    EXPECT_EQ(uploaded.at(index),
              result.cohort.at(index).upload.value() - progress.upload_left.value());
    EXPECT_EQ(responded.at(index),
              result.cohort.at(index).response.value() - progress.response_left.value());
    EXPECT_EQ(result.outcomes.at(index).outcome,
              classify_receipt(result.cohort.at(index).deadline, progress.receipt, result.horizon));
  }
}

std::array<Work, 4> work_rebuilt(const RunResult& result) {
  std::array<Work, 4> work{};
  for (std::size_t index = 0; index < result.outcomes.size(); ++index) {
    const Request& offered = result.cohort.at(index);
    const Progress& progress = result.outcomes.at(index).progress;
    Work& spent = work.at(static_cast<std::size_t>(result.outcomes.at(index).outcome));
    spent.upload =
        Bytes{spent.upload.value() + (offered.upload.value() - progress.upload_left.value())};
    spent.response =
        Bytes{spent.response.value() + (offered.response.value() - progress.response_left.value())};
    if (progress.compute_start && progress.produced) {
      const std::int64_t start = progress.compute_start->ns();
      const std::int64_t observed_end = std::min(progress.produced->ns(), result.horizon.ns());
      spent.compute = Duration::from_ns(spent.compute.ns() + (observed_end - start));
      spent.committed_compute =
          Duration::from_ns(spent.committed_compute.ns() + (progress.produced->ns() - start));
    }
  }
  return work;
}

void expect_metric_laws(const RunResult& result) {
  const std::optional<std::array<Work, 4>> work = work_by_outcome(result);
  ASSERT_TRUE(work.has_value());
  EXPECT_EQ(*work, work_rebuilt(result));
  for (const Work& spent : *work) {
    EXPECT_LE(spent.compute, spent.committed_compute);
  }
  EXPECT_LE(
      std::accumulate(work->begin(), work->end(), std::int64_t{0},
                      [](std::int64_t sum, const Work& spent) { return sum + spent.compute.ns(); }),
      result.horizon.ns());
  const auto observed_deliveries = static_cast<std::uint64_t>(
      std::ranges::count_if(result.outcomes, [&result](const RequestOutcome& outcome) {
        return outcome.plan && outcome.progress.receipt &&
               *outcome.progress.receipt <= result.horizon;
      }));
  const PlanDeviation deviation = plan_deviation(result);
  EXPECT_EQ(
      deviation.delivered_as_planned + deviation.delivered_earlier + deviation.delivered_later,
      observed_deliveries);
  EXPECT_EQ(observed_deliveries + deviation.undelivered, deviation.planned);
}

// While the compute model holds and every observation is sent on release, nothing may disturb a
// reservation: each admitted chain keeps its lease and arrives exactly when planned.
void expect_reservations_kept(const RunResult& result, const Environment& environment) {
  if (environment.compute_actual > environment.compute_planned) {
    return;
  }
  for (std::size_t index = 0; index < result.outcomes.size(); ++index) {
    const RequestOutcome& outcome = result.outcomes.at(index);
    if (outcome.commitment != Commitment::kAdmitted ||
        result.cohort.at(index).send_delay != Duration{}) {
      continue;
    }
    ASSERT_TRUE(outcome.plan.has_value());
    EXPECT_FALSE(outcome.lease_missed) << "request " << index;
    EXPECT_EQ(outcome.progress.receipt, outcome.plan->delivery) << "request " << index;
  }
}

// A drop ends only work served without a reservation, for a reason the declared rules give, before
// any receipt and after any compute it had already started.
void expect_lawful_drops(const RunResult& result, const RecoveryRules& rules) {
  const bool linked = capabilities_of(result.policy).atomic_admission;
  for (std::size_t index = 0; index < result.outcomes.size(); ++index) {
    const RequestOutcome& outcome = result.outcomes.at(index);
    const Progress& progress = outcome.progress;
    ASSERT_EQ(outcome.dropped.has_value(), progress.dropped.has_value());
    if (!progress.dropped) {
      continue;
    }
    const TimePoint at = *progress.dropped;
    const Request& offered = result.cohort.at(index);
    EXPECT_FALSE(progress.receipt.has_value()) << "request " << index;
    EXPECT_TRUE(!progress.compute_start || *progress.compute_start < at) << "request " << index;
    EXPECT_TRUE(!linked || outcome.best_effort) << "request " << index;
    const auto newer = [&](const Request& other) {
      return other.robot == offered.robot && other.release <= at;
    };
    switch (*outcome.dropped) {
      case DropReason::kExpired:
        EXPECT_TRUE(rules.maximum_age && at.ns() - offered.release.ns() > rules.maximum_age->ns());
        break;
      case DropReason::kSuperseded:
        EXPECT_TRUE(
            rules.supersede &&
            std::any_of(std::next(result.cohort.begin(), static_cast<std::ptrdiff_t>(index + 1)),
                        result.cohort.end(), newer));
        break;
      case DropReason::kBacklogFull:
        EXPECT_TRUE(rules.backlog.has_value());
        break;
    }
  }
}

// A bound that covers the actual compute keeps every unreserved compute clear of every reservation;
// without a bound, none is dispatched.
void expect_unreserved_compute_clear(const RunResult& result, const Environment& environment,
                                     const RecoveryRules& rules) {
  for (std::size_t index = 0; index < result.outcomes.size(); ++index) {
    const RequestOutcome& outcome = result.outcomes.at(index);
    if (!outcome.unreserved_compute) {
      continue;
    }
    EXPECT_TRUE(outcome.best_effort) << "request " << index;
    ASSERT_TRUE(rules.compute_bound.has_value()) << "request " << index;
    ASSERT_TRUE(outcome.progress.compute_start && outcome.progress.produced);
    if (*rules.compute_bound < environment.compute_actual) {
      continue;
    }
    for (const RequestOutcome& reserved : result.outcomes) {
      EXPECT_TRUE(!reserved.plan || *outcome.progress.produced <= reserved.plan->compute.start ||
                  *outcome.progress.compute_start >= reserved.plan->compute.end)
          << "request " << index << " overlaps a reserved compute";
    }
  }
}

struct Coverage {
  int censored_deliveries{0};
  int clipped_computes{0};
  int unreserved_computes{0};
  int drops_while_waiting{0};
  int deferred_computes{0};

  void add(const RunResult& result, const RecoveryRules& rules) {
    for (const RequestOutcome& outcome : result.outcomes) {
      const Progress& progress = outcome.progress;
      censored_deliveries +=
          outcome.plan && progress.receipt && *progress.receipt > result.horizon ? 1 : 0;
      clipped_computes += progress.produced && *progress.produced > result.horizon ? 1 : 0;
      unreserved_computes += outcome.unreserved_compute ? 1 : 0;
      const bool waiting = outcome.best_effort && progress.ready && !progress.compute_start;
      drops_while_waiting +=
          waiting && progress.dropped && *progress.ready <= *progress.dropped ? 1 : 0;
      deferred_computes += waiting && !progress.dropped && !rules.compute_bound ? 1 : 0;
    }
  }
};

// Each trial has its own seed, so a failure can be replayed from the seed it prints.
TEST(ComparisonInvariant, RandomRunsConserveServiceLaneOutcomesAndMetrics) {
  Coverage coverage;
  for (std::uint64_t seed = 1; seed <= kTrials; ++seed) {
    SCOPED_TRACE(::testing::Message() << "seed " << seed);
    std::mt19937_64 generator{seed};
    const Environment environment = random_environment(generator);
    const std::vector<Request> cohort = random_cohort(generator);
    const RecoveryRules rules = random_rules(generator, environment);
    for (const ComparisonPolicy policy : kComparisonPolicies) {
      SCOPED_TRACE(name_of(policy));
      const std::optional<RunResult> result = run_comparison(policy, environment, cohort, rules);
      ASSERT_TRUE(result.has_value());
      EXPECT_EQ(result->cohort, cohort);
      expect_service_and_lane_laws(*result, environment);
      expect_metric_laws(*result);
      expect_reservations_kept(*result, environment);
      expect_lawful_drops(*result, rules);
      expect_unreserved_compute_clear(*result, environment, rules);
      const std::optional<RunResult> again = run_comparison(policy, environment, cohort, rules);
      ASSERT_TRUE(again.has_value());
      EXPECT_EQ(again->digest, result->digest);
      EXPECT_EQ(again->events, result->events);
      const RecoveryCounts counts = count_recovery(*result);
      EXPECT_LE(counts.delivered, counts.entered);
      coverage.add(*result, rules);
    }
  }
  EXPECT_GT(coverage.censored_deliveries, 0) << "no planned receipt fell after the horizon";
  EXPECT_GT(coverage.clipped_computes, 0) << "no compute crossed the horizon";
  EXPECT_GT(coverage.unreserved_computes, 0) << "no recovery compute was dispatched";
  EXPECT_GT(coverage.drops_while_waiting, 0) << "no recovery was cancelled while it waited";
  EXPECT_GT(coverage.deferred_computes, 0) << "no recovery waited for a declared bound";
}

// Every request misses its deadline at release and recovers best effort, with inputs arriving late
// and out of deadline order, and cancellation while work waits, so the dispatch rule alone decides
// the lane. The bound equals the actual compute, so the predicted lane is the real one.
struct RecoveryTrial {
  Environment environment;
  std::vector<Request> cohort;
  RecoveryRules rules;
};

RecoveryTrial random_recovery_trial(std::mt19937_64& generator) {
  const auto draw = [&generator](std::int64_t bound) {
    return static_cast<std::int64_t>(generator() % static_cast<std::uint64_t>(bound));
  };
  const Duration compute = Duration::from_ns(1 + draw(8 * kMillisecond));
  RecoveryTrial trial{
      .environment = Environment{.slot_length = Duration::from_ns(kMillisecond),
                                 .slot_count = 80,
                                 .uplink = uplink_profile(),
                                 .downlink = downlink_profile(),
                                 .uplink_delay = Duration::from_ns(draw(12 * kMillisecond)),
                                 .downlink_delay = Duration::from_ns(draw(3 * kMillisecond)),
                                 .compute_planned = compute,
                                 .compute_actual = compute},
      .cohort = {},
      .rules = RecoveryRules{.best_effort = true,
                             .compute_bound = compute,
                             .maximum_age = draw(2) == 0
                                                ? std::optional{Duration::from_ns(
                                                      (2 * kMillisecond) + draw(20 * kMillisecond))}
                                                : std::nullopt,
                             .supersede = draw(2) == 0}};
  const auto count = static_cast<std::uint32_t>(2 + draw(7));
  for (std::uint32_t index = 0; index < count; ++index) {
    const std::int64_t release = draw(15 * kMillisecond);
    trial.cohort.push_back(
        Request{.id = JobId{index},
                .robot = RobotId{static_cast<std::uint32_t>(draw(1 + (count / 2)))},
                .release = TimePoint::from_ns(release),
                .deadline = TimePoint::from_ns(release + 1 + draw(2 * kMillisecond)),
                .upload = Bytes{1 + draw(30'000)},
                .response = Bytes{1 + draw(10'000)},
                .send_delay = Duration::from_ns(draw(2) == 0 ? draw(10 * kMillisecond) : 0)});
  }
  std::ranges::sort(trial.cohort, {}, &Request::release);
  return trial;
}

bool before(const Request& lhs, const Request& rhs) {
  return std::tie(lhs.deadline, lhs.release, lhs.id) < std::tie(rhs.deadline, rhs.release, rhs.id);
}

// The lane never idles while recovery work is ready, and each dispatch takes the earliest-deadline
// request ready at that moment.
void expect_recovery_dispatch_laws(const RunResult& result) {
  std::vector<std::pair<TimePoint, TimePoint>> lane;
  for (const RequestOutcome& outcome : result.outcomes) {
    ASSERT_EQ(outcome.commitment, Commitment::kRejected);
    if (outcome.progress.compute_start) {
      lane.emplace_back(*outcome.progress.compute_start, *outcome.progress.produced);
    }
  }
  std::ranges::sort(lane);
  for (std::size_t index = 0; index < result.outcomes.size(); ++index) {
    const Progress& waiting = result.outcomes.at(index).progress;
    if (!waiting.ready) {
      continue;
    }
    const TimePoint until =
        waiting.compute_start ? *waiting.compute_start : waiting.dropped.value_or(result.horizon);
    TimePoint busy = *waiting.ready;
    for (const auto& [start, end] : lane) {
      busy = start <= busy && busy < end ? end : busy;
    }
    EXPECT_GE(busy, until) << "request " << index << " waited at an idle lane";
    for (std::size_t other = 0; other < result.outcomes.size(); ++other) {
      const Progress& chosen = result.outcomes.at(other).progress;
      const bool passed_over = chosen.compute_start && *waiting.ready <= *chosen.compute_start &&
                               until > *chosen.compute_start;
      EXPECT_TRUE(!passed_over || before(result.cohort.at(other), result.cohort.at(index)))
          << "request " << other << " started before earlier-deadline request " << index;
    }
  }
}

TEST(ComparisonInvariant, RecoveryDispatchTakesReadyWorkByDeadlineAndNeverIdles) {
  int inverted = 0;
  int cancelled_waiting = 0;
  for (std::uint64_t seed = 1; seed <= kTrials; ++seed) {
    SCOPED_TRACE(::testing::Message() << "seed " << seed);
    std::mt19937_64 generator{seed};
    const RecoveryTrial trial = random_recovery_trial(generator);
    const std::optional<RunResult> result = run_comparison(
        ComparisonPolicy::kLinkedLeastSlack, trial.environment, trial.cohort, trial.rules);
    ASSERT_TRUE(result.has_value());
    expect_service_and_lane_laws(*result, trial.environment);
    expect_lawful_drops(*result, trial.rules);
    expect_recovery_dispatch_laws(*result);
    for (std::size_t index = 0; index < result->outcomes.size(); ++index) {
      const Progress& progress = result->outcomes.at(index).progress;
      cancelled_waiting += progress.ready && progress.dropped &&
                                   *progress.ready <= *progress.dropped && !progress.compute_start
                               ? 1
                               : 0;
      for (std::size_t other = 0; other < index; ++other) {
        const Progress& earlier = result->outcomes.at(other).progress;
        inverted += progress.ready && earlier.ready &&
                            before(result->cohort.at(index), result->cohort.at(other)) !=
                                (*progress.ready < *earlier.ready)
                        ? 1
                        : 0;
      }
    }
  }
  EXPECT_GT(inverted, 0) << "no inputs arrived out of deadline order";
  EXPECT_GT(cancelled_waiting, 0) << "no ready recovery was cancelled while it waited";
}

}  // namespace
}  // namespace rovi::core
