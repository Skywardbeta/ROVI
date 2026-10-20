#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <vector>

#include "rovi_core/cohort.hpp"
#include "rovi_core/comparison.hpp"
#include "rovi_core/engine.hpp"
#include "rovi_core/receipt.hpp"
#include "rovi_core/units.hpp"

namespace rovi::core {
namespace {

constexpr std::size_t kMedianPercent = 50;
constexpr std::size_t kTailPercent = 95;
constexpr std::size_t kPercent = 100;

void count(ReceiptSummary& summary, ReceiptOutcome outcome) {
  ++summary.offered;
  switch (outcome) {
    case ReceiptOutcome::kOnTime:
      ++summary.on_time;
      break;
    case ReceiptOutcome::kLate:
      ++summary.late;
      break;
    case ReceiptOutcome::kMissing:
      ++summary.missing;
      break;
    case ReceiptOutcome::kPending:
      ++summary.pending;
      break;
  }
}

// Nearest-rank percentile of a sorted, non-empty sample.
Duration percentile(const std::vector<Duration>& sorted, std::size_t percent) {
  const std::size_t rank = ((percent * sorted.size()) + kPercent - 1) / kPercent;
  return sorted.at(std::max<std::size_t>(rank, 1) - 1);
}

bool received(ReceiptOutcome outcome) {
  return outcome == ReceiptOutcome::kOnTime || outcome == ReceiptOutcome::kLate;
}

std::optional<std::int64_t> difference(std::int64_t total, std::int64_t left) {
  std::int64_t value{};
  if (__builtin_sub_overflow(total, left, &value)) {
    return std::nullopt;
  }
  return value;
}

std::optional<Work> served(const Request& request, const Progress& progress, TimePoint horizon) {
  const std::optional<std::int64_t> upload =
      difference(request.upload.value(), progress.upload_left.value());
  const std::optional<std::int64_t> response =
      difference(request.response.value(), progress.response_left.value());
  if (!upload || !response || *upload < 0 || *response < 0) {
    return std::nullopt;
  }
  Work work{.upload = Bytes{*upload},
            .compute = Duration{},
            .committed_compute = Duration{},
            .response = Bytes{*response}};
  if (!progress.compute_start || !progress.produced || *progress.compute_start >= horizon) {
    return work;
  }
  const std::optional<Duration> committed = elapsed(*progress.compute_start, *progress.produced);
  const std::optional<Duration> observed =
      elapsed(*progress.compute_start, std::min(*progress.produced, horizon));
  if (!committed || !observed || *committed < Duration{}) {
    return std::nullopt;
  }
  work.committed_compute = *committed;
  work.compute = *observed;
  return work;
}

bool add(Work& total, const Work& more) {
  std::int64_t upload{};
  std::int64_t compute{};
  std::int64_t committed{};
  std::int64_t response{};
  if (__builtin_add_overflow(total.upload.value(), more.upload.value(), &upload) ||
      __builtin_add_overflow(total.compute.ns(), more.compute.ns(), &compute) ||
      __builtin_add_overflow(total.committed_compute.ns(), more.committed_compute.ns(),
                             &committed) ||
      __builtin_add_overflow(total.response.value(), more.response.value(), &response)) {
    return false;
  }
  total = Work{.upload = Bytes{upload},
               .compute = Duration::from_ns(compute),
               .committed_compute = Duration::from_ns(committed),
               .response = Bytes{response}};
  return true;
}

}  // namespace

ReceiptSummary summarize(const RunResult& run) {
  ReceiptSummary summary;
  for (const RequestOutcome& outcome : run.outcomes) {
    count(summary, outcome.outcome);
  }
  return summary;
}

std::map<RobotId, ReceiptSummary> summarize_by_robot(const RunResult& run) {
  std::map<RobotId, ReceiptSummary> robots;
  for (std::size_t index = 0; index < run.outcomes.size(); ++index) {
    count(robots[run.cohort.at(index).robot], run.outcomes.at(index).outcome);
  }
  return robots;
}

std::optional<Latency> receipt_latency(const RunResult& run) {
  std::vector<Duration> latencies;
  for (std::size_t index = 0; index < run.outcomes.size(); ++index) {
    const RequestOutcome& outcome = run.outcomes.at(index);
    const std::optional<Duration> latency =
        received(outcome.outcome) && outcome.progress.receipt
            ? elapsed(run.cohort.at(index).release, *outcome.progress.receipt)
            : std::nullopt;
    if (latency) {
      latencies.push_back(*latency);
    }
  }
  if (latencies.empty()) {
    return std::nullopt;
  }
  std::ranges::sort(latencies);
  return Latency{.received = latencies.size(),
                 .minimum = latencies.front(),
                 .median = percentile(latencies, kMedianPercent),
                 .p95 = percentile(latencies, kTailPercent),
                 .maximum = latencies.back()};
}

std::optional<std::array<Work, 4>> work_by_outcome(const RunResult& run) {
  std::array<Work, 4> work{};
  for (std::size_t index = 0; index < run.outcomes.size(); ++index) {
    const RequestOutcome& outcome = run.outcomes.at(index);
    const std::optional<Work> spent = served(run.cohort.at(index), outcome.progress, run.horizon);
    if (!spent || !add(work.at(static_cast<std::size_t>(outcome.outcome)), *spent)) {
      return std::nullopt;
    }
  }
  return work;
}

CommitmentCounts count_commitments(const RunResult& run) {
  CommitmentCounts counts;
  for (const RequestOutcome& outcome : run.outcomes) {
    switch (outcome.commitment) {
      case Commitment::kNone:
        ++counts.none;
        break;
      case Commitment::kAdmitted:
        ++counts.admitted;
        break;
      case Commitment::kRejected:
        ++counts.rejected;
        break;
      case Commitment::kUndecided:
        ++counts.undecided;
        break;
    }
    counts.leases_missed += outcome.lease_missed ? 1U : 0U;
  }
  return counts;
}

PlanDeviation plan_deviation(const RunResult& run) {
  PlanDeviation deviation;
  for (const RequestOutcome& outcome : run.outcomes) {
    if (!outcome.plan) {
      continue;
    }
    ++deviation.planned;
    const std::optional<TimePoint>& started = outcome.progress.compute_start;
    deviation.compute_not_started += started ? 0U : 1U;
    deviation.compute_moved += started && *started != outcome.plan->compute.start ? 1U : 0U;
    const std::optional<TimePoint>& receipt = outcome.progress.receipt;
    if (!receipt || *receipt > run.horizon) {
      ++deviation.undelivered;
    } else if (*receipt > outcome.plan->delivery) {
      ++deviation.delivered_later;
      deviation.worst_delivery_delay =
          std::max(deviation.worst_delivery_delay,
                   elapsed(outcome.plan->delivery, *receipt).value_or(Duration{}));
    } else if (*receipt < outcome.plan->delivery) {
      ++deviation.delivered_earlier;
    } else {
      ++deviation.delivered_as_planned;
    }
  }
  return deviation;
}

RecoveryCounts count_recovery(const RunResult& run) {
  RecoveryCounts counts;
  for (const RequestOutcome& outcome : run.outcomes) {
    counts.entered += outcome.best_effort ? 1U : 0U;
    counts.delivered += outcome.best_effort && received(outcome.outcome) ? 1U : 0U;
    if (!outcome.dropped) {
      continue;
    }
    switch (*outcome.dropped) {
      case DropReason::kExpired:
        ++counts.expired;
        break;
      case DropReason::kSuperseded:
        ++counts.superseded;
        break;
      case DropReason::kBacklogFull:
        ++counts.backlog_full;
        break;
    }
  }
  return counts;
}

std::optional<Pairing> pair_runs(const RunResult& reference, const RunResult& candidate) {
  if (reference.cohort != candidate.cohort || reference.horizon != candidate.horizon ||
      reference.outcomes.size() != candidate.outcomes.size()) {
    return std::nullopt;
  }
  Pairing pairing{.offered = reference.cohort.size()};
  for (std::size_t index = 0; index < reference.outcomes.size(); ++index) {
    const ReceiptOutcome before = reference.outcomes.at(index).outcome;
    const ReceiptOutcome after = candidate.outcomes.at(index).outcome;
    if (before == ReceiptOutcome::kPending || after == ReceiptOutcome::kPending) {
      ++pairing.unresolved;
      continue;
    }
    const bool was_on_time = before == ReceiptOutcome::kOnTime;
    const bool is_on_time = after == ReceiptOutcome::kOnTime;
    pairing.recovered += !was_on_time && is_on_time ? 1U : 0U;
    pairing.harmed += was_on_time && !is_on_time ? 1U : 0U;
  }
  return pairing;
}

}  // namespace rovi::core
