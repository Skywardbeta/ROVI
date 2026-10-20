// Command-line entry point that runs fixed-cohort comparisons of the linked and independent-stage
// policies on the common execution engine and prints a deterministic YAML report.
#include <algorithm>
#include <array>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <ios>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "rovi_core/cohort.hpp"
#include "rovi_core/comparison.hpp"
#include "rovi_core/engine.hpp"
#include "rovi_core/receipt.hpp"
#include "rovi_core/units.hpp"
#include "rovi_core/wireless_profile.hpp"

namespace {

namespace core = rovi::core;

constexpr int kRunError = 1;
constexpr int kUsageError = 2;
constexpr std::int64_t kMillisecond = 1'000'000;
constexpr std::int64_t kHundredthsPerUnit = 10'000;
constexpr std::int64_t kHundredths = 100;
constexpr int kDigestWidth = 16;

struct Options {
  std::vector<core::ComparisonPolicy> policies{core::ComparisonPolicy::kIndependentEarliestDeadline,
                                               core::ComparisonPolicy::kIndependentEqualShare,
                                               core::ComparisonPolicy::kLinkedLeastSlack,
                                               core::ComparisonPolicy::kLinkedEarliestDeadline,
                                               core::ComparisonPolicy::kLinkedFirstInFirstOut};
  core::CohortSpec cohort{.robots = 4,
                          .requests_per_robot = 10,
                          .period = core::Duration::from_ns(100 * kMillisecond),
                          .phase_spread = core::Duration{},
                          .relative_deadline = core::Duration::from_ns(40 * kMillisecond),
                          .upload = core::Bytes{100'000},
                          .response = core::Bytes{10'000},
                          .seed = 0};
  std::int64_t slot_ns{kMillisecond};
  std::int64_t slots{1'200};
  std::int64_t uplink_bps{80'000'000};
  std::int64_t downlink_bps{80'000'000};
  std::int64_t uplink_delay_ns{0};
  std::int64_t downlink_delay_ns{0};
  std::int64_t compute_ns{10 * kMillisecond};
  std::optional<std::int64_t> compute_actual_ns;
  std::optional<std::int64_t> endpoint_uplink_bytes;
  std::optional<std::int64_t> endpoint_downlink_bytes;
  std::optional<std::int64_t> medium_bps;
  core::RecoveryRules recovery;
};

using Setter = bool (*)(Options&, std::int64_t);

bool count_into(std::uint32_t& field, std::int64_t value) {
  if (value < 0 || std::cmp_greater(value, std::numeric_limits<std::uint32_t>::max())) {
    return false;
  }
  field = static_cast<std::uint32_t>(value);
  return true;
}

const std::map<std::string_view, Setter>& numeric_options() {
  static const std::map<std::string_view, Setter> options{
      {"--robots", [](Options& o, std::int64_t v) { return count_into(o.cohort.robots, v); }},
      {"--requests",
       [](Options& o, std::int64_t v) { return count_into(o.cohort.requests_per_robot, v); }},
      {"--period-ns",
       [](Options& o, std::int64_t v) {
         o.cohort.period = core::Duration::from_ns(v);
         return true;
       }},
      {"--phase-spread-ns",
       [](Options& o, std::int64_t v) {
         o.cohort.phase_spread = core::Duration::from_ns(v);
         return true;
       }},
      {"--deadline-ns",
       [](Options& o, std::int64_t v) {
         o.cohort.relative_deadline = core::Duration::from_ns(v);
         return true;
       }},
      {"--upload-bytes",
       [](Options& o, std::int64_t v) {
         o.cohort.upload = core::Bytes{v};
         return true;
       }},
      {"--response-bytes",
       [](Options& o, std::int64_t v) {
         o.cohort.response = core::Bytes{v};
         return true;
       }},
      {"--slot-ns",
       [](Options& o, std::int64_t v) {
         o.slot_ns = v;
         return true;
       }},
      {"--slots",
       [](Options& o, std::int64_t v) {
         o.slots = v;
         return true;
       }},
      {"--uplink-bps",
       [](Options& o, std::int64_t v) {
         o.uplink_bps = v;
         return true;
       }},
      {"--downlink-bps",
       [](Options& o, std::int64_t v) {
         o.downlink_bps = v;
         return true;
       }},
      {"--uplink-delay-ns",
       [](Options& o, std::int64_t v) {
         o.uplink_delay_ns = v;
         return true;
       }},
      {"--downlink-delay-ns",
       [](Options& o, std::int64_t v) {
         o.downlink_delay_ns = v;
         return true;
       }},
      {"--compute-ns",
       [](Options& o, std::int64_t v) {
         o.compute_ns = v;
         return true;
       }},
      {"--compute-actual-ns",
       [](Options& o, std::int64_t v) {
         o.compute_actual_ns = v;
         return true;
       }},
      {"--endpoint-uplink-bytes",
       [](Options& o, std::int64_t v) {
         o.endpoint_uplink_bytes = v;
         return v >= 0;
       }},
      {"--endpoint-downlink-bytes",
       [](Options& o, std::int64_t v) {
         o.endpoint_downlink_bytes = v;
         return v >= 0;
       }},
      {"--medium-bps",
       [](Options& o, std::int64_t v) {
         o.medium_bps = v;
         return true;
       }},
      {"--best-effort",
       [](Options& o, std::int64_t v) {
         o.recovery.best_effort = v == 1;
         return v == 0 || v == 1;
       }},
      {"--recovery-compute-bound-ns",
       [](Options& o, std::int64_t v) {
         o.recovery.compute_bound = core::Duration::from_ns(v);
         return v > 0;
       }},
      {"--supersede",
       [](Options& o, std::int64_t v) {
         o.recovery.supersede = v == 1;
         return v == 0 || v == 1;
       }},
      {"--maximum-age-ns",
       [](Options& o, std::int64_t v) {
         o.recovery.maximum_age = core::Duration::from_ns(v);
         return v > 0;
       }},
      {"--backlog",
       [](Options& o, std::int64_t v) {
         if (v <= 0) {
           return false;
         }
         o.recovery.backlog = static_cast<std::size_t>(v);
         return true;
       }},
  };
  return options;
}

template <typename Number>
std::optional<Number> number_in(std::string_view text) {
  Number value{};
  const auto [parsed_to, error] = std::from_chars(text.begin(), text.end(), value);
  if (error != std::errc{} || parsed_to != text.end()) {
    return std::nullopt;
  }
  return value;
}

// A comma-separated list of distinct policy names, with no empty entries.
std::optional<std::vector<core::ComparisonPolicy>> policies_in(std::string_view list) {
  std::vector<core::ComparisonPolicy> parsed;
  while (true) {
    const std::size_t comma = list.find(',');
    const std::optional<core::ComparisonPolicy> policy = core::policy_named(list.substr(0, comma));
    if (!policy || std::ranges::find(parsed, *policy) != parsed.end()) {
      return std::nullopt;
    }
    parsed.push_back(*policy);
    if (comma == std::string_view::npos) {
      return parsed;
    }
    list = list.substr(comma + 1);
  }
}

struct Assignment {
  std::string_view name;
  std::string_view value;
};

bool apply(Options& options, const Assignment& assignment) {
  if (assignment.name == "--policies") {
    std::optional<std::vector<core::ComparisonPolicy>> parsed = policies_in(assignment.value);
    if (parsed) {
      options.policies = std::move(*parsed);
    }
    return parsed.has_value();
  }
  if (assignment.name == "--seed") {
    const std::optional<std::uint64_t> seed = number_in<std::uint64_t>(assignment.value);
    options.cohort.seed = seed.value_or(0);
    return seed.has_value();
  }
  const auto setter = numeric_options().find(assignment.name);
  const std::optional<std::int64_t> number = number_in<std::int64_t>(assignment.value);
  return setter != numeric_options().end() && number && setter->second(options, *number);
}

std::optional<Options> parse(const std::vector<std::string_view>& arguments) {
  Options options;
  std::set<std::string_view> given;
  for (std::size_t index = 1; index < arguments.size(); index += 2) {
    const std::string_view name = arguments.at(index);
    if (index + 1 >= arguments.size() || !given.insert(name).second) {
      std::cerr << "rovi_compare: " << name << " is missing its value or repeated\n";
      return std::nullopt;
    }
    if (!apply(options, {.name = name, .value = arguments.at(index + 1)})) {
      std::cerr << "rovi_compare: invalid " << name << " " << arguments.at(index + 1) << "\n";
      return std::nullopt;
    }
  }
  return options;
}

std::optional<core::Bytes> bytes_or_none(const std::optional<std::int64_t>& value) {
  return value ? std::optional{core::Bytes{*value}} : std::nullopt;
}

std::string or_none(const std::optional<std::int64_t>& value) {
  return value ? std::to_string(*value) : std::string{"none"};
}

core::Environment environment_of(const Options& options) {
  const auto profile = [](std::int64_t bps) {
    return std::vector<core::ServiceSegment>{
        {.first_slot = 0, .rate = core::BitsPerSecond{bps}, .reduction = core::Permille{0}}};
  };
  return core::Environment{
      .slot_length = core::Duration::from_ns(options.slot_ns),
      .slot_count = options.slots,
      .uplink = profile(options.uplink_bps),
      .downlink = profile(options.downlink_bps),
      .uplink_delay = core::Duration::from_ns(options.uplink_delay_ns),
      .downlink_delay = core::Duration::from_ns(options.downlink_delay_ns),
      .compute_planned = core::Duration::from_ns(options.compute_ns),
      .compute_actual =
          core::Duration::from_ns(options.compute_actual_ns.value_or(options.compute_ns)),
      .medium = options.medium_bps ? std::optional{profile(*options.medium_bps)} : std::nullopt,
      .endpoint_uplink = bytes_or_none(options.endpoint_uplink_bytes),
      .endpoint_downlink = bytes_or_none(options.endpoint_downlink_bytes)};
}

std::string hex(std::uint64_t value) {
  std::ostringstream text;
  text << std::hex << std::setw(kDigestWidth) << std::setfill('0') << value;
  return text.str();
}

std::string flag(bool value) { return value ? "true" : "false"; }

std::string fraction(core::Fraction value) {
  return std::to_string(value.count) + "/" + std::to_string(value.of);
}

std::string outcomes(const core::ReceiptSummary& summary) {
  return "{offered: " + std::to_string(summary.offered) +
         ", on_time: " + std::to_string(summary.on_time) +
         ", late: " + std::to_string(summary.late) +
         ", missing: " + std::to_string(summary.missing) +
         ", pending: " + std::to_string(summary.pending) + "}";
}

std::string_view linked_execution(const core::RecoveryRules& recovery) {
  if (!recovery.best_effort) {
    return "  linked_execution: strict; a missed lease is lost, never replanned or shifted\n";
  }
  return recovery.compute_bound
             ? "  linked_execution: best_effort; a missed lease is never replanned or shifted; that"
               " request, like a rejected one, gets only unreserved service and computes only in a"
               " lane gap that fits the declared compute bound\n"
             : "  linked_execution: best_effort; a missed lease is never replanned or shifted; that"
               " request, like a rejected one, gets only unreserved service and is never computed,"
               " since no compute bound is declared\n";
}

void print_header(const Options& options, const std::vector<core::Request>& cohort,
                  const core::Environment& environment, core::TimePoint horizon) {
  const bool cancels =
      options.recovery.maximum_age || options.recovery.supersede || options.recovery.backlog;
  std::cout
      << "harness: rovi_compare\nformat_version: 1\n"
      << "claims:\n  measured_network_service: false\n  measured_inference: false\n"
      << "  vla_policy_or_action_use: false\n  task_success: false\n  optimality: false\n"
      << "model:\n"
      << "  network: fixed-slot logical capacity; data is first served at the slot boundary at or"
         " after it exists and completes at the end of its last slot, then its fixed delay\n"
      << "  compute: one non-preemptive synthetic lane, batch size one\n"
      << "  robots: independent clients, no cooperative groups\n"
      << "  outcome: complete response received by the original deadline\n"
      << "  information: a policy sees a request from the first slot boundary at or after its"
         " release; linked policies also read the declared capacity profile\n"
      << (cancels
              ? "  independent: work-conserving; a request is dropped only by the declared recovery"
                " rules, never because its deadline has passed\n"
              : "  independent: work-conserving; a stage never drops a request whose deadline has"
                " passed\n")
      << "  equal_share: a link-sharing stress diagnostic, not a competitive policy\n"
      << linked_execution(options.recovery)
      << (cancels
              ? "  cancellation: maximum age, supersession, and backlog apply only to work served"
                " without a reservation\n"
              : "")
      << "  offered: the whole cohort, including requests released after the horizon, which stay"
         " pending\n"
      << "  latency: receipt time minus release time\n"
      << "cohort:\n  robots: " << options.cohort.robots
      << "\n  requests_per_robot: " << options.cohort.requests_per_robot
      << "\n  period_ns: " << options.cohort.period.ns()
      << "\n  phase_spread_ns: " << options.cohort.phase_spread.ns()
      << "\n  relative_deadline_ns: " << options.cohort.relative_deadline.ns()
      << "\n  upload_bytes: " << options.cohort.upload.value()
      << "\n  response_bytes: " << options.cohort.response.value()
      << "\n  seed: " << options.cohort.seed << "\n  offered: " << cohort.size()
      << "\n  digest: " << hex(core::cohort_digest(cohort)) << "\n"
      << "environment:\n  slot_ns: " << environment.slot_length.ns()
      << "\n  slots: " << environment.slot_count << "\n  horizon_ns: " << horizon.ns()
      << "\n  uplink_bps: " << options.uplink_bps << "\n  downlink_bps: " << options.downlink_bps
      << "\n  uplink_delay_ns: " << environment.uplink_delay.ns()
      << "\n  downlink_delay_ns: " << environment.downlink_delay.ns()
      << "\n  compute_planned_ns: " << environment.compute_planned.ns()
      << "\n  compute_actual_ns: " << environment.compute_actual.ns() << "\n"
      << "  endpoint_uplink_bytes: " << or_none(options.endpoint_uplink_bytes) << "\n"
      << "  endpoint_downlink_bytes: " << or_none(options.endpoint_downlink_bytes) << "\n"
      << "  medium_bps: " << or_none(options.medium_bps) << "\n"
      << "recovery: {best_effort: " << flag(options.recovery.best_effort) << ", compute_bound_ns: "
      << (options.recovery.compute_bound ? std::to_string(options.recovery.compute_bound->ns())
                                         : std::string{"none"})
      << ", maximum_age_ns: "
      << (options.recovery.maximum_age ? std::to_string(options.recovery.maximum_age->ns())
                                       : std::string{"none"})
      << ", supersede: " << flag(options.recovery.supersede) << ", backlog: "
      << (options.recovery.backlog ? std::to_string(*options.recovery.backlog)
                                   : std::string{"none"})
      << "}\n"
      << "reference_policy: " << core::name_of(options.policies.front()) << "\nruns:\n";
}

void print_ratio(const core::ReceiptSummary& summary) {
  if (const std::optional<core::Fraction> ratio = summary.on_time_ratio()) {
    std::cout << "    on_time_ratio: " << fraction(*ratio) << "\n";
  } else if (const auto bounds = summary.on_time_bounds()) {
    std::cout << "    on_time_ratio: undefined while requests are pending\n"
              << "    on_time_bounds: [" << fraction(bounds->first) << ", "
              << fraction(bounds->second) << "]\n";
  }
}

void print_latency(const core::RunResult& run) {
  if (const std::optional<core::Latency> latency = core::receipt_latency(run)) {
    std::cout << "    receipt_latency_ns: {received: " << latency->received
              << ", minimum: " << latency->minimum.ns() << ", median: " << latency->median.ns()
              << ", p95: " << latency->p95.ns() << ", maximum: " << latency->maximum.ns() << "}\n";
    return;
  }
  std::cout << "    receipt_latency_ns: none received\n";
}

void print_work(const core::RunResult& run) {
  constexpr std::array<std::string_view, 4> kOutcomes{"on_time", "late", "missing", "pending"};
  const std::optional<std::array<core::Work, 4>> work = core::work_by_outcome(run);
  if (!work) {
    std::cout << "    work: not representable in 64-bit integers\n";
    return;
  }
  std::cout << "    work:\n";
  for (std::size_t index = 0; index < work->size(); ++index) {
    const core::Work& spent = work->at(index);
    std::cout << "      " << kOutcomes.at(index) << ": {upload_bytes: " << spent.upload.value()
              << ", compute_ns: " << spent.compute.ns()
              << ", committed_compute_ns: " << spent.committed_compute.ns()
              << ", response_bytes: " << spent.response.value() << "}\n";
  }
}

void print_run(const core::RunResult& run) {
  const core::Capabilities capabilities = core::capabilities_of(run.policy);
  const core::ReceiptSummary summary = core::summarize(run);
  const core::CommitmentCounts commitments = core::count_commitments(run);
  const core::PlanDeviation deviation = core::plan_deviation(run);
  std::cout << "  - policy: " << core::name_of(run.policy) << "\n"
            << "    capabilities: {cross_stage_planning: "
            << flag(capabilities.cross_stage_planning)
            << ", atomic_admission: " << flag(capabilities.atomic_admission)
            << ", capacity_lookahead: " << flag(capabilities.capacity_lookahead) << "}\n"
            << "    outcomes: " << outcomes(summary) << "\n";
  print_ratio(summary);
  print_latency(run);
  std::cout << "    commitments: {none: " << commitments.none
            << ", admitted: " << commitments.admitted << ", rejected: " << commitments.rejected
            << ", undecided: " << commitments.undecided
            << ", leases_missed: " << commitments.leases_missed << "}\n";
  const core::RecoveryCounts recovery = core::count_recovery(run);
  std::cout << "    recovery_counts: {entered: " << recovery.entered
            << ", delivered: " << recovery.delivered << ", expired: " << recovery.expired
            << ", superseded: " << recovery.superseded
            << ", backlog_full: " << recovery.backlog_full << "}\n";
  print_work(run);
  std::cout << "    plan_versus_actual: {planned: " << deviation.planned
            << ", delivered_as_planned: " << deviation.delivered_as_planned
            << ", delivered_earlier: " << deviation.delivered_earlier
            << ", delivered_later: " << deviation.delivered_later
            << ", undelivered: " << deviation.undelivered
            << ", compute_moved: " << deviation.compute_moved
            << ", compute_not_started: " << deviation.compute_not_started
            << ", worst_delivery_delay_ns: " << deviation.worst_delivery_delay.ns() << "}\n"
            << "    per_robot:\n";
  for (const auto& [robot, counts] : core::summarize_by_robot(run)) {
    std::cout << "      - {robot: " << robot.value() << ", outcomes: " << outcomes(counts) << "}\n";
  }
  std::cout << "    trace_digest: " << hex(run.digest) << "\n";
}

// 100 * (recovered - harmed) / offered percentage points, truncated to hundredths.
std::string net_change(const core::Pairing& pairing) {
  const std::int64_t change =
      static_cast<std::int64_t>(pairing.recovered) - static_cast<std::int64_t>(pairing.harmed);
  const std::int64_t hundredths =
      change * kHundredthsPerUnit / static_cast<std::int64_t>(pairing.offered);
  const std::int64_t magnitude = hundredths < 0 ? -hundredths : hundredths;
  std::ostringstream text;
  text << change << "/" << pairing.offered << " (" << (change < 0 ? "-" : "+")
       << magnitude / kHundredths << "." << std::setw(2) << std::setfill('0')
       << magnitude % kHundredths << " pp)";
  return text.str();
}

void print_pairings(const std::vector<core::RunResult>& runs) {
  std::cout << "paired_with_reference:\n";
  for (std::size_t index = 1; index < runs.size(); ++index) {
    const std::optional<core::Pairing> pairing = core::pair_runs(runs.front(), runs.at(index));
    if (!pairing || pairing->offered == 0) {
      continue;
    }
    std::cout << "  - candidate: " << core::name_of(runs.at(index).policy)
              << "\n    offered: " << pairing->offered << "\n    recovered: " << pairing->recovered
              << "\n    harmed: " << pairing->harmed << "\n    unresolved: " << pairing->unresolved
              << "\n    net_delivery_change: " << net_change(*pairing) << "\n";
  }
}

void print_usage() {
  std::cerr
      << "usage: rovi_compare [--policies name,...] [--robots n] [--requests n]"
         " [--period-ns t] [--phase-spread-ns t] [--deadline-ns t] [--upload-bytes b]"
         " [--response-bytes b] [--seed s] [--slot-ns t] [--slots n] [--uplink-bps r]"
         " [--downlink-bps r] [--uplink-delay-ns t] [--downlink-delay-ns t]"
         " [--compute-ns t] [--compute-actual-ns t] [--endpoint-uplink-bytes b]"
         " [--endpoint-downlink-bytes b] [--medium-bps r] [--best-effort 0|1]"
         " [--recovery-compute-bound-ns t] [--supersede 0|1] [--maximum-age-ns t] [--backlog n]\n";
}

}  // namespace

int main(int argc, char** argv) {
  const std::span<char* const> raw{argv, static_cast<std::size_t>(argc)};
  const std::optional<Options> options =
      parse(std::vector<std::string_view>(raw.begin(), raw.end()));
  if (!options) {
    print_usage();
    return kUsageError;
  }
  const std::optional<std::vector<core::Request>> cohort = core::make_cohort(options->cohort);
  if (!cohort) {
    std::cerr << "rovi_compare: invalid cohort\n";
    return kRunError;
  }
  const core::Environment environment = environment_of(*options);
  if (options->recovery.compute_bound &&
      *options->recovery.compute_bound < environment.compute_planned) {
    std::cerr << "rovi_compare: --recovery-compute-bound-ns is shorter than the planned compute\n";
    return kRunError;
  }
  std::vector<core::RunResult> runs;
  for (const core::ComparisonPolicy policy : options->policies) {
    std::optional<core::RunResult> run =
        core::run_comparison(policy, environment, *cohort, options->recovery);
    if (!run) {
      std::cerr << "rovi_compare: invalid environment for " << core::name_of(policy) << "\n";
      return kRunError;
    }
    runs.push_back(std::move(*run));
  }
  print_header(*options, *cohort, environment, runs.front().horizon);
  for (const core::RunResult& run : runs) {
    print_run(run);
  }
  print_pairings(runs);
  return 0;
}
