// Fixed-cohort runs of the linked-plan policies and an independent-stage control on one common
// execution engine, with outcome summaries and paired comparisons.
#ifndef ROVI_CORE_COMPARISON_HPP_
#define ROVI_CORE_COMPARISON_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string_view>
#include <vector>

#include "rovi_core/admission.hpp"
#include "rovi_core/cohort.hpp"
#include "rovi_core/engine.hpp"
#include "rovi_core/receipt.hpp"
#include "rovi_core/units.hpp"

namespace rovi::core {

// The linked policies order the full-chain planner. The independent policies decide each
// stage separately from what has actually happened; the equal-share variant splits each link slot
// evenly instead of by deadline.
enum class ComparisonPolicy : std::uint8_t {
  kLinkedLeastSlack,
  kLinkedEarliestDeadline,
  kLinkedFirstInFirstOut,
  kIndependentEarliestDeadline,
  kIndependentEqualShare,
};

inline constexpr std::array<ComparisonPolicy, 5> kComparisonPolicies{
    ComparisonPolicy::kLinkedLeastSlack, ComparisonPolicy::kLinkedEarliestDeadline,
    ComparisonPolicy::kLinkedFirstInFirstOut, ComparisonPolicy::kIndependentEarliestDeadline,
    ComparisonPolicy::kIndependentEqualShare};

struct Capabilities {
  bool cross_stage_planning{false};
  bool atomic_admission{false};
  bool capacity_lookahead{false};

  bool operator==(const Capabilities&) const = default;
};

[[nodiscard]] Capabilities capabilities_of(ComparisonPolicy policy);
[[nodiscard]] std::string_view name_of(ComparisonPolicy policy);
[[nodiscard]] std::optional<ComparisonPolicy> policy_named(std::string_view name);

enum class Commitment : std::uint8_t { kNone, kAdmitted, kRejected, kUndecided };

// How work without a deadline promise is treated. Strict, the labeled control, serves a linked
// request only through its reservation. Best effort also serves rejected requests and requests that
// lost a reservation, from service the commitments leave unused. The cancellation rules apply to
// every request served without a promise, which for the independent policies is every request.
struct RecoveryRules {
  bool best_effort{false};
  // Worst-case best-effort compute, fitted into lane gaps. Reservations stay protected only if
  // actual compute never exceeds it. Without it, best-effort compute never starts.
  std::optional<Duration> compute_bound{};
  std::optional<Duration> maximum_age{};
  bool supersede{false};
  std::optional<std::size_t> backlog{};

  bool operator==(const RecoveryRules&) const = default;
};

enum class DropReason : std::uint8_t { kExpired, kSuperseded, kBacklogFull };

struct RequestOutcome {
  ReceiptOutcome outcome{ReceiptOutcome::kPending};
  Commitment commitment{Commitment::kNone};
  std::optional<ChainReservation> plan;
  bool lease_missed{false};
  Progress progress;
  bool best_effort{false};
  // Its compute ran in a lane gap, not in a reservation.
  bool unreserved_compute{false};
  std::optional<DropReason> dropped{};
};

struct RunResult {
  ComparisonPolicy policy{ComparisonPolicy::kLinkedLeastSlack};
  std::vector<Request> cohort;
  std::vector<RequestOutcome> outcomes;
  std::vector<Event> events;
  TimePoint horizon;
  std::uint64_t digest{0};
  RecoveryRules recovery{};
};

// Runs one policy over the cohort until the environment's horizon. Every policy sees a request only
// from its release and acts only through the engine; nullopt means the inputs were invalid or a
// policy proposed an action the engine refused.
[[nodiscard]] std::optional<RunResult> run_comparison(ComparisonPolicy policy,
                                                      const Environment& environment,
                                                      const std::vector<Request>& cohort,
                                                      const RecoveryRules& recovery = {});

struct Latency {
  std::uint64_t received{0};
  Duration minimum;
  Duration median;
  Duration p95;
  Duration maximum;
};

// Service a request received by the horizon. `compute` is lane time elapsed by then, and
// `committed_compute` is the full lane occupancy of every compute started before it.
struct Work {
  Bytes upload{};
  Duration compute{};
  Duration committed_compute{};
  Bytes response{};

  bool operator==(const Work&) const = default;
};

struct CommitmentCounts {
  std::uint64_t none{0};
  std::uint64_t admitted{0};
  std::uint64_t rejected{0};
  std::uint64_t undecided{0};
  std::uint64_t leases_missed{0};

  bool operator==(const CommitmentCounts&) const = default;
};

// Admitted plans against what was observed by the horizon: the delivery counts partition `planned`,
// a receipt after the horizon counting as undelivered, and the compute counts cover plans whose
// compute started at another time or never started.
struct PlanDeviation {
  std::uint64_t planned{0};
  std::uint64_t delivered_as_planned{0};
  std::uint64_t delivered_earlier{0};
  std::uint64_t delivered_later{0};
  std::uint64_t undelivered{0};
  std::uint64_t compute_moved{0};
  std::uint64_t compute_not_started{0};
  Duration worst_delivery_delay{};

  bool operator==(const PlanDeviation&) const = default;
};

// Work served without a reservation: how much entered, how much of it was delivered by the horizon
// on time or late, and how much each declared rule dropped.
struct RecoveryCounts {
  std::uint64_t entered{0};
  std::uint64_t delivered{0};
  std::uint64_t expired{0};
  std::uint64_t superseded{0};
  std::uint64_t backlog_full{0};

  bool operator==(const RecoveryCounts&) const = default;
};

// Matched outcomes of two runs over the same cohort: recovered requests are on time only in the
// candidate, harmed ones only in the reference. A pair with a pending side is unresolved.
struct Pairing {
  std::uint64_t offered{0};
  std::uint64_t recovered{0};
  std::uint64_t harmed{0};
  std::uint64_t unresolved{0};

  bool operator==(const Pairing&) const = default;
};

[[nodiscard]] ReceiptSummary summarize(const RunResult& run);
[[nodiscard]] std::map<RobotId, ReceiptSummary> summarize_by_robot(const RunResult& run);
[[nodiscard]] std::optional<Latency> receipt_latency(const RunResult& run);
// Totals by outcome, or nullopt for progress the engine cannot produce or a total that cannot be
// represented.
[[nodiscard]] std::optional<std::array<Work, 4>> work_by_outcome(const RunResult& run);
[[nodiscard]] CommitmentCounts count_commitments(const RunResult& run);
[[nodiscard]] PlanDeviation plan_deviation(const RunResult& run);
[[nodiscard]] RecoveryCounts count_recovery(const RunResult& run);
[[nodiscard]] std::optional<Pairing> pair_runs(const RunResult& reference,
                                               const RunResult& candidate);

}  // namespace rovi::core

#endif  // ROVI_CORE_COMPARISON_HPP_
