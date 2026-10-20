// Single-job transactional admission of an upload, inference, and action-chunk download chain.
#ifndef ROVI_CORE_ADMISSION_HPP_
#define ROVI_CORE_ADMISSION_HPP_

#include <cstdint>
#include <optional>
#include <vector>

#include "rovi_core/capacity_calendar.hpp"
#include "rovi_core/interval_calendar.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/units.hpp"

namespace rovi::core {

struct ExecutionProfile {
  ModelClass model;
  Duration duration;

  bool operator==(const ExecutionProfile&) const = default;
};

struct Worker {
  WorkerId id;
  std::vector<ExecutionProfile> profiles;
  IntervalCalendar lane;

  bool operator==(const Worker&) const = default;
};

enum class ServiceSource : std::uint8_t { kLogical, kEnforced, kMeasured, kReplayed };

// The service one robot may receive per slot in each direction, summed over all its requests; an
// absent calendar leaves that direction uncapped.
struct EndpointBudget {
  RobotId robot;
  std::optional<CapacityCalendar> uplink;
  std::optional<CapacityCalendar> downlink;

  bool operator==(const EndpointBudget&) const = default;
};

struct ResourceSnapshot {
  CapacityCalendar uplink;
  CapacityCalendar downlink;
  std::vector<Worker> workers;
  std::optional<CapacityCalendar> shared_medium{};
  Duration uplink_delay{};
  Duration downlink_delay{};
  Duration dispatch_lead{};
  ServiceSource source{ServiceSource::kLogical};
  std::vector<EndpointBudget> endpoints{};

  bool operator==(const ResourceSnapshot&) const = default;
};

struct ChainReservation {
  JobId job;
  WorkerId worker;
  Transfer upload;
  Interval compute;
  Transfer download;
  TimePoint deadline;
  Duration slack;
  TimePoint delivery{};
  RobotId robot{};

  bool operator==(const ChainReservation&) const = default;
};

enum class DecisionStatus : std::uint8_t { kRejected, kDeferred, kAdmitted };

enum class ReasonCode : std::uint8_t {
  kNone,
  kInvalidSnapshot,
  kInvalidPayloadSize,
  kInvalidTiming,
  kDuplicateJob,
  kArithmeticOverflow,
  kNoEligibleWorker,
  kPredecessorIncomplete,
  kUplinkServiceUnavailable,
  kNoFeasibleChain,
  kDeadlineMiss,
  kStaleResult,
  kGroupMemberUnavailable,
  kGroupSkewExceeded,
  kQueueFull,
  kMalformedRequest,
};

struct Decision {
  DecisionStatus status{DecisionStatus::kRejected};
  ReasonCode reason{ReasonCode::kNone};

  bool operator==(const Decision&) const = default;
};

struct AdmissionResult {
  Decision decision;
  std::optional<ChainReservation> reservation;
  ResourceSnapshot snapshot;
};

[[nodiscard]] bool is_consistent(const ResourceSnapshot& snapshot);

[[nodiscard]] std::optional<ResourceSnapshot> commit(const ResourceSnapshot& snapshot,
                                                     const ChainReservation& chain);

[[nodiscard]] AdmissionResult admit(const ResourceSnapshot& snapshot, const Job& job,
                                    TimePoint now);

}  // namespace rovi::core

#endif  // ROVI_CORE_ADMISSION_HPP_
