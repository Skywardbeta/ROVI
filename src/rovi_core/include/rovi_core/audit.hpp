// Independent first-principles audit of a schedule result against the ROVI resource invariants.
#ifndef ROVI_CORE_AUDIT_HPP_
#define ROVI_CORE_AUDIT_HPP_

#include <cstdint>
#include <vector>

#include "rovi_core/admission.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/scheduler.hpp"
#include "rovi_core/units.hpp"

namespace rovi::core {

enum class Violation : std::uint8_t {
  kCapacityExceeded,
  kStageOrder,
  kDeliveryMismatch,
  kDeadlineExceeded,
  kComputeOverlap,
  kComputeLengthMismatch,
  kUnknownJob,
  kUnknownWorker,
  kGroupNotAtomic,
  kGroupSkewExceeded,
  kPredecessorIncomplete,
  kSnapshotMismatch,
  kDigestMismatch,
};

struct AuditFinding {
  Violation violation{Violation::kSnapshotMismatch};
  JobId job;

  bool operator==(const AuditFinding&) const = default;
};

[[nodiscard]] std::vector<AuditFinding> audit_schedule(const ResourceSnapshot& initial,
                                                       const std::vector<Job>& pending,
                                                       const std::vector<JobId>& completed,
                                                       const ScheduleResult& result);

}  // namespace rovi::core

#endif  // ROVI_CORE_AUDIT_HPP_
