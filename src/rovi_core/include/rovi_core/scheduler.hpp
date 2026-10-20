// One rolling-horizon cycle of least-slack transactional reservation over atomic cooperative units.
#ifndef ROVI_CORE_SCHEDULER_HPP_
#define ROVI_CORE_SCHEDULER_HPP_

#include <cstdint>
#include <optional>
#include <vector>

#include "rovi_core/admission.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/units.hpp"

namespace rovi::core {

enum class Policy : std::uint8_t { kLeastSlack, kEarliestDeadline, kFirstInFirstOut };

struct JobDecision {
  JobId job;
  Decision decision;
  std::optional<ChainReservation> reservation;

  bool operator==(const JobDecision&) const = default;
};

struct ScheduleResult {
  std::vector<JobDecision> decisions;
  ResourceSnapshot snapshot;
  ServiceSource source{ServiceSource::kLogical};
  std::uint64_t digest{0};
};

[[nodiscard]] ScheduleResult schedule(const ResourceSnapshot& snapshot,
                                      const std::vector<Job>& pending,
                                      const std::vector<JobId>& completed, TimePoint now,
                                      Policy policy);

[[nodiscard]] std::uint64_t decision_digest(const std::vector<JobDecision>& decisions);

}  // namespace rovi::core

#endif  // ROVI_CORE_SCHEDULER_HPP_
