// Fixed offered request cohort, identical for every compared policy, with robot phases seeded by
// stable robot identity rather than by draw order.
#ifndef ROVI_CORE_COHORT_HPP_
#define ROVI_CORE_COHORT_HPP_

#include <cstdint>
#include <optional>
#include <vector>

#include "rovi_core/units.hpp"

namespace rovi::core {

struct Request {
  JobId id;
  RobotId robot;
  TimePoint release;
  TimePoint deadline;
  Bytes upload;
  Bytes response;
  // How long after its release the robot actually sends the observation; the planner never sees it.
  Duration send_delay{};

  bool operator==(const Request&) const = default;
};

struct CohortSpec {
  std::uint32_t robots{0};
  std::uint32_t requests_per_robot{0};
  Duration period;
  // Each robot's first release lies in [0, phase_spread); zero releases every robot together.
  Duration phase_spread;
  Duration relative_deadline;
  Bytes upload;
  Bytes response;
  std::uint64_t seed{0};
};

inline constexpr std::uint64_t kMaximumCohort = 1'000'000;

// Requests are ordered by release, robot, and sequence. A request's id is its robot in the high 32
// bits and its sequence within that robot in the low 32 bits.
[[nodiscard]] std::optional<std::vector<Request>> make_cohort(const CohortSpec& spec);
[[nodiscard]] std::uint64_t cohort_digest(const std::vector<Request>& cohort);

}  // namespace rovi::core

#endif  // ROVI_CORE_COHORT_HPP_
