// Observation-to-action-chunk job description and effective-deadline derivation.
#ifndef ROVI_CORE_JOB_HPP_
#define ROVI_CORE_JOB_HPP_

#include <cstdint>
#include <optional>
#include <vector>

#include "rovi_core/units.hpp"

namespace rovi::core {

struct Job {
  JobId id;
  TimePoint release_time;
  TimePoint observation_time;
  TimePoint buffer_empty_time;
  Duration safety_margin;
  std::optional<TimePoint> task_deadline;
  std::optional<Duration> maximum_result_age;
  Bytes upload_bytes;
  Bytes action_chunk_bytes;
  ModelClass model;
  std::vector<WorkerId> eligible_workers;
  RobotId robot{};
  std::vector<JobId> predecessors{};
  std::optional<GroupId> synchronization_group{};
  Duration maximum_result_ready_skew{};
  std::uint64_t sequence{0};

  bool operator==(const Job&) const = default;
};

[[nodiscard]] std::optional<TimePoint> effective_deadline(const Job& job);
[[nodiscard]] std::optional<TimePoint> freshness_deadline(const Job& job);

}  // namespace rovi::core

#endif  // ROVI_CORE_JOB_HPP_
