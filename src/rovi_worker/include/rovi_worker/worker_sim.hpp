// Deterministic non-preemptive inference lane that executes admitted jobs in reserved-start order.
#ifndef ROVI_WORKER_WORKER_SIM_HPP_
#define ROVI_WORKER_WORKER_SIM_HPP_

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <vector>

#include "rovi_core/admission.hpp"
#include "rovi_core/units.hpp"

namespace rovi::worker {

struct Assignment {
  core::JobId job;
  core::RobotId robot;
  core::ModelClass model;
  core::TimePoint planned_start;
  core::Bytes chunk_bytes;
};

struct WorkerTiming {
  core::Duration slowdown;
  core::Duration orphan_timeout;
  core::Duration max_lead;
};

struct Completion {
  core::JobId job;
  core::RobotId robot;
  core::Bytes chunk_bytes;
  core::TimePoint observation_time;
  core::TimePoint start;
  core::TimePoint end;

  bool operator==(const Completion&) const = default;
};

class WorkerSim {
 public:
  [[nodiscard]] static std::optional<WorkerSim> create(
      const std::vector<core::ExecutionProfile>& profiles, WorkerTiming timing);

  bool assign(const Assignment& assignment, core::TimePoint now);
  void arrive(core::JobId job, core::TimePoint at, core::TimePoint observation_time);
  [[nodiscard]] std::vector<Completion> advance(core::TimePoint now);
  [[nodiscard]] std::size_t queued() const;
  [[nodiscard]] std::uint64_t discarded() const;

 private:
  struct Arrival {
    core::TimePoint at;
    core::TimePoint observation_time;
  };

  WorkerSim(std::map<core::ModelClass, core::Duration> durations, WorkerTiming timing);

  bool start_next(core::TimePoint now);
  void prune(core::TimePoint now);

  std::map<core::ModelClass, core::Duration> durations_;
  WorkerTiming timing_;
  std::map<core::JobId, Assignment> assigned_;
  std::map<core::JobId, Arrival> arrived_;
  std::optional<Completion> running_;
  std::optional<core::TimePoint> lane_free_;
  std::uint64_t discarded_{0};
};

}  // namespace rovi::worker

#endif  // ROVI_WORKER_WORKER_SIM_HPP_
