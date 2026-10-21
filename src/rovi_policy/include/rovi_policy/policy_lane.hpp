// One real-inference lane: runs an admitted job no earlier than its reserved compute start, once
// its observation has arrived, and never two at once.
#ifndef ROVI_POLICY_POLICY_LANE_HPP_
#define ROVI_POLICY_POLICY_LANE_HPP_

#include <cstdint>
#include <map>
#include <optional>

#include "rovi_core/units.hpp"
#include "rovi_policy/wire.hpp"

namespace rovi::policy {

struct LaneJob {
  core::JobId job;
  core::RobotId robot;
  core::TimePoint planned_start;
  core::TimePoint planned_end;
  core::Bytes chunk_bytes;

  bool operator==(const LaneJob&) const = default;
};

struct LaneTiming {
  core::Duration orphan_timeout;
  core::Duration max_lead;
};

struct Started {
  LaneJob job;
  core::TimePoint observation_time;
  Observation observation;
};

class PolicyLane {
 public:
  [[nodiscard]] static std::optional<PolicyLane> create(LaneTiming timing);

  bool assign(const LaneJob& job, core::TimePoint now);
  bool arrive(core::JobId job, core::TimePoint at, core::TimePoint observation_time,
              Observation observation);
  // Drops orphans, and ready jobs whose reserved compute already ended.
  void prune(core::TimePoint now);
  [[nodiscard]] std::optional<Started> start(core::TimePoint now);
  bool finish(core::JobId job);

  [[nodiscard]] bool busy() const;
  [[nodiscard]] std::uint64_t discarded() const;

 private:
  struct Arrival {
    core::TimePoint at;
    core::TimePoint observation_time;
    Observation observation;
  };

  explicit PolicyLane(LaneTiming timing);

  LaneTiming timing_;
  std::map<core::JobId, LaneJob> assigned_;
  std::map<core::JobId, Arrival> arrived_;
  std::optional<core::JobId> running_;
  std::uint64_t discarded_{0};
};

}  // namespace rovi::policy

#endif  // ROVI_POLICY_POLICY_LANE_HPP_
