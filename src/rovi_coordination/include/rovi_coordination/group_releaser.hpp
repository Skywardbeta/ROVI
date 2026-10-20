// Releases a group's action chunks together once all arrive within its skew bound, or refuses them.
#ifndef ROVI_COORDINATION_GROUP_RELEASER_HPP_
#define ROVI_COORDINATION_GROUP_RELEASER_HPP_

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <vector>

#include "rovi_coordination/group.hpp"
#include "rovi_core/admission.hpp"
#include "rovi_core/units.hpp"

namespace rovi::coordination {

struct ReleaseOutcome {
  std::uint32_t workcell{0};
  core::GroupId group;
  bool released{false};
  core::ReasonCode reason{core::ReasonCode::kNone};
  core::TimePoint release_time;
  core::Duration skew;
  std::vector<core::JobId> jobs;
  std::vector<core::RobotId> robots;

  bool operator==(const ReleaseOutcome&) const = default;
};

class GroupReleaser {
 public:
  GroupReleaser(std::size_t max_tracked, core::Duration grace);

  [[nodiscard]] bool track(const AssembledGroup& group);
  void on_decision(core::JobId job, core::DecisionStatus status, core::ReasonCode reason);
  [[nodiscard]] std::optional<ReleaseOutcome> on_delivery(core::JobId job,
                                                          core::TimePoint delivered);
  [[nodiscard]] std::vector<ReleaseOutcome> expire(core::TimePoint now);
  [[nodiscard]] std::size_t tracked() const;

 private:
  struct Tracked {
    AssembledGroup group;
    std::map<core::JobId, core::TimePoint> delivered;
  };

  std::size_t max_tracked_;
  core::Duration grace_;
  std::vector<Tracked> groups_;
};

}  // namespace rovi::coordination

#endif  // ROVI_COORDINATION_GROUP_RELEASER_HPP_
