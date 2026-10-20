// Collects one job proposal per workcell member and emits each complete round as a group.
#ifndef ROVI_COORDINATION_GROUP_ASSEMBLER_HPP_
#define ROVI_COORDINATION_GROUP_ASSEMBLER_HPP_

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <vector>

#include "rovi_coordination/group.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/units.hpp"

namespace rovi::coordination {

inline constexpr std::size_t kMaximumWorkcellSize = 64;

enum class ProposalStatus : std::uint8_t {
  kHeld,
  kAssembled,
  kUnknownRobot,
  kAlreadyGrouped,
  kMalformed
};

struct ProposalOutcome {
  ProposalStatus status{ProposalStatus::kHeld};
  std::optional<AssembledGroup> group;

  bool operator==(const ProposalOutcome&) const = default;
};

class GroupAssembler {
 public:
  [[nodiscard]] static std::optional<GroupAssembler> create(std::vector<Workcell> workcells);

  [[nodiscard]] ProposalOutcome propose(const core::Job& job);
  [[nodiscard]] std::vector<AssembledGroup> expire(core::TimePoint now);

 private:
  explicit GroupAssembler(std::vector<Workcell> workcells);

  std::vector<Workcell> workcells_;
  std::map<std::uint32_t, std::map<core::RobotId, core::Job>> rounds_;
  std::uint32_t next_group_{1};
};

}  // namespace rovi::coordination

#endif  // ROVI_COORDINATION_GROUP_ASSEMBLER_HPP_
