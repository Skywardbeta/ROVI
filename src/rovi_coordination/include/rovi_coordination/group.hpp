// Workcell membership and assembled cooperative groups shared by the coordinator's logic.
#ifndef ROVI_COORDINATION_GROUP_HPP_
#define ROVI_COORDINATION_GROUP_HPP_

#include <cstdint>
#include <vector>

#include "rovi_core/job.hpp"
#include "rovi_core/units.hpp"

namespace rovi::coordination {

struct Workcell {
  std::uint32_t id{0};
  std::vector<core::RobotId> members;
  core::Duration skew;
};

struct AssembledGroup {
  std::uint32_t workcell{0};
  core::GroupId group;
  std::vector<core::Job> members;

  bool operator==(const AssembledGroup&) const = default;
};

}  // namespace rovi::coordination

#endif  // ROVI_COORDINATION_GROUP_HPP_
