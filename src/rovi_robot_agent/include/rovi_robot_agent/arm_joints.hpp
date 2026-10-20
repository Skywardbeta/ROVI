// Pure helpers that order reported joint positions and split flat joint-goal parameters.
#ifndef ROVI_ROBOT_AGENT_ARM_JOINTS_HPP_
#define ROVI_ROBOT_AGENT_ARM_JOINTS_HPP_

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace rovi::robot_agent {

[[nodiscard]] std::optional<std::vector<double>> ordered_positions(
    const std::vector<std::string>& names, const std::vector<double>& positions,
    const std::vector<std::string>& joints);

[[nodiscard]] std::optional<std::vector<std::vector<double>>> split_goals(
    const std::vector<double>& flat, std::size_t joints);

[[nodiscard]] bool valid_joint_names(const std::vector<std::string>& joints);

}  // namespace rovi::robot_agent

#endif  // ROVI_ROBOT_AGENT_ARM_JOINTS_HPP_
