#include "rovi_robot_agent/arm_joints.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace rovi::robot_agent {
namespace {

bool name_start(char symbol) {
  return (symbol >= 'a' && symbol <= 'z') || (symbol >= 'A' && symbol <= 'Z') || symbol == '_';
}

bool name_symbol(char symbol) { return name_start(symbol) || (symbol >= '0' && symbol <= '9'); }

bool topic_token(const std::string& name) {
  return !name.empty() && name_start(name.front()) && std::ranges::all_of(name, name_symbol);
}

}  // namespace

std::optional<std::vector<double>> ordered_positions(const std::vector<std::string>& names,
                                                     const std::vector<double>& positions,
                                                     const std::vector<std::string>& joints) {
  if (names.size() != positions.size()) {
    return std::nullopt;
  }
  std::vector<double> ordered;
  ordered.reserve(joints.size());
  for (const std::string& joint : joints) {
    const auto found = std::ranges::find(names, joint);
    if (found == names.end()) {
      return std::nullopt;
    }
    const double position =
        positions.at(static_cast<std::size_t>(std::distance(names.begin(), found)));
    if (!std::isfinite(position)) {
      return std::nullopt;
    }
    ordered.push_back(position);
  }
  return ordered;
}

std::optional<std::vector<std::vector<double>>> split_goals(const std::vector<double>& flat,
                                                            std::size_t joints) {
  if (joints == 0 || flat.empty() || flat.size() % joints != 0) {
    return std::nullopt;
  }
  std::vector<std::vector<double>> goals;
  goals.reserve(flat.size() / joints);
  for (std::size_t first = 0; first < flat.size(); first += joints) {
    goals.emplace_back(std::next(flat.begin(), static_cast<std::ptrdiff_t>(first)),
                       std::next(flat.begin(), static_cast<std::ptrdiff_t>(first + joints)));
  }
  return goals;
}

bool valid_joint_names(const std::vector<std::string>& joints) {
  const std::set<std::string> distinct(joints.begin(), joints.end());
  return !joints.empty() && distinct.size() == joints.size() &&
         std::ranges::all_of(joints, topic_token);
}

}  // namespace rovi::robot_agent
