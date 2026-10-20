// Speed-limited joint-space reference that cycles through goals for a position-controlled arm.
#ifndef ROVI_ROBOT_AGENT_JOINT_GOAL_FOLLOWER_HPP_
#define ROVI_ROBOT_AGENT_JOINT_GOAL_FOLLOWER_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "rovi_core/units.hpp"

namespace rovi::robot_agent {

struct JointFollowerConfig {
  std::vector<std::vector<double>> goals;
  std::vector<double> lower;
  std::vector<double> upper;
  double max_speed{0.0};
  double tolerance{0.0};
  core::Duration settle_time;
};

class JointGoalFollower {
 public:
  [[nodiscard]] static std::optional<JointGoalFollower> create(JointFollowerConfig config);

  [[nodiscard]] std::optional<std::vector<double>> step(const std::vector<double>& measured,
                                                        core::Duration dt);
  [[nodiscard]] std::optional<std::vector<double>> stop(const std::vector<double>& measured);
  [[nodiscard]] std::size_t goal() const;
  [[nodiscard]] std::uint64_t cycles() const;
  [[nodiscard]] std::size_t joint_count() const;

 private:
  explicit JointGoalFollower(JointFollowerConfig config);
  [[nodiscard]] std::vector<double> clamped(std::vector<double> positions) const;

  JointFollowerConfig config_;
  std::size_t goal_{0};
  std::uint64_t cycles_{0};
  core::Duration held_;
  std::optional<std::vector<double>> reference_;
};

}  // namespace rovi::robot_agent

#endif  // ROVI_ROBOT_AGENT_JOINT_GOAL_FOLLOWER_HPP_
