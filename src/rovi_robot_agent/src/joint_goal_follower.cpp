#include "rovi_robot_agent/joint_goal_follower.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "rovi_core/units.hpp"

namespace rovi::robot_agent {
namespace {

constexpr double kNanosecondsPerSecond = 1e9;

bool all_finite(const std::vector<double>& values) {
  return std::ranges::all_of(values, [](double value) { return std::isfinite(value); });
}

bool positive_finite(double value) { return std::isfinite(value) && value > 0.0; }

bool valid_limits(const JointFollowerConfig& config, std::size_t joints) {
  if (config.lower.size() != joints || config.upper.size() != joints || !all_finite(config.lower) ||
      !all_finite(config.upper)) {
    return false;
  }
  for (std::size_t joint = 0; joint < joints; ++joint) {
    if (config.lower.at(joint) >= config.upper.at(joint)) {
      return false;
    }
  }
  return true;
}

bool within_limits(const JointFollowerConfig& config, const std::vector<double>& goal) {
  for (std::size_t joint = 0; joint < goal.size(); ++joint) {
    if (goal.at(joint) < config.lower.at(joint) || goal.at(joint) > config.upper.at(joint)) {
      return false;
    }
  }
  return true;
}

}  // namespace

JointGoalFollower::JointGoalFollower(JointFollowerConfig config) : config_{std::move(config)} {}

std::optional<JointGoalFollower> JointGoalFollower::create(JointFollowerConfig config) {
  if (config.goals.empty() || config.goals.front().empty() || !positive_finite(config.max_speed) ||
      !positive_finite(config.tolerance) || config.settle_time <= core::Duration{}) {
    return std::nullopt;
  }
  const std::size_t joints = config.goals.front().size();
  if (!valid_limits(config, joints)) {
    return std::nullopt;
  }
  const bool consistent = std::ranges::all_of(config.goals, [&config, joints](const auto& goal) {
    return goal.size() == joints && all_finite(goal) && within_limits(config, goal);
  });
  if (!consistent) {
    return std::nullopt;
  }
  return JointGoalFollower{std::move(config)};
}

std::optional<std::vector<double>> JointGoalFollower::step(const std::vector<double>& measured,
                                                           core::Duration dt) {
  if (measured.size() != joint_count() || !all_finite(measured) || dt <= core::Duration{}) {
    return std::nullopt;
  }
  if (!reference_) {
    reference_ = clamped(measured);
  }
  std::vector<double>& reference = *reference_;
  const std::vector<double>& current = config_.goals.at(goal_);
  const bool arrived = std::ranges::equal(reference, current);
  const bool settled = std::ranges::equal(measured, current, [this](double actual, double target) {
    return std::abs(actual - target) <= config_.tolerance;
  });
  if (arrived && (settled || held_ >= config_.settle_time)) {
    goal_ = (goal_ + 1) % config_.goals.size();
    cycles_ += goal_ == 0 ? 1U : 0U;
    held_ = core::Duration{};
  } else if (arrived) {
    held_ = dt.ns() >= config_.settle_time.ns() - held_.ns()
                ? config_.settle_time
                : core::Duration::from_ns(held_.ns() + dt.ns());
  }
  const std::vector<double>& next = config_.goals.at(goal_);
  const double limit = config_.max_speed * static_cast<double>(dt.ns()) / kNanosecondsPerSecond;
  for (std::size_t joint = 0; joint < reference.size(); ++joint) {
    const double remaining = next.at(joint) - reference.at(joint);
    reference.at(joint) = std::abs(remaining) <= limit
                              ? next.at(joint)
                              : reference.at(joint) + std::copysign(limit, remaining);
  }
  return reference;
}

std::optional<std::vector<double>> JointGoalFollower::stop(const std::vector<double>& measured) {
  if (measured.size() != joint_count() || !all_finite(measured)) {
    return std::nullopt;
  }
  reference_.reset();
  held_ = core::Duration{};
  return clamped(measured);
}

std::vector<double> JointGoalFollower::clamped(std::vector<double> positions) const {
  for (std::size_t joint = 0; joint < positions.size(); ++joint) {
    positions.at(joint) =
        std::clamp(positions.at(joint), config_.lower.at(joint), config_.upper.at(joint));
  }
  return positions;
}

std::size_t JointGoalFollower::goal() const { return goal_; }

std::uint64_t JointGoalFollower::cycles() const { return cycles_; }

std::size_t JointGoalFollower::joint_count() const { return config_.goals.front().size(); }

}  // namespace rovi::robot_agent
