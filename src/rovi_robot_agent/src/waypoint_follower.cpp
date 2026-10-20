#include "rovi_robot_agent/waypoint_follower.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <optional>
#include <utility>

namespace rovi::robot_agent {
namespace {

constexpr double kFullTurn = 2.0 * std::numbers::pi;
constexpr double kQuaternionScale = 2.0;

double wrapped(double angle) { return std::remainder(angle, kFullTurn); }

bool positive_finite(double value) { return std::isfinite(value) && value > 0.0; }

bool non_negative_finite(double value) { return std::isfinite(value) && value >= 0.0; }

}  // namespace

double yaw_from_quaternion(double x, double y, double z, double w) {
  return std::atan2(kQuaternionScale * ((w * z) + (x * y)),
                    1.0 - (kQuaternionScale * ((y * y) + (z * z))));
}

std::optional<Pose2D> planar_pose(double x, double y, double qx, double qy, double qz, double qw) {
  const bool finite = std::isfinite(x) && std::isfinite(y) && std::isfinite(qx) &&
                      std::isfinite(qy) && std::isfinite(qz) && std::isfinite(qw);
  if (!finite) {
    return std::nullopt;
  }
  return Pose2D{.x = x, .y = y, .yaw = yaw_from_quaternion(qx, qy, qz, qw)};
}

WaypointFollower::WaypointFollower(FollowerConfig config) : config_{std::move(config)} {}

std::optional<WaypointFollower> WaypointFollower::create(FollowerConfig config) {
  const bool finite_waypoints = std::ranges::all_of(config.waypoints, [](const Point2D& point) {
    return std::isfinite(point.x) && std::isfinite(point.y);
  });
  if (config.waypoints.empty() || !finite_waypoints || !positive_finite(config.arrival_tolerance) ||
      !non_negative_finite(config.max_linear) || !non_negative_finite(config.max_angular) ||
      !positive_finite(config.heading_gain)) {
    return std::nullopt;
  }
  return WaypointFollower{std::move(config)};
}

Velocity WaypointFollower::command(const Pose2D& pose) {
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw)) {
    return Velocity{};
  }
  const auto distance_to = [&pose](const Point2D& point) {
    return std::hypot(point.x - pose.x, point.y - pose.y);
  };
  for (std::size_t checked = 0;
       distance_to(config_.waypoints.at(target_)) <= config_.arrival_tolerance; ++checked) {
    if (checked == config_.waypoints.size()) {
      return Velocity{};
    }
    target_ = (target_ + 1) % config_.waypoints.size();
  }
  const Point2D& goal = config_.waypoints.at(target_);
  const double error = wrapped(std::atan2(goal.y - pose.y, goal.x - pose.x) - pose.yaw);
  const double angular =
      std::clamp(config_.heading_gain * error, -config_.max_angular, config_.max_angular);
  const double alignment = std::cos(error);
  return Velocity{.linear = alignment > 0.0 ? config_.max_linear * alignment : 0.0,
                  .angular = angular};
}

std::size_t WaypointFollower::target() const { return target_; }

}  // namespace rovi::robot_agent
