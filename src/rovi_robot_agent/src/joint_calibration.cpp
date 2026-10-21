#include "rovi_robot_agent/joint_calibration.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace rovi::robot_agent {
namespace {

constexpr double kRadiansPerDegree = std::numbers::pi / 180.0;

bool valid(const JointCalibration& joint) {
  return (joint.sign == 1.0 || joint.sign == -1.0) && std::isfinite(joint.zero_deg) &&
         std::isfinite(joint.lower_rad) && std::isfinite(joint.upper_rad) &&
         joint.lower_rad < joint.upper_rad;
}

}  // namespace

CalibrationMap::CalibrationMap(std::vector<JointCalibration> joints) : joints_{std::move(joints)} {}

std::optional<CalibrationMap> CalibrationMap::create(std::vector<JointCalibration> joints) {
  if (joints.empty() || !std::ranges::all_of(joints, valid)) {
    return std::nullopt;
  }
  return CalibrationMap{std::move(joints)};
}

std::optional<MappedPose> CalibrationMap::to_radians(std::span<const float> values) const {
  if (values.size() != joints_.size() ||
      !std::ranges::all_of(values, [](float value) { return std::isfinite(value); })) {
    return std::nullopt;
  }
  MappedPose pose{.radians = std::vector<double>(values.size()), .clamped = false};
  std::ranges::transform(
      values, joints_, pose.radians.begin(), [&pose](float value, const JointCalibration& joint) {
        const double angle =
            joint.sign * (static_cast<double>(value) - joint.zero_deg) * kRadiansPerDegree;
        const double limited = std::clamp(angle, joint.lower_rad, joint.upper_rad);
        pose.clamped = pose.clamped || limited != angle;
        return limited;
      });
  return pose;
}

std::size_t CalibrationMap::size() const { return joints_.size(); }

}  // namespace rovi::robot_agent
