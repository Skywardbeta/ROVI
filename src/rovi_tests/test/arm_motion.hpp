// Joint-space motion measures shared by the Gazebo arm smoke tests.
#ifndef ROVI_TESTS_ARM_MOTION_HPP_
#define ROVI_TESTS_ARM_MOTION_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "gazebo_fixture.hpp"

namespace rovi::tests {

struct ArmSample {
  std::int64_t stamp_ns{0};
  std::vector<double> positions;
};

inline std::vector<ArmSample> arm_samples(const std::vector<JointSample>& joint_samples,
                                          std::size_t topic) {
  std::vector<ArmSample> samples;
  for (const JointSample& joint_sample : joint_samples) {
    const sensor_msgs::msg::JointState& state = joint_sample.state;
    if (joint_sample.topic != topic || state.position.size() != state.name.size()) {
      continue;
    }
    ArmSample sample{.stamp_ns = rclcpp::Time(state.header.stamp).nanoseconds(), .positions = {}};
    for (int joint = 1; joint <= kArmJoints; ++joint) {
      const auto found = std::ranges::find(state.name, "panda_joint" + std::to_string(joint));
      if (found == state.name.end()) {
        break;
      }
      sample.positions.push_back(
          state.position.at(static_cast<std::size_t>(std::distance(state.name.begin(), found))));
    }
    if (sample.positions.size() == static_cast<std::size_t>(kArmJoints)) {
      samples.push_back(sample);
    }
  }
  return samples;
}

inline double largest_change(const ArmSample& from, const ArmSample& to) {
  double change = 0.0;
  for (std::size_t joint = 0; joint < from.positions.size(); ++joint) {
    change = std::max(change, std::abs(to.positions.at(joint) - from.positions.at(joint)));
  }
  return change;
}

inline double travel_between(const std::vector<ArmSample>& samples, std::int64_t from_ns,
                             std::int64_t to_ns) {
  double travel = 0.0;
  for (std::size_t later = 1; later < samples.size(); ++later) {
    const ArmSample& previous = samples.at(later - 1);
    const ArmSample& current = samples.at(later);
    if (previous.stamp_ns >= from_ns && current.stamp_ns <= to_ns) {
      travel += largest_change(previous, current);
    }
  }
  return travel;
}

inline std::optional<double> drift_between(const std::vector<ArmSample>& samples,
                                           std::int64_t from_ns, std::int64_t to_ns) {
  const auto first = std::ranges::find_if(
      samples, [from_ns](const ArmSample& sample) { return sample.stamp_ns >= from_ns; });
  double drift = 0.0;
  std::size_t counted = 0;
  for (auto sample = first; sample != samples.end() && sample->stamp_ns <= to_ns; ++sample) {
    drift = std::max(drift, largest_change(*first, *sample));
    ++counted;
  }
  return counted >= 2 ? std::optional{drift} : std::nullopt;
}

}  // namespace rovi::tests

#endif  // ROVI_TESTS_ARM_MOTION_HPP_
