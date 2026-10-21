// Maps policy joint values onto simulator joints: radians = sign * (value - zero_deg) * pi / 180,
// clamped to the joint's limits. The policy's gripper 0-100 is treated as degrees, as LeRobot does.
#ifndef ROVI_ROBOT_AGENT_JOINT_CALIBRATION_HPP_
#define ROVI_ROBOT_AGENT_JOINT_CALIBRATION_HPP_

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

namespace rovi::robot_agent {

struct JointCalibration {
  double sign{1.0};
  double zero_deg{0.0};
  double lower_rad{0.0};
  double upper_rad{0.0};
};

struct MappedPose {
  std::vector<double> radians;
  bool clamped{false};
};

class CalibrationMap {
 public:
  [[nodiscard]] static std::optional<CalibrationMap> create(std::vector<JointCalibration> joints);

  [[nodiscard]] std::optional<MappedPose> to_radians(std::span<const float> values) const;
  [[nodiscard]] std::size_t size() const;

 private:
  explicit CalibrationMap(std::vector<JointCalibration> joints);

  std::vector<JointCalibration> joints_;
};

}  // namespace rovi::robot_agent

#endif  // ROVI_ROBOT_AGENT_JOINT_CALIBRATION_HPP_
