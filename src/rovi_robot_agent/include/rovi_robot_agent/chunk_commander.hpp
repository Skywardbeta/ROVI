// Plays action chunks onto position-controlled joints while motion is permitted and a chunk covers
// the time, and otherwise holds the measured pose.
#ifndef ROVI_ROBOT_AGENT_CHUNK_COMMANDER_HPP_
#define ROVI_ROBOT_AGENT_CHUNK_COMMANDER_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "rovi_core/units.hpp"
#include "rovi_robot_agent/chunk_player.hpp"
#include "rovi_robot_agent/joint_calibration.hpp"
#include "rovi_robot_agent/motion_monitor.hpp"

namespace rovi::robot_agent {

class ChunkCommander {
 public:
  ChunkCommander(std::vector<std::string> joints, MotionMonitor monitor,
                 CalibrationMap calibration);

  void on_state(const ReportedState& state, core::TimePoint now);
  void on_joint_state(const std::vector<std::string>& names, const std::vector<double>& positions,
                      core::TimePoint now);
  bool on_chunk(PlayedChunk chunk);
  [[nodiscard]] std::optional<std::vector<double>> command(core::TimePoint now);
  [[nodiscard]] bool playing() const;
  // Commands whose mapped action fell outside the joint limits and was clamped.
  [[nodiscard]] std::uint64_t clamped_commands() const;

 private:
  std::vector<std::string> joints_;
  MotionMonitor monitor_;
  CalibrationMap calibration_;
  ChunkPlayer player_;
  std::optional<std::vector<double>> measured_;
  std::optional<std::vector<double>> last_command_;
  bool playing_{false};
  std::uint64_t clamped_commands_{0};
};

}  // namespace rovi::robot_agent

#endif  // ROVI_ROBOT_AGENT_CHUNK_COMMANDER_HPP_
