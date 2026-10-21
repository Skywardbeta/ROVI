#include "rovi_robot_agent/chunk_commander.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "rovi_core/units.hpp"
#include "rovi_robot_agent/arm_joints.hpp"
#include "rovi_robot_agent/chunk_player.hpp"
#include "rovi_robot_agent/joint_calibration.hpp"
#include "rovi_robot_agent/motion_monitor.hpp"

namespace rovi::robot_agent {

ChunkCommander::ChunkCommander(std::vector<std::string> joints, MotionMonitor monitor,
                               CalibrationMap calibration)
    : joints_{std::move(joints)},
      monitor_{monitor},
      calibration_{std::move(calibration)},
      player_{static_cast<std::uint32_t>(calibration_.size())} {}

void ChunkCommander::on_state(const ReportedState& state, core::TimePoint now) {
  monitor_.on_state(state, now);
}

void ChunkCommander::on_joint_state(const std::vector<std::string>& names,
                                    const std::vector<double>& positions, core::TimePoint now) {
  std::optional<std::vector<double>> ordered = ordered_positions(names, positions, joints_);
  if (!ordered) {
    return;
  }
  measured_ = std::move(ordered);
  monitor_.on_feedback(now);
}

bool ChunkCommander::on_chunk(PlayedChunk chunk) { return player_.add(std::move(chunk)); }

std::optional<std::vector<double>> ChunkCommander::command(core::TimePoint now) {
  const std::optional<std::vector<float>> action = player_.action_at(now);
  std::optional<MappedPose> target =
      action && monitor_.may_move(now) ? calibration_.to_radians(*action) : std::nullopt;
  if (target) {
    clamped_commands_ += target->clamped ? 1U : 0U;
    last_command_ = std::move(target->radians);
    playing_ = true;
  } else if (playing_ && measured_) {
    last_command_ = measured_;
    playing_ = false;
  }
  return last_command_;
}

bool ChunkCommander::playing() const { return playing_; }

std::uint64_t ChunkCommander::clamped_commands() const { return clamped_commands_; }

}  // namespace rovi::robot_agent
