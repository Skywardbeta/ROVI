// Plays accepted action chunks on the robot agent's timeline: step k applies at start + k * period,
// and a newer chunk supersedes older ones from its own start.
#ifndef ROVI_ROBOT_AGENT_CHUNK_PLAYER_HPP_
#define ROVI_ROBOT_AGENT_CHUNK_PLAYER_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

#include "rovi_core/units.hpp"

namespace rovi::robot_agent {

inline constexpr std::size_t kMaximumQueuedChunks = 8;

struct PlayedChunk {
  core::TimePoint start;
  core::Duration period;
  std::uint32_t steps{0};
  std::uint32_t dim{0};
  std::vector<float> actions;
};

class ChunkPlayer {
 public:
  explicit ChunkPlayer(std::uint32_t dim);

  // Refuses a malformed chunk, one starting before a chunk already held, or any chunk while
  // kMaximumQueuedChunks are held.
  bool add(PlayedChunk chunk);
  // The action due at now, or nullopt when no chunk covers it; forgets superseded chunks.
  [[nodiscard]] std::optional<std::vector<float>> action_at(core::TimePoint now);

 private:
  std::uint32_t dim_;
  std::deque<PlayedChunk> chunks_;
};

}  // namespace rovi::robot_agent

#endif  // ROVI_ROBOT_AGENT_CHUNK_PLAYER_HPP_
