#include "rovi_robot_agent/chunk_player.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <optional>
#include <ranges>
#include <utility>
#include <vector>

#include "rovi_core/units.hpp"

namespace rovi::robot_agent {
namespace {

std::optional<core::TimePoint> end_of(const PlayedChunk& chunk) {
  std::int64_t length = 0;
  if (__builtin_mul_overflow(std::int64_t{chunk.steps}, chunk.period.ns(), &length)) {
    return std::nullopt;
  }
  return core::checked_add(chunk.start, core::Duration::from_ns(length));
}

}  // namespace

ChunkPlayer::ChunkPlayer(std::uint32_t dim) : dim_{dim} {}

bool ChunkPlayer::add(PlayedChunk chunk) {
  const bool malformed = chunk.dim != dim_ || chunk.steps == 0 || chunk.period.ns() <= 0 ||
                         chunk.actions.size() != std::size_t{chunk.steps} * chunk.dim ||
                         !end_of(chunk);
  const bool earlier = !chunks_.empty() && chunk.start < chunks_.back().start;
  if (malformed || earlier || chunks_.size() >= kMaximumQueuedChunks) {
    return false;
  }
  chunks_.push_back(std::move(chunk));
  return true;
}

std::optional<std::vector<float>> ChunkPlayer::action_at(core::TimePoint now) {
  const auto newest = std::ranges::find_if(
      std::views::reverse(chunks_), [now](const PlayedChunk& chunk) { return chunk.start <= now; });
  if (newest == chunks_.rend()) {
    return std::nullopt;
  }
  chunks_.erase(chunks_.begin(), std::prev(newest.base()));
  const PlayedChunk& playing = chunks_.front();
  const std::optional<core::TimePoint> end = end_of(playing);
  if (!end || now >= *end) {
    return std::nullopt;
  }
  const std::int64_t step = (now.ns() - playing.start.ns()) / playing.period.ns();
  const auto first = std::next(playing.actions.begin(), step * std::int64_t{playing.dim});
  return std::vector<float>(first, std::next(first, playing.dim));
}

}  // namespace rovi::robot_agent
