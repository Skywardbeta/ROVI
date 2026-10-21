#include "rovi_robot_agent/chunk_player.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "rovi_core/units.hpp"

namespace rovi::robot_agent {
namespace {

core::TimePoint at_ms(std::int64_t ms) { return core::TimePoint::from_ns(ms * 1'000'000); }

// Two steps of one joint 100 ms apart, whose values name the chunk and the step.
PlayedChunk chunk(std::int64_t start_ms, float base) {
  return PlayedChunk{.start = at_ms(start_ms),
                     .period = core::Duration::from_ns(100'000'000),
                     .steps = 2,
                     .dim = 1,
                     .actions = {base, base + 1.0F}};
}

std::optional<float> played(ChunkPlayer& player, std::int64_t ms) {
  const std::optional<std::vector<float>> action = player.action_at(at_ms(ms));
  return action ? std::optional{action->front()} : std::nullopt;
}

TEST(ChunkPlayer, HoldsBeforeTheFirstChunkStartsAndAfterItEnds) {
  ChunkPlayer player{1};
  EXPECT_FALSE(played(player, 0).has_value());
  ASSERT_TRUE(player.add(chunk(1'000, 10.0F)));
  EXPECT_FALSE(played(player, 999).has_value());
  EXPECT_EQ(played(player, 1'000), 10.0F);
  EXPECT_EQ(played(player, 1'099), 10.0F);
  EXPECT_EQ(played(player, 1'100), 11.0F);
  EXPECT_EQ(played(player, 1'199), 11.0F);
  EXPECT_FALSE(played(player, 1'200).has_value());
}

TEST(ChunkPlayer, StartsAQueuedChunkExactlyWhenTheChunkPlayingEnds) {
  ChunkPlayer player{1};
  ASSERT_TRUE(player.add(chunk(1'000, 10.0F)));
  ASSERT_TRUE(player.add(chunk(1'200, 20.0F)));
  EXPECT_EQ(played(player, 1'199), 11.0F);
  EXPECT_EQ(played(player, 1'200), 20.0F);
  EXPECT_EQ(played(player, 1'399), 21.0F);
  EXPECT_FALSE(played(player, 1'400).has_value());
}

TEST(ChunkPlayer, LetsANewerChunkSupersedeTheRestOfAnOlderOneFromItsStart) {
  ChunkPlayer player{1};
  ASSERT_TRUE(player.add(chunk(1'000, 10.0F)));
  ASSERT_TRUE(player.add(chunk(1'050, 20.0F)));
  EXPECT_EQ(played(player, 1'049), 10.0F);
  EXPECT_EQ(played(player, 1'050), 20.0F);
  EXPECT_EQ(played(player, 1'150), 21.0F);
  EXPECT_FALSE(played(player, 1'250).has_value());
}

TEST(ChunkPlayer, RefusesAChunkThatStartsBeforeOneItHolds) {
  ChunkPlayer player{1};
  ASSERT_TRUE(player.add(chunk(1'200, 20.0F)));
  EXPECT_FALSE(player.add(chunk(1'000, 10.0F)));
  EXPECT_FALSE(played(player, 1'100).has_value());
  EXPECT_EQ(played(player, 1'200), 20.0F);
}

TEST(ChunkPlayer, LetsANewerChunkWithTheSameStartReplaceTheOneHeld) {
  ChunkPlayer player{1};
  ASSERT_TRUE(player.add(chunk(1'000, 10.0F)));
  ASSERT_TRUE(player.add(chunk(1'000, 20.0F)));
  EXPECT_EQ(played(player, 1'000), 20.0F);
}

// As the robot agent's buffer ends at the newest chunk's end, a newer chunk that has already ended
// leaves nothing to play.
TEST(ChunkPlayer, HoldsOnceANewerChunkHasEndedEvenIfAnOlderOneHasNot) {
  ChunkPlayer player{1};
  ASSERT_TRUE(player.add(chunk(1'000, 10.0F)));
  ASSERT_TRUE(player.add(PlayedChunk{.start = at_ms(1'010),
                                     .period = core::Duration::from_ns(10'000'000),
                                     .steps = 1,
                                     .dim = 1,
                                     .actions = {30.0F}}));
  EXPECT_EQ(played(player, 1'010), 30.0F);
  EXPECT_FALSE(played(player, 1'100).has_value());
}

TEST(ChunkPlayer, RefusesMalformedChunks) {
  ChunkPlayer player{1};
  PlayedChunk wide = chunk(0, 1.0F);
  wide.dim = 2;
  PlayedChunk short_actions = chunk(0, 1.0F);
  short_actions.actions.pop_back();
  PlayedChunk no_period = chunk(0, 1.0F);
  no_period.period = core::Duration{};
  PlayedChunk endless = chunk(0, 1.0F);
  endless.start = core::TimePoint::from_ns(std::numeric_limits<std::int64_t>::max() - 1);
  for (const PlayedChunk& bad : {wide, short_actions, no_period, endless}) {
    EXPECT_FALSE(player.add(bad));
  }
  EXPECT_FALSE(played(player, 0).has_value());
}

TEST(ChunkPlayer, BoundsTheChunksWaitingToStart) {
  ChunkPlayer player{1};
  std::int64_t start = 1'000;
  for (std::size_t queued = 0; queued < kMaximumQueuedChunks; ++queued, start += 200) {
    ASSERT_TRUE(player.add(chunk(start, 1.0F)));
  }
  EXPECT_FALSE(player.add(chunk(start, 1.0F)));
}

}  // namespace
}  // namespace rovi::robot_agent
