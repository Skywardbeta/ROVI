#include "so101_playback.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "rovi_interfaces/msg/action_chunk.hpp"
#include "rovi_interfaces/msg/robot_state.hpp"

#include "gazebo_fixture.hpp"
#include "stub_server.hpp"

namespace rovi::tests {
namespace {

using rovi_interfaces::msg::ActionChunk;
using rovi_interfaces::msg::RobotState;

constexpr std::int64_t kSettleNs = 1'500'000'000;

TEST_F(So101Playback, PlaysEachChunkOntoTheMappedJointsBackToBack) {
  const StubServer server{"so101_playback", 0};
  ASSERT_NE(server.port(), 0) << "the stub policy server did not start";
  GazeboScenario scenario{so101_scenario_with(server)};

  ASSERT_TRUE(scenario.run_until(9 * kNsPerSecond, kSo101WallLimit)) << scenario.failure();

  const std::vector<ActionChunk>& chunks = scenario.action_chunks();
  ASSERT_GE(chunks.size(), 3U);
  for (std::size_t later = 1; later < chunks.size(); ++later) {
    EXPECT_EQ(chunks.at(later).start_ns, end_of(chunks.at(later - 1)))
        << "chunk " << later << " does not follow the previous one";
  }
  const std::int64_t first_start = chunks.front().start_ns;
  for (const CommandSample& command : scenario.commands()) {
    EXPECT_GE(command.received_ns, first_start) << "a joint moved before the first chunk started";
  }
  expect_commands_follow_the_chunks(scenario);

  ASSERT_FALSE(scenario.joint_samples().empty());
  // The Foxglove layout reads joint states by index.
  EXPECT_EQ(scenario.joint_samples().front().state.name, so101_joints());
  std::size_t tracked = 0;
  for (const JointSample& sample : scenario.joint_samples()) {
    const std::int64_t stamp = stamp_of(sample);
    if (stamp < first_start + kSettleNs) {
      continue;
    }
    for (std::size_t joint = 0; joint < so101_joints().size(); ++joint) {
      const std::optional<double> target = due(chunks, joint, stamp);
      const std::optional<double> measured = position_of(sample.state, so101_joints().at(joint));
      ASSERT_TRUE(target && measured) << so101_joints().at(joint) << " at " << stamp;
      EXPECT_NEAR(*measured, *target, kTracking) << so101_joints().at(joint) << " at " << stamp;
    }
    ++tracked;
  }
  EXPECT_GT(tracked, 0U);

  ASSERT_FALSE(scenario.states().empty());
  const RobotState& last = scenario.states().back();
  EXPECT_EQ(last.mode, RobotState::MODE_EXECUTING);
  EXPECT_EQ(last.fallback_entries, 0U);
  EXPECT_EQ(last.chunks_invalid, 0U);
}

}  // namespace
}  // namespace rovi::tests
