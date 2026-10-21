#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <sensor_msgs/msg/joint_state.hpp>

#include "rovi_interfaces/msg/action_chunk.hpp"
#include "rovi_interfaces/msg/robot_state.hpp"

#include "gazebo_fixture.hpp"
#include "so101_playback.hpp"
#include "stub_server.hpp"

namespace rovi::tests {
namespace {

using rovi_interfaces::msg::ActionChunk;
using rovi_interfaces::msg::RobotState;

constexpr double kStill = 0.005;
constexpr std::int64_t kHoldMarginNs = 200'000'000;

TEST_F(So101Playback, HoldsTheMeasuredPoseOnceTheLastChunkEndsAndStopsWhenTheRunIsInvalid) {
  StubServer server{"so101_expiry", 0};
  ASSERT_NE(server.port(), 0) << "the stub policy server did not start";
  GazeboScenario scenario{so101_scenario_with(server)};
  ASSERT_TRUE(scenario.run_until(6 * kNsPerSecond, kSo101WallLimit)) << scenario.failure();
  server.stop();
  // Replies received before the stop are delivered at the end of their reserved compute slot.
  ASSERT_TRUE(scenario.run_until(9 * kNsPerSecond, kSo101WallLimit)) << scenario.failure();
  ASSERT_FALSE(scenario.action_chunks().empty());
  const std::int64_t coverage_end = end_of(scenario.action_chunks().back());
  const std::int64_t invalidated_ns = std::max(coverage_end, 9 * kNsPerSecond) + (2 * kNsPerSecond);

  ASSERT_TRUE(scenario.run_until(invalidated_ns, kSo101WallLimit)) << scenario.failure();

  const ActionChunk& last_chunk = scenario.action_chunks().back();
  EXPECT_EQ(end_of(last_chunk), coverage_end) << "a chunk arrived after the server stopped";
  expect_commands_follow_the_chunks(scenario);
  const auto fallback = std::ranges::find_if(scenario.states(), [&](const RobotState& state) {
    return state.stamp_ns >= coverage_end && state.mode == RobotState::MODE_FALLBACK;
  });
  ASSERT_NE(fallback, scenario.states().end()) << "the robot never fell back";
  for (std::size_t joint = 0; joint < so101_joints().size(); ++joint) {
    std::vector<double> held;
    for (const CommandSample& command : scenario.commands()) {
      if (command.topic == joint && command.received_ns >= coverage_end + kLockStepNs) {
        held.push_back(command.value);
      }
    }
    ASSERT_FALSE(held.empty()) << so101_joints().at(joint);
    EXPECT_TRUE(std::ranges::all_of(held, [&held](double value) { return value == held.front(); }))
        << so101_joints().at(joint) << " moved after the last chunk ended";
    const double final_action = radians_of(last_chunk.actions.at(
        (static_cast<std::size_t>(last_chunk.steps - 1) * last_chunk.dim) + joint));
    EXPECT_NEAR(held.front(), final_action, kTracking) << so101_joints().at(joint);
  }

  const std::size_t commands_before = scenario.commands().size();
  scenario.invalidate_run();
  ASSERT_TRUE(scenario.run_until(invalidated_ns + kNsPerSecond, kSo101WallLimit))
      << scenario.failure();
  for (std::size_t later = commands_before; later < scenario.commands().size(); ++later) {
    EXPECT_LE(scenario.commands().at(later).received_ns, invalidated_ns + kLockStepNs)
        << "a joint was commanded after the run became invalid";
  }
  std::optional<sensor_msgs::msg::JointState> settled;
  for (const JointSample& sample : scenario.joint_samples()) {
    if (stamp_of(sample) < coverage_end + kHoldMarginNs) {
      continue;
    }
    if (!settled) {
      settled = sample.state;
    }
    for (const std::string& joint : so101_joints()) {
      const std::optional<double> start = position_of(*settled, joint);
      const std::optional<double> now = position_of(sample.state, joint);
      ASSERT_TRUE(start && now) << joint;
      EXPECT_NEAR(*now, *start, kStill) << joint << " drifted while holding";
    }
  }
  EXPECT_TRUE(settled.has_value());
}

}  // namespace
}  // namespace rovi::tests
