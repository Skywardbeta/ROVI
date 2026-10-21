// Shared setup and measures for the Gazebo SO-101 playback tests, one scenario per test process.
#ifndef ROVI_TESTS_SO101_PLAYBACK_HPP_
#define ROVI_TESTS_SO101_PLAYBACK_HPP_

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iterator>
#include <numbers>
#include <optional>
#include <ranges>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unistd.h>

#include "rovi_interfaces/msg/action_chunk.hpp"
#include "rovi_policy/wire.hpp"

#include "gazebo_fixture.hpp"
#include "stub_server.hpp"

namespace rovi::tests {

inline constexpr std::int64_t kNsPerSecond = 1'000'000'000;
inline constexpr std::int64_t kLockStepNs = 10'000'000;
inline constexpr double kRadiansPerDegree = std::numbers::pi / 180.0;
inline constexpr double kExactly = 1e-12;
inline constexpr double kTracking = 0.02;
inline constexpr std::chrono::seconds kSo101WallLimit{180};
// Distinct signs and sizes, so a swapped joint or a flipped direction cannot pass.
inline constexpr std::array<float, kSo101Joints> kRecordedDegrees{20.0F,  -30.0F, 40.0F,
                                                                  -25.0F, 60.0F,  30.0F};

class So101Playback : public ::testing::Test {
 protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

inline policy::Observation recorded_observation() {
  return policy::Observation{.state = {kRecordedDegrees.begin(), kRecordedDegrees.end()},
                             .width = 4,
                             .height = 2,
                             .rgb = policy::Bytes(24, 0)};
}

inline GazeboScenarioSpec so101_scenario_with(const StubServer& server) {
  const std::filesystem::path observation =
      std::filesystem::temp_directory_path() /
      ("rovi_so101_" + std::to_string(::getpid()) + ".rovobs");
  EXPECT_TRUE(policy::write_observation_file(
      observation, {.episode = 0, .frame = 0, .observation = recorded_observation()}));
  GazeboScenarioSpec spec = so101_scenario();
  spec.overrides = {
      {"rovi_policy_worker",
       {"server_host:=127.0.0.1", "server_port:=" + std::to_string(server.port()),
        "policy_id:=" + std::string{kStubPolicy}, "task:=test"}},
      {"robot_agent",
       {"observation_file:=" + observation.string(),
        "upload_bytes:=" + std::to_string(policy::request_bytes(recorded_observation())),
        "policy_id:=" + std::string{kStubPolicy}}}};
  return spec;
}

// The scenario's calibration: sign +1 and zero 0 for every joint.
inline double radians_of(float degrees) { return static_cast<double>(degrees) * kRadiansPerDegree; }

inline std::int64_t end_of(const rovi_interfaces::msg::ActionChunk& chunk) {
  return chunk.start_ns + (std::int64_t{chunk.steps} * chunk.period_ns);
}

// The commanded angle of a joint at t on the robot agent's timeline, if a chunk covers t.
inline std::optional<double> due(const std::vector<rovi_interfaces::msg::ActionChunk>& chunks,
                                 std::size_t joint, std::int64_t t) {
  const auto active = std::ranges::find_if(
      std::views::reverse(chunks),
      [t](const rovi_interfaces::msg::ActionChunk& chunk) { return chunk.start_ns <= t; });
  if (active == chunks.rend() || t >= end_of(*active)) {
    return std::nullopt;
  }
  const std::int64_t step = (t - active->start_ns) / active->period_ns;
  return radians_of(active->actions.at((static_cast<std::size_t>(step) * active->dim) + joint));
}

inline std::optional<double> position_of(const sensor_msgs::msg::JointState& state,
                                         const std::string& joint) {
  const auto found = std::ranges::find(state.name, joint);
  if (found == state.name.end() || state.position.size() != state.name.size()) {
    return std::nullopt;
  }
  return state.position.at(static_cast<std::size_t>(std::distance(state.name.begin(), found)));
}

inline std::int64_t stamp_of(const JointSample& sample) {
  return rclcpp::Time(sample.state.header.stamp).nanoseconds();
}

// Every joint command is the calibrated action due at its time, allowing for one lock step between
// the playback node reading the clock and the probe receiving the command.
inline void expect_commands_follow_the_chunks(const GazeboScenario& scenario) {
  const std::vector<rovi_interfaces::msg::ActionChunk>& chunks = scenario.action_chunks();
  ASSERT_FALSE(chunks.empty());
  std::size_t checked = 0;
  for (const CommandSample& command : scenario.commands()) {
    const std::optional<double> now = due(chunks, command.topic, command.received_ns);
    const std::optional<double> before =
        due(chunks, command.topic, command.received_ns - kLockStepNs);
    if (!now && !before) {
      continue;
    }
    const bool matches = (now && std::abs(*now - command.value) <= kExactly) ||
                         (before && std::abs(*before - command.value) <= kExactly);
    EXPECT_TRUE(matches) << so101_joints().at(command.topic) << " at " << command.received_ns
                         << " ns: " << command.value;
    ++checked;
  }
  EXPECT_GT(checked, 0U);
}

}  // namespace rovi::tests

#endif  // ROVI_TESTS_SO101_PLAYBACK_HPP_
