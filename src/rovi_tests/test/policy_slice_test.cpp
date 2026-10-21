#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/utilities.hpp>
#include <unistd.h>

#include "rovi_interfaces/msg/action_chunk.hpp"
#include "rovi_interfaces/msg/inference_record.hpp"
#include "rovi_interfaces/msg/policy_server.hpp"
#include "rovi_interfaces/msg/reservation_violation.hpp"
#include "rovi_interfaces/msg/robot_state.hpp"
#include "rovi_policy/wire.hpp"

#include "slice_fixture.hpp"
#include "stub_server.hpp"

namespace rovi::tests {
namespace {

using rovi_interfaces::msg::ActionChunk;
using rovi_interfaces::msg::InferenceRecord;
using rovi_interfaces::msg::PolicyServer;
using rovi_interfaces::msg::ReservationViolation;
using rovi_interfaces::msg::RobotState;

constexpr std::chrono::seconds kReplyLimit{20};
constexpr std::int64_t kMsNs = 1'000'000;

const std::vector<float>& recorded_state() {
  static const std::vector<float> state{1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F};
  return state;
}

policy::Observation recorded_observation() {
  return policy::Observation{
      .state = recorded_state(), .width = 4, .height = 2, .rgb = policy::Bytes(24, 0)};
}

class PolicySlice : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  static Overrides overrides(const StubServer& server, const std::string& robot_policy,
                             const std::string& worker_task = "test") {
    const std::filesystem::path observation =
        std::filesystem::temp_directory_path() /
        ("rovi_policy_slice_" + std::to_string(::getpid()) + ".rovobs");
    EXPECT_TRUE(policy::write_observation_file(
        observation, {.episode = 0, .frame = 0, .observation = recorded_observation()}));
    return Overrides{
        {"rovi_policy_worker",
         {"server_host:=127.0.0.1", "server_port:=" + std::to_string(server.port()),
          "policy_id:=" + std::string{kStubPolicy}, "task:=" + worker_task}},
        {"robot_agent",
         {"observation_file:=" + observation.string(),
          "upload_bytes:=" + std::to_string(policy::request_bytes(recorded_observation())),
          "policy_id:=" + robot_policy}}};
  }
};

// Chunks play back to back after the robot's initial coverage, and every state credits the buffer
// to the end of the newest chunk accepted before it.
void expect_states_follow_published_playback(const Slice& slice) {
  const std::vector<ActionChunk>& chunks = slice.action_chunks();
  ASSERT_FALSE(chunks.empty());
  ASSERT_FALSE(slice.states().empty());
  ASSERT_LT(slice.states().front().stamp_ns, chunks.front().accepted_ns);
  const auto end_of = [](const ActionChunk& chunk) {
    return chunk.start_ns + (static_cast<std::int64_t>(chunk.steps) * chunk.period_ns);
  };
  std::int64_t covered_until = slice.states().front().buffer_empty_time_ns;
  for (std::size_t index = 0; index < chunks.size(); ++index) {
    EXPECT_EQ(chunks.at(index).start_ns, std::max(chunks.at(index).accepted_ns, covered_until))
        << "chunk " << index;
    covered_until = end_of(chunks.at(index));
  }
  std::size_t checked = 0;
  for (const RobotState& state : slice.states()) {
    const ActionChunk* newest = nullptr;
    bool simultaneous = false;
    for (const ActionChunk& chunk : chunks) {
      newest = chunk.accepted_ns < state.stamp_ns ? &chunk : newest;
      simultaneous = simultaneous || chunk.accepted_ns == state.stamp_ns;
    }
    if (newest != nullptr && !simultaneous) {
      EXPECT_EQ(state.buffer_empty_time_ns, end_of(*newest)) << "state at " << state.stamp_ns;
      ++checked;
    }
  }
  EXPECT_GT(checked, 10U);
}

TEST_F(PolicySlice, DeliversValidatedStubChunksInTheirReservedSlots) {
  const StubServer server{"nominal", 0};
  ASSERT_NE(server.port(), 0U) << "the stub server did not start";
  Slice slice{overrides(server, std::string{kStubPolicy}), policy_slice(), "so101_policy"};
  slice.run_until(kStartNs + (8'000 * kMsNs));

  ASSERT_GE(slice.action_chunks().size(), 3U);
  for (const ActionChunk& chunk : slice.action_chunks()) {
    EXPECT_EQ(chunk.policy_id, kStubPolicy);
    EXPECT_EQ(chunk.steps, 50U);
    EXPECT_EQ(chunk.dim, 6U);
    EXPECT_EQ(chunk.period_ns, 33'333'333);
    ASSERT_EQ(chunk.actions.size(), 300U);
    for (std::size_t step = 0; step < chunk.steps; ++step) {
      for (std::size_t joint = 0; joint < chunk.dim; ++joint) {
        EXPECT_NEAR(chunk.actions.at((step * chunk.dim) + joint),
                    recorded_state().at(joint) + (0.01F * static_cast<float>(step)), 1e-4F);
      }
    }
  }
  for (const InferenceRecord& record : slice.inference_records()) {
    EXPECT_EQ(record.outcome, InferenceRecord::OUTCOME_OK);
    EXPECT_GE(record.request_ns, record.planned_start_ns);
    EXPECT_LE(record.response_ns, record.planned_end_ns);
    EXPECT_EQ(record.noise_seed, record.job_id);
    EXPECT_EQ(record.connection, 1U);
  }
  ASSERT_EQ(slice.policy_servers().size(), 1U);
  const PolicyServer& announced = slice.policy_servers().front();
  EXPECT_EQ(announced.connection, 1U);
  EXPECT_EQ(announced.policy_id, kStubPolicy);
  EXPECT_EQ(announced.task, "test");
  EXPECT_EQ(announced.policy_type, "stub");
  EXPECT_TRUE(announced.runtime.starts_with("stub, python "));
  EXPECT_EQ(announced.adapter_id.size(), 64U);
  EXPECT_EQ(announced.action_period_ns, 33'333'333);
  EXPECT_EQ(announced.image_width, 4U);
  EXPECT_TRUE(slice.violations().empty());
  ASSERT_TRUE(slice.last_state().has_value());
  EXPECT_EQ(slice.last_state()->chunks_invalid, 0U);
  EXPECT_EQ(slice.last_state()->chunks_missed, 0U);
  EXPECT_EQ(slice.last_state()->fallback_entries, 0U);
  expect_states_follow_published_playback(slice);
}

TEST_F(PolicySlice, RejectsChunksWhosePlaybackRateDiffersFromTheCreditedHorizon) {
  const StubServer server{"sixty_hertz", 0, {"--fps", "60"}};
  ASSERT_NE(server.port(), 0U) << "the stub server did not start";
  Slice slice{overrides(server, std::string{kStubPolicy}), policy_slice(), "so101_policy"};
  slice.run_until(kStartNs + (3'500 * kMsNs));

  ASSERT_FALSE(slice.inference_records().empty());
  EXPECT_FALSE(slice.chunks().empty());
  EXPECT_TRUE(slice.action_chunks().empty());
  ASSERT_TRUE(slice.last_state().has_value());
  EXPECT_GE(slice.last_state()->chunks_invalid, 1U);
  EXPECT_EQ(slice.last_state()->chunks_accepted, 0U);
}

TEST_F(PolicySlice, RefusesAChunkThatIsReadyAfterItsDownlinkLeaseStarts) {
  const StubServer server{"slow", 1'500};
  ASSERT_NE(server.port(), 0U) << "the stub server did not start";
  Overrides settings = overrides(server, std::string{kStubPolicy});
  settings["rovi_scheduler"] = {"workers.durations_ns:=[50000000]"};
  Slice slice{settings, policy_slice(), "so101_policy"};
  std::int64_t now = kStartNs;
  const auto deadline = std::chrono::steady_clock::now() + kReplyLimit;
  while (slice.inference_records().empty() && std::chrono::steady_clock::now() < deadline) {
    now += kMsNs;
    slice.run_until(now);
    std::this_thread::sleep_for(std::chrono::milliseconds{2});
  }
  slice.run_until(now + (10 * kMsNs));

  ASSERT_EQ(slice.inference_records().size(), 1U);
  const InferenceRecord& record = slice.inference_records().front();
  EXPECT_EQ(record.outcome, InferenceRecord::OUTCOME_OK);
  EXPECT_GT(record.response_ns, record.planned_end_ns);
  ASSERT_EQ(slice.violations().size(), 1U);
  const ReservationViolation& refused = slice.violations().front();
  EXPECT_EQ(refused.job_id, record.job_id);
  EXPECT_EQ(refused.direction, ReservationViolation::DIRECTION_DOWNLINK);
  EXPECT_EQ(refused.reason, ReservationViolation::REASON_LATE_SEND);
  EXPECT_TRUE(slice.action_chunks().empty());
}

TEST_F(PolicySlice, RefusesAServerServingAnotherTask) {
  const StubServer server{"other_task", 0};
  ASSERT_NE(server.port(), 0U) << "the stub server did not start";
  Slice slice{overrides(server, std::string{kStubPolicy}, "place"), policy_slice(), "so101_policy"};
  slice.run_until(kStartNs + (3'500 * kMsNs));

  ASSERT_FALSE(slice.inference_records().empty());
  EXPECT_EQ(slice.inference_records().front().outcome, InferenceRecord::OUTCOME_IDENTITY);
  EXPECT_TRUE(slice.policy_servers().empty());
  EXPECT_TRUE(slice.chunks().empty());
  EXPECT_TRUE(slice.action_chunks().empty());
}

TEST_F(PolicySlice, DiscardsAChunkFromAnotherPolicyThanTheRobotExpects) {
  const StubServer server{"other_policy", 0};
  ASSERT_NE(server.port(), 0U) << "the stub server did not start";
  Slice slice{overrides(server, std::string(64, 'a')), policy_slice(), "so101_policy"};
  slice.run_until(kStartNs + (3'500 * kMsNs));

  ASSERT_FALSE(slice.inference_records().empty());
  EXPECT_FALSE(slice.chunks().empty());
  EXPECT_TRUE(slice.action_chunks().empty());
  ASSERT_TRUE(slice.last_state().has_value());
  EXPECT_GE(slice.last_state()->chunks_invalid, 1U);
  EXPECT_EQ(slice.last_state()->chunks_accepted, 0U);
}

}  // namespace
}  // namespace rovi::tests
