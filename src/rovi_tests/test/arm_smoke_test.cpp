#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/utilities.hpp>

#include "rovi_interfaces/msg/robot_state.hpp"
#include "rovi_interfaces/msg/run_metadata.hpp"

#include "arm_motion.hpp"
#include "gazebo_fixture.hpp"

namespace rovi::tests {
namespace {

using rovi_interfaces::msg::RobotState;

constexpr std::int64_t kNsPerSecond = 1'000'000'000;
constexpr double kSecondsPerNs = 1e-9;
constexpr std::int64_t kEndNs = 16 * kNsPerSecond;
constexpr std::int64_t kOutageStartNs = 6 * kNsPerSecond;
constexpr std::int64_t kOutageEndNs = 7'500'000'000;
constexpr std::int64_t kSettleNs = kNsPerSecond;
constexpr std::int64_t kCommandMarginNs = 50'000'000;
constexpr std::int64_t kBrakingAllowanceNs = 800'000'000;
constexpr double kHeldSpeed = 0.02;
constexpr double kMinimumTravel = 2.0;
constexpr double kMinimumRecoveryTravel = 0.5;
constexpr std::size_t kMinimumHeldCommands = 10;
constexpr std::size_t kMinimumChunks = 20;
constexpr double kAbovePickJoint1 = -0.168;
constexpr double kAbovePlaceJoint1 = 0.168;
constexpr double kSetpointPrecision = 1e-9;
constexpr std::chrono::seconds kWallLimit{90};

class ArmSmoke : public ::testing::Test {
 protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

std::vector<double> speeds_between(const std::vector<ArmSample>& samples, std::int64_t from_ns,
                                   std::int64_t to_ns) {
  std::vector<double> speeds;
  for (std::size_t later = 1; later < samples.size(); ++later) {
    const ArmSample& previous = samples.at(later - 1);
    const ArmSample& current = samples.at(later);
    if (previous.stamp_ns >= from_ns && current.stamp_ns <= to_ns &&
        current.stamp_ns > previous.stamp_ns) {
      speeds.push_back(largest_change(previous, current) /
                       (static_cast<double>(current.stamp_ns - previous.stamp_ns) * kSecondsPerNs));
    }
  }
  return speeds;
}

TEST_F(ArmSmoke, ArmCyclesHoldsItsSetpointDuringTheHandoverOutageAndResumes) {
  GazeboScenario scenario{arm_workcell_scenario()};

  ASSERT_TRUE(scenario.run_until(kEndNs, kWallLimit)) << scenario.failure();

  ASSERT_TRUE(scenario.metadata().has_value());
  EXPECT_EQ(scenario.metadata()->network_profile_id, "synthetic-edge-handover-v1");
  const std::int64_t epoch = scenario.metadata()->epoch_ns;
  const std::int64_t outage_start = epoch + kOutageStartNs;
  const std::int64_t outage_end = epoch + kOutageEndNs;
  const std::vector<ArmSample> samples = arm_samples(scenario.joint_samples(), 0);

  EXPECT_GE(travel_between(samples, 0, outage_start), kMinimumTravel);
  std::vector<double> joint1_setpoints;
  for (const CommandSample& command : scenario.commands()) {
    if (command.topic == 0) {
      joint1_setpoints.push_back(command.value);
    }
  }
  ASSERT_FALSE(joint1_setpoints.empty());
  EXPECT_LE(std::ranges::min(joint1_setpoints), kAbovePickJoint1 + kSetpointPrecision)
      << "the arm never reached the above-pick goal";
  EXPECT_GE(std::ranges::max(joint1_setpoints), kAbovePlaceJoint1 - kSetpointPrecision)
      << "the arm never cycled on to the above-place goal";

  const auto intervals = fallback_intervals(scenario.states(), 0);
  const auto during_outage = std::ranges::find_if(intervals, [&](const auto& interval) {
    return interval.first >= outage_start && interval.first <= outage_end;
  });
  ASSERT_NE(during_outage, intervals.end());
  for (std::size_t topic = 0; topic < static_cast<std::size_t>(kArmJoints); ++topic) {
    std::vector<double> held;
    for (const CommandSample& command : scenario.commands()) {
      if (command.topic == topic &&
          command.received_ns >= during_outage->first + kCommandMarginNs &&
          command.received_ns <= during_outage->second - kCommandMarginNs) {
        held.push_back(command.value);
      }
    }
    ASSERT_GE(held.size(), kMinimumHeldCommands) << "joint " << topic + 1;
    EXPECT_TRUE(std::ranges::all_of(held, [&held](double value) { return value == held.front(); }))
        << "joint " << topic + 1 << " setpoint changed while the robot was in fallback";
  }
  const std::vector<double> held_speeds =
      speeds_between(samples, during_outage->first + kBrakingAllowanceNs, during_outage->second);
  ASSERT_FALSE(held_speeds.empty());
  EXPECT_LT(std::ranges::max(held_speeds), kHeldSpeed);

  EXPECT_GE(travel_between(samples, outage_end + kSettleNs, kEndNs), kMinimumRecoveryTravel);
  ASSERT_FALSE(scenario.states().empty());
  const RobotState& last = scenario.states().back();
  EXPECT_EQ(last.mode, RobotState::MODE_EXECUTING);
  EXPECT_GE(last.fallback_entries, 1U);
  EXPECT_GE(scenario.chunks().size(), kMinimumChunks);
  const std::uint64_t open = last.has_job_in_flight ? 1U : 0U;
  EXPECT_EQ(last.jobs_submitted,
            last.chunks_accepted + last.jobs_rejected + last.chunks_missed + open);
}

}  // namespace
}  // namespace rovi::tests
