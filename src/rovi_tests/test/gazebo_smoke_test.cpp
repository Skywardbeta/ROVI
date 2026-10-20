#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/utilities.hpp>

#include "rovi_interfaces/msg/robot_state.hpp"
#include "rovi_interfaces/msg/run_metadata.hpp"

#include "gazebo_fixture.hpp"

namespace rovi::tests {
namespace {

using rovi_interfaces::msg::RobotState;

constexpr std::int64_t kNsPerSecond = 1'000'000'000;
constexpr std::int64_t kEndNs = 13 * kNsPerSecond;
constexpr std::int64_t kOutageStartNs = 6 * kNsPerSecond;
constexpr std::int64_t kOutageEndNs = 7'500'000'000;
constexpr std::int64_t kSettleNs = kNsPerSecond;
constexpr double kStoppedSpeed = 0.05;
constexpr std::int64_t kBrakingAllowanceNs = 600'000'000;
constexpr double kMinimumTravel = 1.0;
constexpr double kMinimumRecoveryTravel = 0.3;
constexpr std::size_t kMinimumChunks = 20;
constexpr std::chrono::seconds kWallLimit{60};

class GazeboSmoke : public ::testing::Test {
 protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

double path_length_after(const std::vector<OdometrySample>& samples, std::int64_t from_ns) {
  double length = 0.0;
  const OdometrySample* previous = nullptr;
  for (const OdometrySample& sample : samples) {
    if (sample.stamp_ns < from_ns) {
      continue;
    }
    if (previous != nullptr) {
      length += std::hypot(sample.x - previous->x, sample.y - previous->y);
    }
    previous = &sample;
  }
  return length;
}

TEST_F(GazeboSmoke, RobotDrivesStopsDuringTheHandoverOutageAndRecovers) {
  GazeboScenario scenario{mobile_robot_scenario()};

  ASSERT_TRUE(scenario.run_until(kEndNs, kWallLimit)) << scenario.failure();

  ASSERT_TRUE(scenario.metadata().has_value());
  EXPECT_EQ(scenario.metadata()->network_profile_id, "synthetic-edge-handover-v1");
  const std::int64_t epoch = scenario.metadata()->epoch_ns;
  const std::int64_t outage_start = epoch + kOutageStartNs;
  const std::int64_t outage_end = epoch + kOutageEndNs;

  EXPECT_GE(path_length_after(scenario.odometry(), 0) -
                path_length_after(scenario.odometry(), outage_start),
            kMinimumTravel);

  const auto intervals = fallback_intervals(scenario.states(), 0);
  const auto during_outage = std::ranges::find_if(intervals, [&](const auto& interval) {
    return interval.first >= outage_start && interval.first <= outage_end;
  });
  ASSERT_NE(during_outage, intervals.end());
  const std::int64_t braked = during_outage->first + kBrakingAllowanceNs;
  std::vector<double> stopped_speeds;
  for (const OdometrySample& sample : scenario.odometry()) {
    if (sample.stamp_ns >= braked && sample.stamp_ns <= during_outage->second) {
      stopped_speeds.push_back(std::abs(sample.linear_speed));
    }
  }
  ASSERT_FALSE(stopped_speeds.empty());
  EXPECT_TRUE(
      std::ranges::all_of(stopped_speeds, [](double speed) { return speed < kStoppedSpeed; }));

  EXPECT_GE(path_length_after(scenario.odometry(), outage_end + kSettleNs), kMinimumRecoveryTravel);
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
