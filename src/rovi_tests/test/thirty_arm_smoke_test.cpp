#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/utilities.hpp>

#include "rovi_interfaces/msg/decision.hpp"
#include "rovi_interfaces/msg/group_release.hpp"
#include "rovi_interfaces/msg/job_request.hpp"
#include "rovi_interfaces/msg/payload.hpp"
#include "rovi_interfaces/msg/robot_state.hpp"
#include "rovi_interfaces/msg/run_metadata.hpp"

#include "arm_motion.hpp"
#include "gazebo_fixture.hpp"

namespace rovi::tests {
namespace {

using rovi_interfaces::msg::Decision;
using rovi_interfaces::msg::GroupRelease;
using rovi_interfaces::msg::JobRequest;
using rovi_interfaces::msg::Payload;
using rovi_interfaces::msg::RobotState;

constexpr std::int64_t kNsPerSecond = 1'000'000'000;
constexpr std::int64_t kNsPerMs = 1'000'000;
constexpr std::int64_t kProfileStepNs = 100'000'000;
constexpr std::int64_t kEndNs = 13 * kNsPerSecond;
constexpr std::int64_t kOutageStartNs = 6 * kNsPerSecond;
constexpr std::int64_t kOutageEndNs = 7'500'000'000;
constexpr std::int64_t kSettleNs = kNsPerSecond;
constexpr std::int64_t kBrakingAllowanceNs = 800'000'000;
constexpr std::int64_t kSkewBoundNs = 80'000'000;
constexpr std::uint32_t kRobots = 30;
constexpr std::uint32_t kTeamSize = 3;
constexpr std::uint32_t kWorkcells = kRobots / kTeamSize;
constexpr double kMinimumTravel = 1.0;
constexpr double kMinimumRecoveryTravel = 0.3;
constexpr double kMaximumHoldDrift = 0.05;
constexpr std::size_t kMinimumReleasesPerTeam = 5;
constexpr std::size_t kMinimumSharedTimelines = 3;
constexpr std::chrono::seconds kWallLimit{1'500};

class ThirtyArms : public ::testing::Test {
 protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

struct Window {
  std::int64_t epoch{0};
  std::int64_t outage_start{0};
  std::int64_t outage_end{0};
  std::int64_t end{0};
};

std::string travel_profile(const std::vector<ArmSample>& samples, std::int64_t from_ns,
                           std::int64_t to_ns, std::int64_t epoch) {
  std::ostringstream profile;
  for (std::int64_t start = from_ns; start < to_ns; start += kProfileStepNs) {
    profile << (start - epoch) / kNsPerMs
            << "ms:" << travel_between(samples, start, start + kProfileStepNs) << " ";
  }
  return profile.str();
}

std::string describe(const std::vector<std::pair<std::int64_t, std::int64_t>>& intervals,
                     std::int64_t epoch) {
  std::ostringstream text;
  for (const auto& [start, end] : intervals) {
    text << (start - epoch) / kNsPerMs << ".." << (end - epoch) / kNsPerMs << " ";
  }
  return text.str();
}

void expect_groups_all_or_none(const GazeboScenario& scenario, const Window& window) {
  std::map<std::uint32_t, std::vector<const JobRequest*>> groups;
  for (const JobRequest& request : scenario.requests()) {
    EXPECT_TRUE(request.has_synchronization_group) << "job " << request.job_id;
    EXPECT_EQ(request.synchronization_group_size, kTeamSize) << "job " << request.job_id;
    groups[request.synchronization_group].push_back(&request);
  }
  std::map<std::uint64_t, std::uint8_t> statuses;
  for (const Decision& decision : scenario.decisions()) {
    if (decision.status != Decision::STATUS_DEFERRED) {
      statuses.insert_or_assign(decision.job_id, decision.status);
    }
  }
  for (const auto& [group, members] : groups) {
    EXPECT_EQ(members.size(), kTeamSize) << "group " << group;
    std::set<std::uint8_t> outcomes;
    std::size_t undecided = 0;
    std::int64_t latest_release = 0;
    for (const JobRequest* member : members) {
      latest_release = std::max(latest_release, member->release_time_ns);
      if (const auto status = statuses.find(member->job_id); status != statuses.end()) {
        outcomes.insert(status->second);
      } else {
        ++undecided;
      }
    }
    EXPECT_LE(outcomes.size(), 1U) << "group " << group << " was partially admitted";
    if (latest_release < window.end - kNsPerSecond) {
      EXPECT_EQ(undecided, 0U) << "group " << group << " left members undecided";
    }
  }
}

void expect_releases_within_skew(const GazeboScenario& scenario) {
  std::map<std::uint64_t, std::int64_t> delivered;
  for (const Payload& chunk : scenario.chunks()) {
    delivered.insert_or_assign(chunk.job_id, chunk.delivered_ns);
  }
  std::map<std::uint32_t, std::size_t> released_per_team;
  for (const GroupRelease& release : scenario.releases()) {
    if (!release.released) {
      continue;
    }
    ++released_per_team[release.workcell];
    std::vector<std::int64_t> times;
    for (const std::uint64_t job : release.jobs) {
      if (const auto found = delivered.find(job); found != delivered.end()) {
        times.push_back(found->second);
      }
    }
    ASSERT_EQ(times.size(), release.jobs.size()) << "group " << release.synchronization_group;
    const auto [earliest, latest] = std::ranges::minmax(times);
    EXPECT_LE(latest - earliest, kSkewBoundNs) << "group " << release.synchronization_group;
    EXPECT_EQ(latest - earliest, release.result_ready_skew_ns)
        << "group " << release.synchronization_group;
    EXPECT_EQ(latest, release.release_time_ns) << "group " << release.synchronization_group;
  }
  for (std::uint32_t team = 0; team < kWorkcells; ++team) {
    EXPECT_GE(released_per_team[team], kMinimumReleasesPerTeam) << "workcell " << team;
  }
}

void expect_teams_share_one_timeline(const GazeboScenario& scenario, const Window& window) {
  std::map<std::uint32_t, std::set<std::int64_t>> buffer_ends;
  for (const RobotState& state : scenario.states()) {
    if (state.stamp_ns > window.outage_end + kSettleNs &&
        state.mode == RobotState::MODE_EXECUTING) {
      buffer_ends[state.robot_id].insert(state.buffer_empty_time_ns);
    }
  }
  for (std::uint32_t team = 0; team < kWorkcells; ++team) {
    std::set<std::int64_t> shared = buffer_ends[team * kTeamSize];
    for (std::uint32_t member = 1; member < kTeamSize; ++member) {
      std::set<std::int64_t> common;
      std::ranges::set_intersection(shared, buffer_ends[(team * kTeamSize) + member],
                                    std::inserter(common, common.begin()));
      shared = std::move(common);
    }
    EXPECT_GE(shared.size(), kMinimumSharedTimelines) << "workcell " << team;
  }
}

void expect_arm_cycles_holds_and_recovers(const GazeboScenario& scenario, const Window& window,
                                          std::uint32_t robot) {
  SCOPED_TRACE(::testing::Message() << "robot " << robot);
  const std::vector<ArmSample> samples = arm_samples(scenario.joint_samples(), robot);
  EXPECT_GE(travel_between(samples, 0, window.outage_start), kMinimumTravel);
  const auto intervals = fallback_intervals(scenario.states(), robot);
  const auto during_outage = std::ranges::find_if(intervals, [&](const auto& interval) {
    return interval.first <= window.outage_end && interval.second >= window.outage_start;
  });
  if (during_outage == intervals.end()) {
    ADD_FAILURE() << "no fallback during the outage; intervals "
                  << describe(intervals, window.epoch);
    return;
  }
  EXPECT_GT(during_outage->second - during_outage->first, kBrakingAllowanceNs);
  const std::optional<double> drift =
      drift_between(samples, during_outage->first + kBrakingAllowanceNs, during_outage->second);
  ASSERT_TRUE(drift.has_value()) << "no joint samples while holding";
  EXPECT_LE(*drift, kMaximumHoldDrift)
      << "fallback " << describe({*during_outage}, window.epoch) << "; travel per 100 ms: "
      << travel_profile(samples, during_outage->first, during_outage->second, window.epoch);
  EXPECT_GE(travel_between(samples, window.outage_end + kSettleNs, window.end),
            kMinimumRecoveryTravel)
      << "fallback intervals " << describe(intervals, window.epoch);
  EXPECT_TRUE(std::ranges::any_of(scenario.states(), [&](const RobotState& state) {
    return state.robot_id == robot && state.stamp_ns > window.outage_end + kSettleNs &&
           state.mode == RobotState::MODE_EXECUTING;
  })) << "never executed again after the outage";
}

void expect_accounting_balances(const GazeboScenario& scenario) {
  std::map<std::uint32_t, RobotState> last;
  for (const RobotState& state : scenario.states()) {
    last.insert_or_assign(state.robot_id, state);
  }
  EXPECT_EQ(last.size(), kRobots);
  for (const auto& [robot, state] : last) {
    const std::uint64_t open = state.has_job_in_flight ? 1U : 0U;
    EXPECT_EQ(state.jobs_submitted,
              state.chunks_accepted + state.jobs_rejected + state.chunks_missed + open)
        << "robot " << robot;
  }
}

TEST_F(ThirtyArms, TeamsCycleTogetherHoldThroughTheHandoverOutageAndRecover) {
  GazeboScenario scenario{thirty_arm_scenario()};

  ASSERT_TRUE(scenario.run_until(kEndNs, kWallLimit)) << scenario.failure();

  ASSERT_TRUE(scenario.metadata().has_value());
  const std::int64_t epoch = scenario.metadata()->epoch_ns;
  const Window window{.epoch = epoch,
                      .outage_start = epoch + kOutageStartNs,
                      .outage_end = epoch + kOutageEndNs,
                      .end = kEndNs};
  expect_groups_all_or_none(scenario, window);
  expect_releases_within_skew(scenario);
  expect_teams_share_one_timeline(scenario, window);
  for (std::uint32_t robot = 0; robot < kRobots; ++robot) {
    expect_arm_cycles_holds_and_recovers(scenario, window, robot);
  }
  expect_accounting_balances(scenario);
}

}  // namespace
}  // namespace rovi::tests
