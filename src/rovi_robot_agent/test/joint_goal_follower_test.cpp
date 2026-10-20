#include "rovi_robot_agent/joint_goal_follower.hpp"

#include <cstddef>
#include <functional>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "rovi_core/units.hpp"

namespace rovi::robot_agent {
namespace {

constexpr double kSpeed = 0.5;
constexpr double kTolerance = 0.05;
constexpr double kPrecision = 1e-12;
constexpr double kLimit = 2.0;
constexpr core::Duration kStep = core::Duration::from_ns(100'000'000);
constexpr core::Duration kSettle = core::Duration::from_ns(250'000'000);

JointFollowerConfig config_through(std::vector<std::vector<double>> goals) {
  const std::size_t joints = goals.front().size();
  return JointFollowerConfig{.goals = std::move(goals),
                             .lower = std::vector<double>(joints, -kLimit),
                             .upper = std::vector<double>(joints, kLimit),
                             .max_speed = kSpeed,
                             .tolerance = kTolerance,
                             .settle_time = kSettle};
}

JointGoalFollower follower_through(std::vector<std::vector<double>> goals) {
  return JointGoalFollower::create(config_through(std::move(goals))).value();
}

void expect_near(const std::optional<std::vector<double>>& actual,
                 const std::vector<double>& expected) {
  ASSERT_TRUE(actual.has_value());
  ASSERT_EQ(actual->size(), expected.size());
  for (std::size_t joint = 0; joint < expected.size(); ++joint) {
    EXPECT_NEAR(actual->at(joint), expected.at(joint), kPrecision) << "joint " << joint;
  }
}

TEST(JointGoalFollower, StartsFromTheMeasuredPoseAndMovesAtMostSpeedTimesStep) {
  JointGoalFollower follower = follower_through({{1.0, -1.0, 0.02}});

  expect_near(follower.step({0.0, 0.0, 0.0}, kStep), {0.05, -0.05, 0.02});
  expect_near(follower.step({0.9, 0.9, 0.9}, kStep), {0.10, -0.10, 0.02});
  EXPECT_EQ(follower.joint_count(), 3U);
}

TEST(JointGoalFollower, ReachesTheGoalExactlyWithoutOvershooting) {
  JointGoalFollower follower = follower_through({{0.12}, {-1.0}});

  expect_near(follower.step({0.0}, kStep), {0.05});
  expect_near(follower.step({0.0}, kStep), {0.10});
  expect_near(follower.step({0.0}, kStep), {0.12});
  EXPECT_EQ(follower.goal(), 0U);
}

TEST(JointGoalFollower, HoldsTheGoalWhileTheMeasurementLagsBeyondTolerance) {
  JointGoalFollower follower = follower_through({{0.05}, {-1.0}});

  expect_near(follower.step({0.0}, kStep), {0.05});
  expect_near(follower.step({-0.2}, kStep), {0.05});

  EXPECT_EQ(follower.goal(), 0U);
}

TEST(JointGoalFollower, AdvancesOnceReachedAndMeasuredWithinToleranceThenLoops) {
  JointGoalFollower follower = follower_through({{0.05}, {0.0}});

  expect_near(follower.step({0.0}, kStep), {0.05});
  expect_near(follower.step({0.04}, kStep), {0.0});
  EXPECT_EQ(follower.goal(), 1U);
  expect_near(follower.step({0.01}, kStep), {0.05});
  EXPECT_EQ(follower.goal(), 0U);
}

TEST(JointGoalFollower, AdvancesOnlyWhenEveryJointIsWithinTolerance) {
  JointGoalFollower follower = follower_through({{0.05, 0.05}, {0.0, 0.0}});

  expect_near(follower.step({0.0, 0.0}, kStep), {0.05, 0.05});
  expect_near(follower.step({0.05, -0.2}, kStep), {0.05, 0.05});

  EXPECT_EQ(follower.goal(), 0U);
}

TEST(JointGoalFollower, AdvancesAfterTheSettleTimeWhenTheMeasurementNeverConverges) {
  JointGoalFollower follower = follower_through({{0.05}, {0.0}});

  expect_near(follower.step({0.0}, kStep), {0.05});
  expect_near(follower.step({-0.2}, kStep), {0.05});
  expect_near(follower.step({-0.2}, kStep), {0.05});
  expect_near(follower.step({-0.2}, kStep), {0.05});
  EXPECT_EQ(follower.goal(), 0U);

  expect_near(follower.step({-0.2}, kStep), {0.0});
  EXPECT_EQ(follower.goal(), 1U);
}

TEST(JointGoalFollower, RestartsTheSettleTimeAtEveryGoal) {
  JointGoalFollower follower = follower_through({{0.05}, {0.0}, {0.05}});

  static_cast<void>(follower.step({0.0}, kStep));
  for (int held = 0; held < 3; ++held) {
    static_cast<void>(follower.step({-0.2}, kStep));
  }
  expect_near(follower.step({-0.2}, kStep), {0.0});
  expect_near(follower.step({-0.2}, kStep), {0.0});

  EXPECT_EQ(follower.goal(), 1U);
}

TEST(JointGoalFollower, SeedsTheReferenceInsideTheJointLimits) {
  const double huge = 1e300;
  JointGoalFollower follower = follower_through({{1.0, -1.0}});

  expect_near(follower.step({huge, -huge}, kStep), {kLimit - 0.05, -kLimit + 0.05});
  expect_near(follower.step({huge, -huge}, kStep), {kLimit - 0.10, -kLimit + 0.10});
}

TEST(JointGoalFollower, StopHoldsTheClampedMeasuredPoseAndTheNextStepStartsFromThere) {
  JointGoalFollower follower = follower_through({{1.0}, {-1.0}});
  static_cast<void>(follower.step({0.0}, kStep));
  static_cast<void>(follower.step({0.0}, kStep));

  expect_near(follower.stop({0.02}), {0.02});
  expect_near(follower.stop({9.0}), {kLimit});
  expect_near(follower.step({0.3}, kStep), {0.35});
  EXPECT_EQ(follower.goal(), 0U);
}

TEST(JointGoalFollower, StopRejectsMismatchedOrNonFiniteMeasurements) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  JointGoalFollower follower = follower_through({{1.0, 1.0}});

  EXPECT_EQ(follower.stop({0.0}), std::nullopt);
  EXPECT_EQ(follower.stop({0.0, nan}), std::nullopt);
}

TEST(JointGoalFollower, RejectsMismatchedOrNonFiniteMeasurementsAndSteps) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  JointGoalFollower follower = follower_through({{1.0, 1.0}});

  EXPECT_EQ(follower.step({0.0}, kStep), std::nullopt);
  EXPECT_EQ(follower.step({0.0, nan}, kStep), std::nullopt);
  EXPECT_EQ(follower.step({0.0, 0.0}, core::Duration{}), std::nullopt);
  EXPECT_EQ(follower.step({0.0, 0.0}, core::Duration::from_ns(-1)), std::nullopt);
  expect_near(follower.step({0.0, 0.0}, kStep), {0.05, 0.05});
}

TEST(JointGoalFollower, CreateRejectsAnInvalidConfiguration) {
  const double infinity = std::numeric_limits<double>::infinity();
  const JointFollowerConfig valid = config_through({{0.0, 1.0}});
  const auto invalid = [&valid](const std::function<void(JointFollowerConfig&)>& change) {
    JointFollowerConfig config = valid;
    change(config);
    return !JointGoalFollower::create(std::move(config)).has_value();
  };

  EXPECT_TRUE(invalid([](JointFollowerConfig& config) { config.goals = {}; }));
  EXPECT_TRUE(invalid([](JointFollowerConfig& config) { config.goals = {{}}; }));
  EXPECT_TRUE(invalid([](JointFollowerConfig& config) { config.goals = {{0.0, 1.0}, {0.0}}; }));
  EXPECT_TRUE(invalid([&](JointFollowerConfig& config) { config.goals = {{0.0, infinity}}; }));
  EXPECT_TRUE(invalid([](JointFollowerConfig& config) { config.goals = {{0.0, 2.5}}; }));
  EXPECT_TRUE(invalid([](JointFollowerConfig& config) { config.lower = {-1.0}; }));
  EXPECT_TRUE(invalid([](JointFollowerConfig& config) { config.upper = {1.0, 2.0, 3.0}; }));
  EXPECT_TRUE(invalid([](JointFollowerConfig& config) { config.lower = {-1.0, 2.0}; }));
  EXPECT_TRUE(invalid([&](JointFollowerConfig& config) { config.lower = {-infinity, -1.0}; }));
  EXPECT_TRUE(invalid([](JointFollowerConfig& config) { config.max_speed = 0.0; }));
  EXPECT_TRUE(invalid([&](JointFollowerConfig& config) { config.max_speed = infinity; }));
  EXPECT_TRUE(invalid([](JointFollowerConfig& config) { config.tolerance = 0.0; }));
  EXPECT_TRUE(invalid([](JointFollowerConfig& config) { config.settle_time = core::Duration{}; }));
  EXPECT_FALSE(invalid([](JointFollowerConfig& /*config*/) {}));
}

TEST(JointGoalFollower, CountsACycleEachTimeTheLastGoalLoopsBackToTheFirst) {
  JointGoalFollower follower = follower_through({{0.05}, {0.0}});
  EXPECT_EQ(follower.cycles(), 0U);

  static_cast<void>(follower.step({0.0}, kStep));
  static_cast<void>(follower.step({0.05}, kStep));
  EXPECT_EQ(follower.goal(), 1U);
  EXPECT_EQ(follower.cycles(), 0U);
  static_cast<void>(follower.step({0.0}, kStep));

  EXPECT_EQ(follower.goal(), 0U);
  EXPECT_EQ(follower.cycles(), 1U);
}

}  // namespace
}  // namespace rovi::robot_agent
