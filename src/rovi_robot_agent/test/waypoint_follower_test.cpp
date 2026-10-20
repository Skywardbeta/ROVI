#include "rovi_robot_agent/waypoint_follower.hpp"

#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace rovi::robot_agent {
namespace {

constexpr double kMaxLinear = 0.5;
constexpr double kMaxAngular = 1.0;
constexpr double kGain = 0.8;
constexpr double kTolerance = 0.1;
constexpr double kPrecision = 1e-9;

FollowerConfig config_through(std::vector<Point2D> waypoints) {
  return FollowerConfig{.waypoints = std::move(waypoints),
                        .arrival_tolerance = kTolerance,
                        .max_linear = kMaxLinear,
                        .max_angular = kMaxAngular,
                        .heading_gain = kGain};
}

WaypointFollower follower_through(std::vector<Point2D> waypoints) {
  return WaypointFollower::create(config_through(std::move(waypoints))).value();
}

TEST(WaypointFollower, DrivesStraightAtFullSpeedWhenFacingTheTarget) {
  WaypointFollower follower = follower_through({{.x = 5.0, .y = 0.0}});

  const Velocity velocity = follower.command(Pose2D{});

  EXPECT_NEAR(velocity.linear, kMaxLinear, kPrecision);
  EXPECT_NEAR(velocity.angular, 0.0, kPrecision);
}

TEST(WaypointFollower, TurnsInPlaceWhenTheTargetIsBehind) {
  WaypointFollower follower = follower_through({{.x = -5.0, .y = 0.0}});

  const Velocity velocity = follower.command(Pose2D{});

  EXPECT_NEAR(velocity.linear, 0.0, kPrecision);
  EXPECT_NEAR(velocity.angular, kMaxAngular, kPrecision);
}

TEST(WaypointFollower, SlowsAndSteersInProportionToTheHeadingError) {
  WaypointFollower follower = follower_through({{.x = 5.0, .y = 5.0}});
  const double error = std::numbers::pi / 4.0;

  const Velocity velocity = follower.command(Pose2D{});

  EXPECT_NEAR(velocity.linear, kMaxLinear * std::cos(error), kPrecision);
  EXPECT_NEAR(velocity.angular, kGain * error, kPrecision);
}

TEST(WaypointFollower, ClampsTheAngularVelocityInBothDirections) {
  WaypointFollower left = follower_through({{.x = 0.0, .y = 5.0}});
  WaypointFollower right = follower_through({{.x = 0.0, .y = -5.0}});

  EXPECT_NEAR(left.command(Pose2D{}).angular, kMaxAngular, kPrecision);
  EXPECT_NEAR(right.command(Pose2D{}).angular, -kMaxAngular, kPrecision);
}

TEST(WaypointFollower, WrapsTheHeadingErrorAcrossPi) {
  WaypointFollower follower = follower_through({{.x = -5.0, .y = -0.5}});
  const Pose2D pose{.x = 0.0, .y = 0.0, .yaw = 3.0};
  const double error = std::atan2(-0.5, -5.0) + (2.0 * std::numbers::pi) - 3.0;

  const Velocity velocity = follower.command(pose);

  EXPECT_NEAR(velocity.angular, kGain * error, kPrecision);
  EXPECT_NEAR(velocity.linear, kMaxLinear * std::cos(error), kPrecision);
}

TEST(WaypointFollower, AdvancesWithinToleranceAndLoopsBackToTheFirstWaypoint) {
  WaypointFollower follower = follower_through({{.x = 1.0, .y = 0.0}, {.x = 1.0, .y = 1.0}});

  EXPECT_EQ(follower.target(), 0U);
  const Velocity toward_second = follower.command(Pose2D{.x = 0.95, .y = 0.0, .yaw = 0.0});
  EXPECT_EQ(follower.target(), 1U);
  EXPECT_NEAR(toward_second.angular, kMaxAngular, kPrecision);
  static_cast<void>(follower.command(Pose2D{.x = 1.0, .y = 0.95, .yaw = 0.0}));
  EXPECT_EQ(follower.target(), 0U);
}

TEST(WaypointFollower, StopsWhenEveryWaypointIsWithinTolerance) {
  WaypointFollower follower = follower_through({{.x = 0.0, .y = 0.0}, {.x = 0.05, .y = 0.0}});

  EXPECT_EQ(follower.command(Pose2D{}), Velocity{});
}

TEST(WaypointFollower, CreateRejectsAnInvalidOrNonFiniteConfiguration) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  const auto invalid = [](auto mutate) {
    FollowerConfig config = config_through({{.x = 1.0, .y = 0.0}});
    mutate(config);
    return !WaypointFollower::create(config).has_value();
  };

  EXPECT_TRUE(invalid([](FollowerConfig& config) { config.waypoints.clear(); }));
  EXPECT_TRUE(invalid([nan](FollowerConfig& config) { config.waypoints.front().x = nan; }));
  EXPECT_TRUE(
      invalid([infinity](FollowerConfig& config) { config.waypoints.front().y = infinity; }));
  EXPECT_TRUE(invalid([](FollowerConfig& config) { config.arrival_tolerance = 0.0; }));
  EXPECT_TRUE(invalid([](FollowerConfig& config) { config.max_linear = -0.1; }));
  EXPECT_TRUE(invalid([](FollowerConfig& config) { config.max_angular = -0.1; }));
  EXPECT_TRUE(invalid([](FollowerConfig& config) { config.heading_gain = 0.0; }));
  EXPECT_TRUE(invalid([nan](FollowerConfig& config) { config.max_linear = nan; }));
  EXPECT_FALSE(invalid([](FollowerConfig&) {}));
}

TEST(WaypointFollower, StopsWithoutAdvancingForANonFinitePose) {
  WaypointFollower follower = follower_through({{.x = 1.0, .y = 0.0}, {.x = 2.0, .y = 0.0}});
  const double nan = std::numeric_limits<double>::quiet_NaN();

  EXPECT_EQ(follower.command(Pose2D{.x = nan, .y = 0.0, .yaw = 0.0}), Velocity{});
  EXPECT_EQ(follower.command(Pose2D{.x = 0.0, .y = 0.0, .yaw = nan}), Velocity{});
  EXPECT_EQ(follower.target(), 0U);
}

TEST(MotionGate, BuildsAPlanarPoseOnlyFromFiniteOdometry) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  const double half = std::numbers::sqrt2 / 2.0;

  const std::optional<Pose2D> pose = planar_pose(1.0, 2.0, 0.0, 0.0, half, half);

  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->x, 1.0, kPrecision);
  EXPECT_NEAR(pose->y, 2.0, kPrecision);
  EXPECT_NEAR(pose->yaw, std::numbers::pi / 2.0, kPrecision);
  EXPECT_FALSE(planar_pose(nan, 0.0, 0.0, 0.0, 0.0, 1.0).has_value());
  EXPECT_FALSE(planar_pose(0.0, infinity, 0.0, 0.0, 0.0, 1.0).has_value());
  EXPECT_FALSE(planar_pose(0.0, 0.0, nan, 0.0, 0.0, 1.0).has_value());
  EXPECT_FALSE(planar_pose(0.0, 0.0, 0.0, 0.0, 0.0, infinity).has_value());
}

TEST(MotionGate, ExtractsYawFromAUnitQuaternion) {
  const double half = std::numbers::sqrt2 / 2.0;

  EXPECT_NEAR(yaw_from_quaternion(0.0, 0.0, 0.0, 1.0), 0.0, kPrecision);
  EXPECT_NEAR(yaw_from_quaternion(0.0, 0.0, half, half), std::numbers::pi / 2.0, kPrecision);
  EXPECT_NEAR(std::abs(yaw_from_quaternion(0.0, 0.0, 1.0, 0.0)), std::numbers::pi, kPrecision);
}

}  // namespace
}  // namespace rovi::robot_agent
