#include "rovi_robot_agent/joint_calibration.hpp"

#include <limits>
#include <numbers>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

namespace rovi::robot_agent {
namespace {

constexpr double kDegree = std::numbers::pi / 180.0;

JointCalibration joint(double lower, double upper) {
  return JointCalibration{.sign = 1.0, .zero_deg = 0.0, .lower_rad = lower, .upper_rad = upper};
}

TEST(CalibrationMap, ConvertsPolicyDegreesToRadiansByDefault) {
  const CalibrationMap map = *CalibrationMap::create({joint(-2.0, 2.0), joint(-0.2, 1.8)});
  const std::optional<MappedPose> pose = map.to_radians(std::vector<float>{-45.0F, 50.0F});
  ASSERT_TRUE(pose.has_value());
  EXPECT_DOUBLE_EQ(pose->radians.at(0), -45.0 * kDegree);
  EXPECT_DOUBLE_EQ(pose->radians.at(1), 50.0 * kDegree);
  EXPECT_FALSE(pose->clamped);
}

TEST(CalibrationMap, AppliesEachJointsSignAndZero) {
  const CalibrationMap map = *CalibrationMap::create(
      {JointCalibration{.sign = -1.0, .zero_deg = 10.0, .lower_rad = -2.0, .upper_rad = 2.0}});
  EXPECT_DOUBLE_EQ(map.to_radians(std::vector<float>{30.0F})->radians.at(0), -20.0 * kDegree);
}

TEST(CalibrationMap, ClampsToTheSimulatorJointLimitsAndSaysSo) {
  const CalibrationMap map = *CalibrationMap::create({joint(-1.0, 1.0), joint(-1.0, 1.0)});
  const std::optional<MappedPose> pose = map.to_radians(std::vector<float>{90.0F, -90.0F});
  ASSERT_TRUE(pose.has_value());
  EXPECT_DOUBLE_EQ(pose->radians.at(0), 1.0);
  EXPECT_DOUBLE_EQ(pose->radians.at(1), -1.0);
  EXPECT_TRUE(pose->clamped);
  EXPECT_FALSE(map.to_radians(std::vector<float>{0.0F, 57.0F})->clamped);
}

TEST(CalibrationMap, RefusesValuesOfAnotherCountOrNonFiniteValues) {
  const CalibrationMap map = *CalibrationMap::create({joint(-1.0, 1.0), joint(-1.0, 1.0)});
  EXPECT_FALSE(map.to_radians(std::vector<float>{1.0F}).has_value());
  EXPECT_FALSE(map.to_radians(std::vector<float>{1.0F, std::numeric_limits<float>::quiet_NaN()})
                   .has_value());
}

TEST(CalibrationMap, RefusesAnEmptyMapNonUnitSignsAndUnorderedOrNonFiniteValues) {
  EXPECT_FALSE(CalibrationMap::create({}).has_value());
  EXPECT_FALSE(
      CalibrationMap::create(
          {JointCalibration{.sign = 2.0, .zero_deg = 0.0, .lower_rad = -1.0, .upper_rad = 1.0}})
          .has_value());
  EXPECT_FALSE(CalibrationMap::create({joint(1.0, -1.0)}).has_value());
  EXPECT_FALSE(
      CalibrationMap::create({joint(-std::numeric_limits<double>::infinity(), 1.0)}).has_value());
  EXPECT_FALSE(
      CalibrationMap::create({JointCalibration{.sign = 1.0,
                                               .zero_deg = std::numeric_limits<double>::quiet_NaN(),
                                               .lower_rad = -1.0,
                                               .upper_rad = 1.0}})
          .has_value());
}

}  // namespace
}  // namespace rovi::robot_agent
