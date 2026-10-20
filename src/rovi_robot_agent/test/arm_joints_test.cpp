#include "rovi_robot_agent/arm_joints.hpp"

#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace rovi::robot_agent {
namespace {

std::vector<std::string> three_joints() { return {"j1", "j2", "j3"}; }

TEST(OrderedPositions, ReordersReportedJointsIntoTheConfiguredOrder) {
  const std::optional<std::vector<double>> ordered =
      ordered_positions({"j3", "finger", "j1", "j2"}, {0.3, 9.0, 0.1, 0.2}, three_joints());

  ASSERT_TRUE(ordered.has_value());
  const std::vector<double> expected{0.1, 0.2, 0.3};
  EXPECT_EQ(*ordered, expected);
}

TEST(OrderedPositions, RejectsAMissingJointOrMismatchedArrays) {
  EXPECT_FALSE(ordered_positions({"j1", "j2"}, {0.1, 0.2}, three_joints()).has_value());
  EXPECT_FALSE(ordered_positions({"j1", "j2", "j3"}, {0.1, 0.2}, three_joints()).has_value());
  EXPECT_FALSE(
      ordered_positions({"j1", "j2", "j3"}, {0.1, 0.2, 0.3, 0.4}, three_joints()).has_value());
}

TEST(OrderedPositions, RejectsANonFiniteConfiguredJointButIgnoresOthers) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  EXPECT_FALSE(ordered_positions({"j1", "j2", "j3"}, {0.1, nan, 0.3}, three_joints()).has_value());
  EXPECT_FALSE(ordered_positions({"j1", "j2", "j3"}, {0.1, 0.2, inf}, three_joints()).has_value());
  EXPECT_TRUE(ordered_positions({"j1", "j2", "j3", "finger"}, {0.1, 0.2, 0.3, nan}, three_joints())
                  .has_value());
}

TEST(OrderedPositions, UsesTheFirstReportOfADuplicatedName) {
  const std::optional<std::vector<double>> ordered =
      ordered_positions({"j1", "j2", "j3", "j1"}, {0.1, 0.2, 0.3, 7.0}, three_joints());

  ASSERT_TRUE(ordered.has_value());
  EXPECT_DOUBLE_EQ(ordered->front(), 0.1);
}

TEST(SplitGoals, SplitsAFlatArrayIntoOneGoalPerJointCount) {
  const std::optional<std::vector<std::vector<double>>> goals =
      split_goals({1.0, 2.0, 3.0, 4.0, 5.0, 6.0}, 3);

  ASSERT_TRUE(goals.has_value());
  const std::vector<std::vector<double>> expected{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
  EXPECT_EQ(*goals, expected);
}

TEST(SplitGoals, RejectsEmptyOrRaggedInput) {
  EXPECT_FALSE(split_goals({}, 3).has_value());
  EXPECT_FALSE(split_goals({1.0, 2.0, 3.0, 4.0}, 3).has_value());
  EXPECT_FALSE(split_goals({1.0, 2.0, 3.0}, 0).has_value());
}

TEST(ValidJointNames, AcceptsDistinctTopicSafeNames) {
  EXPECT_TRUE(valid_joint_names({"panda_joint1", "panda_joint2"}));
}

TEST(ValidJointNames, RejectsEmptyListsEmptyOrDuplicatedNamesAndUnsafeCharacters) {
  EXPECT_FALSE(valid_joint_names({}));
  EXPECT_FALSE(valid_joint_names({"panda_joint1", ""}));
  EXPECT_FALSE(valid_joint_names({"panda_joint1", "panda_joint1"}));
  EXPECT_FALSE(valid_joint_names({"panda/joint1"}));
  EXPECT_FALSE(valid_joint_names({"panda joint1"}));
  EXPECT_FALSE(valid_joint_names({"1panda"}));
  EXPECT_FALSE(valid_joint_names({"panda~joint"}));
}

}  // namespace
}  // namespace rovi::robot_agent
