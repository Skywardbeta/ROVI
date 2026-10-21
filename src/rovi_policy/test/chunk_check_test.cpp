#include "rovi_policy/chunk_check.hpp"

#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace rovi::policy {
namespace {

constexpr ChunkShape kTwoByTwo{.steps = 2, .dim = 2};

JointLimits limits() {
  return JointLimits{.lower = {-10.0F, 0.0F}, .upper = {10.0F, 100.0F}, .tolerance = 2.0F};
}

TEST(ChunkCheck, AcceptsAFiniteChunkOfTheExpectedShapeWithinLimits) {
  const JointLimits bounds = limits();
  const std::vector<float> actions{-10.0F, 0.0F, 10.0F, 100.0F};
  EXPECT_EQ(check_chunk(actions, kTwoByTwo, kTwoByTwo, &bounds), std::nullopt);
  EXPECT_EQ(check_chunk(actions, kTwoByTwo, kTwoByTwo, nullptr), std::nullopt);
}

TEST(ChunkCheck, RejectsAnotherShapeOrACountThatDoesNotMatchIt) {
  const std::vector<float> actions{1.0F, 2.0F, 3.0F, 4.0F};
  EXPECT_EQ(check_chunk(actions, {.steps = 4, .dim = 1}, kTwoByTwo, nullptr), ChunkFault::kShape);
  EXPECT_EQ(check_chunk({actions.data(), 3}, kTwoByTwo, kTwoByTwo, nullptr), ChunkFault::kShape);
  EXPECT_EQ(check_chunk({}, {.steps = 0, .dim = 0}, {.steps = 0, .dim = 0}, nullptr),
            ChunkFault::kShape);
}

TEST(ChunkCheck, RejectsNonFiniteValues) {
  for (const float bad :
       {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
    const std::vector<float> actions{1.0F, 2.0F, bad, 4.0F};
    EXPECT_EQ(check_chunk(actions, kTwoByTwo, kTwoByTwo, nullptr), ChunkFault::kNonFinite);
  }
}

TEST(ChunkCheck, AllowsTheToleranceBeyondEachLimitAndNoMore) {
  const JointLimits bounds = limits();
  EXPECT_EQ(
      check_chunk(std::vector<float>{-12.0F, -2.0F, 12.0F, 102.0F}, kTwoByTwo, kTwoByTwo, &bounds),
      std::nullopt);
  EXPECT_EQ(
      check_chunk(std::vector<float>{0.0F, 50.0F, 12.5F, 50.0F}, kTwoByTwo, kTwoByTwo, &bounds),
      ChunkFault::kOutOfRange);
  EXPECT_EQ(
      check_chunk(std::vector<float>{0.0F, -2.5F, 0.0F, 50.0F}, kTwoByTwo, kTwoByTwo, &bounds),
      ChunkFault::kOutOfRange);
}

TEST(ChunkCheck, TreatsLimitsOfAnotherDimensionAsAShapeFault) {
  const JointLimits bounds{.lower = {0.0F}, .upper = {1.0F}, .tolerance = 0.0F};
  EXPECT_EQ(check_chunk(std::vector<float>{0.0F, 0.0F, 0.0F, 0.0F}, kTwoByTwo, kTwoByTwo, &bounds),
            ChunkFault::kShape);
}

ChunkContract contract() {
  return ChunkContract{.shape = kTwoByTwo,
                       .period_ns = 33'333'333,
                       .limits = limits(),
                       .policy_id = std::string(64, 'a')};
}

TEST(ChunkCheck, AcceptsADeliveryThatMatchesEveryTermOfTheContract) {
  const std::vector<float> actions{0.0F, 50.0F, 1.0F, 51.0F};
  EXPECT_EQ(check_delivery(contract(), actions, kTwoByTwo, 33'333'333, std::string(64, 'a')),
            std::nullopt);
}

TEST(ChunkCheck, RejectsADeliveryAtAnotherPlaybackRateOrFromAnotherPolicy) {
  const std::vector<float> actions{0.0F, 50.0F, 1.0F, 51.0F};
  EXPECT_EQ(check_delivery(contract(), actions, kTwoByTwo, 16'666'666, std::string(64, 'a')),
            ChunkFault::kPeriod);
  EXPECT_EQ(check_delivery(contract(), actions, kTwoByTwo, 33'333'333, std::string(64, 'b')),
            ChunkFault::kPolicy);
  EXPECT_EQ(
      check_delivery(contract(), {actions.data(), 3}, kTwoByTwo, 33'333'333, std::string(64, 'a')),
      ChunkFault::kShape);
}

TEST(ChunkCheck, PlaybackLastsStepsTimesPeriodOrIsUnrepresentable) {
  EXPECT_EQ(playback_ns({.steps = 50, .dim = 6}, 33'333'333), 1'666'666'650);
  EXPECT_FALSE(playback_ns({.steps = 4'000'000'000U, .dim = 6}, 4'000'000'000).has_value());
  EXPECT_FALSE(playback_ns({.steps = 50, .dim = 6}, 0).has_value());
}

TEST(ChunkCheck, AContractNeedsAPositivePeriodFittingLimitsAndAPolicyId) {
  EXPECT_TRUE(contract_fits(contract()));
  ChunkContract no_period = contract();
  no_period.period_ns = 0;
  EXPECT_FALSE(contract_fits(no_period));
  ChunkContract bad_policy = contract();
  bad_policy.policy_id = "not hex";
  EXPECT_FALSE(contract_fits(bad_policy));
  ChunkContract wide = contract();
  wide.limits.lower.push_back(0.0F);
  EXPECT_FALSE(contract_fits(wide));
}

TEST(ChunkCheck, LimitsMustBeFiniteOrderedAndToleranceNonNegative) {
  EXPECT_TRUE(limits_fit(limits(), 2));
  EXPECT_FALSE(limits_fit(limits(), 3));
  EXPECT_FALSE(limits_fit({.lower = {1.0F}, .upper = {0.0F}, .tolerance = 0.0F}, 1));
  EXPECT_FALSE(limits_fit({.lower = {0.0F}, .upper = {1.0F}, .tolerance = -1.0F}, 1));
  EXPECT_FALSE(limits_fit(
      {.lower = {std::numeric_limits<float>::quiet_NaN()}, .upper = {1.0F}, .tolerance = 0.0F}, 1));
}

}  // namespace
}  // namespace rovi::policy
