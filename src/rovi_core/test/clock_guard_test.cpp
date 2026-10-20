#include "rovi_core/clock_guard.hpp"

#include <optional>

#include <gtest/gtest.h>

#include "test_support.hpp"

namespace rovi::core {
namespace {

TEST(ClockGuard, AcceptsAdvancingAndRepeatedTimeWithoutInvalidating) {
  ClockGuard guard;

  EXPECT_EQ(guard.observe(at_ms(0)), ClockStep::kAdvanced);
  EXPECT_EQ(guard.observe(at_ms(0)), ClockStep::kRepeated);
  EXPECT_EQ(guard.observe(at_ms(10)), ClockStep::kAdvanced);
  EXPECT_EQ(guard.observe(at_ms(10)), ClockStep::kRepeated);

  EXPECT_TRUE(guard.valid());
  EXPECT_EQ(guard.latest(), at_ms(10));
}

TEST(ClockGuard, ABackwardStepInvalidatesForGoodEvenIfTimeAdvancesAgain) {
  ClockGuard guard;
  ASSERT_EQ(guard.observe(at_ms(100)), ClockStep::kAdvanced);

  EXPECT_EQ(guard.observe(t((100 * kNsPerMs) - 1)), ClockStep::kBackward);
  EXPECT_EQ(guard.observe(at_ms(200)), ClockStep::kInvalid);
  EXPECT_EQ(guard.observe(at_ms(100)), ClockStep::kInvalid);

  EXPECT_FALSE(guard.valid());
  EXPECT_EQ(guard.latest(), at_ms(100));
}

TEST(ClockGuard, StartsValidWithNoObservation) {
  const ClockGuard guard;

  EXPECT_TRUE(guard.valid());
  EXPECT_EQ(guard.latest(), std::nullopt);
}

}  // namespace
}  // namespace rovi::core
