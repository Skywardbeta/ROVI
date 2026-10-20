#include "rovi_core/units.hpp"

#include <optional>

#include <gtest/gtest.h>

#include "printers.hpp"  // IWYU pragma: keep
#include "test_support.hpp"

namespace rovi::core {
namespace {

TEST(CheckedTimeArithmetic, AddsSignedDurationToTimePoint) {
  EXPECT_EQ(checked_add(t(1'000), d(250)), t(1'250));
  EXPECT_EQ(checked_add(t(1'000), d(-250)), t(750));
}

TEST(CheckedTimeArithmetic, SubtractsSignedDurationFromTimePoint) {
  EXPECT_EQ(checked_sub(t(1'000), d(250)), t(750));
  EXPECT_EQ(checked_sub(t(1'000), d(-250)), t(1'250));
}

TEST(CheckedTimeArithmetic, ElapsedIsSignedDifferenceFromFirstToSecond) {
  EXPECT_EQ(elapsed(t(100), t(350)), d(250));
  EXPECT_EQ(elapsed(t(350), t(100)), d(-250));
}

TEST(CheckedTimeArithmetic, ReportsOverflowInsteadOfWrapping) {
  EXPECT_EQ(checked_add(t(kMaxNs), d(1)), std::nullopt);
  EXPECT_EQ(checked_add(t(kMinNs), d(-1)), std::nullopt);
  EXPECT_EQ(checked_sub(t(kMinNs), d(1)), std::nullopt);
  EXPECT_EQ(checked_sub(t(kMaxNs), d(-1)), std::nullopt);
  EXPECT_EQ(elapsed(t(kMinNs), t(kMaxNs)), std::nullopt);
  EXPECT_EQ(elapsed(t(kMaxNs), t(kMinNs)), std::nullopt);
}

TEST(CheckedTimeArithmetic, ElapsedBeyondIsStrictAndTreatsAnUnrepresentableLimitAsNever) {
  EXPECT_FALSE(elapsed_beyond(t(100), d(50), t(150)));
  EXPECT_TRUE(elapsed_beyond(t(100), d(50), t(151)));
  EXPECT_FALSE(elapsed_beyond(t(kMaxNs - 1), d(10), t(kMaxNs)));
  EXPECT_TRUE(elapsed_beyond(t(kMinNs), d(0), t(kMinNs + 1)));
}

}  // namespace
}  // namespace rovi::core
