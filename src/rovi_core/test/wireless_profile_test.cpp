#include "rovi_core/wireless_profile.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "rovi_core/capacity_calendar.hpp"
#include "rovi_core/units.hpp"

#include "printers.hpp"  // IWYU pragma: keep
#include "test_support.hpp"

namespace rovi::core {
namespace {

ServiceSegment segment(std::int64_t first_slot, std::int64_t bits_per_second,
                       std::int32_t reduction = 0) {
  return ServiceSegment{.first_slot = first_slot,
                        .rate = BitsPerSecond{bits_per_second},
                        .reduction = Permille{reduction}};
}

std::vector<std::int64_t> capacities(const CapacityCalendar& calendar) {
  std::vector<std::int64_t> result;
  result.reserve(static_cast<std::size_t>(calendar.slot_count()));
  for (std::int64_t slot = 0; slot < calendar.slot_count(); ++slot) {
    result.push_back(calendar.capacity(SlotIndex{slot}).value());
  }
  return result;
}

TEST(WirelessProfile, ConvertsRateToWholeBytesPerSlot) {
  EXPECT_EQ(bytes_per_slot(BitsPerSecond{8'000'000}, ms(1), Permille{0}), Bytes{1'000});
  EXPECT_EQ(bytes_per_slot(BitsPerSecond{1'000'000'000}, ms(1), Permille{0}), Bytes{125'000});
  EXPECT_EQ(bytes_per_slot(BitsPerSecond{8'000'000}, ms(1), Permille{100}), Bytes{900});
  EXPECT_EQ(bytes_per_slot(BitsPerSecond{8'000'000}, ms(1), Permille{1'000}), Bytes{0});
  EXPECT_EQ(bytes_per_slot(BitsPerSecond{0}, ms(1), Permille{0}), Bytes{0});
}

TEST(WirelessProfile, RoundsEveryStepDownConservatively) {
  EXPECT_EQ(bytes_per_slot(BitsPerSecond{12'000}, ms(1), Permille{0}), Bytes{1});
  EXPECT_EQ(bytes_per_slot(BitsPerSecond{12'000}, ms(1), Permille{500}), Bytes{0});
  EXPECT_EQ(bytes_per_slot(BitsPerSecond{15'999}, ms(1), Permille{0}), Bytes{1});
}

TEST(WirelessProfile, RejectsInvalidRateSlotOrReduction) {
  EXPECT_EQ(bytes_per_slot(BitsPerSecond{-1}, ms(1), Permille{0}), std::nullopt);
  EXPECT_EQ(bytes_per_slot(BitsPerSecond{8'000}, ms(0), Permille{0}), std::nullopt);
  EXPECT_EQ(bytes_per_slot(BitsPerSecond{8'000}, ms(1), Permille{-1}), std::nullopt);
  EXPECT_EQ(bytes_per_slot(BitsPerSecond{8'000}, ms(1), Permille{1'001}), std::nullopt);
  EXPECT_EQ(bytes_per_slot(BitsPerSecond{kMaxNs}, ms(1'000), Permille{0}), std::nullopt);
}

TEST(WirelessProfile, BuildsPiecewiseCalendarWithHandoverOutageAndReassociation) {
  const std::optional<CapacityCalendar> calendar = build_service_calendar(
      TimePoint{}, ms(1), 8, {segment(0, 8'000'000), segment(3, 0), segment(5, 4'000'000, 0)});

  ASSERT_TRUE(calendar.has_value());
  EXPECT_EQ(capacities(calendar.value()),
            (std::vector<std::int64_t>{1'000, 1'000, 1'000, 0, 0, 500, 500, 500}));
}

TEST(WirelessProfile, RejectsMalformedSegmentLists) {
  EXPECT_EQ(build_service_calendar(TimePoint{}, ms(1), 4, {}), std::nullopt);
  EXPECT_EQ(build_service_calendar(TimePoint{}, ms(1), 4, {segment(1, 8'000)}), std::nullopt);
  EXPECT_EQ(build_service_calendar(TimePoint{}, ms(1), 4, {segment(0, 8'000), segment(0, 8'000)}),
            std::nullopt);
  EXPECT_EQ(build_service_calendar(TimePoint{}, ms(1), 4, {segment(0, 8'000), segment(4, 8'000)}),
            std::nullopt);
  EXPECT_EQ(build_service_calendar(TimePoint{}, ms(1), 4, {segment(0, -8'000)}), std::nullopt);
  EXPECT_EQ(build_service_calendar(TimePoint{}, ms(1), 0, {segment(0, 8'000)}), std::nullopt);
}

TEST(WirelessProfile, ServiceCapacityIndexesSegmentsByAbsoluteSlot) {
  const std::vector<ServiceSegment> profile{segment(0, 8'000'000), segment(5, 0),
                                            segment(8, 4'000'000)};

  const std::optional<std::vector<Bytes>> window = service_capacity(ms(1), profile, 3, 7);

  ASSERT_TRUE(window.has_value());
  std::vector<std::int64_t> values(window->size());
  std::ranges::transform(window.value(), values.begin(),
                         [](Bytes capacity) { return capacity.value(); });
  EXPECT_EQ(values, (std::vector<std::int64_t>{1'000, 1'000, 0, 0, 0, 500, 500}));
}

TEST(WirelessProfile, ServiceCapacityRejectsMalformedRequests) {
  const std::vector<ServiceSegment> profile{segment(0, 8'000'000)};

  EXPECT_EQ(service_capacity(ms(1), profile, 0, 0), std::nullopt);
  EXPECT_EQ(service_capacity(ms(1), profile, -1, 2), std::nullopt);
  EXPECT_EQ(service_capacity(ms(1), profile, kMaxNs, 2), std::nullopt);
  EXPECT_EQ(service_capacity(ms(1), {}, 0, 2), std::nullopt);
  EXPECT_EQ(service_capacity(ms(1), {segment(1, 8'000)}, 0, 2), std::nullopt);
  EXPECT_EQ(service_capacity(ms(1), {segment(0, 8'000), segment(0, 8'000)}, 0, 2), std::nullopt);
  EXPECT_EQ(service_capacity(ms(1), {segment(0, 8'000), segment(9, -1)}, 0, 2), std::nullopt);
  EXPECT_EQ(service_capacity(ms(0), profile, 0, 2), std::nullopt);
}

}  // namespace
}  // namespace rovi::core
