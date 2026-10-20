#include "rovi_core/capacity_calendar.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "rovi_core/units.hpp"

#include "printers.hpp"  // IWYU pragma: keep
#include "test_support.hpp"

namespace rovi::core {
namespace {

constexpr std::int64_t kSlotNs = 10;

std::vector<Bytes> bytes(const std::vector<std::int64_t>& values) {
  std::vector<Bytes> result(values.size());
  std::ranges::transform(values, result.begin(), [](std::int64_t value) { return Bytes{value}; });
  return result;
}

CapacityCalendar calendar_at(TimePoint origin, const std::vector<std::int64_t>& capacity) {
  return CapacityCalendar::create(origin, d(kSlotNs), bytes(capacity)).value();
}

CapacityCalendar calendar(const std::vector<std::int64_t>& capacity) {
  return calendar_at(TimePoint{}, capacity);
}

ServiceSlice slice(std::int64_t slot, std::int64_t amount) {
  return ServiceSlice{.slot = SlotIndex{slot}, .bytes = Bytes{amount}};
}

Transfer transfer(std::vector<ServiceSlice> slices, std::int64_t completion_ns) {
  return Transfer{.slices = std::move(slices), .completion = t(completion_ns)};
}

TEST(CapacityCalendar, CreateRejectsInvalidConfiguration) {
  EXPECT_EQ(CapacityCalendar::create(t(0), d(0), bytes({100})), std::nullopt);
  EXPECT_EQ(CapacityCalendar::create(t(0), d(-10), bytes({100})), std::nullopt);
  EXPECT_EQ(CapacityCalendar::create(t(0), d(kSlotNs), bytes({})), std::nullopt);
  EXPECT_EQ(CapacityCalendar::create(t(0), d(kSlotNs), bytes({100, -1})), std::nullopt);
  EXPECT_EQ(CapacityCalendar::create(t(kMaxNs - 5), d(kSlotNs), bytes({100})), std::nullopt);
  EXPECT_EQ(CapacityCalendar::create(t(0), d(kMaxNs), bytes({100, 100})), std::nullopt);
}

TEST(CapacityCalendar, CreateAcceptsZeroCapacityOutageSlots) {
  EXPECT_TRUE(CapacityCalendar::create(t(0), d(kSlotNs), bytes({0, 0})).has_value());
}

TEST(CapacityCalendar, CreateAcceptsHorizonEndingExactlyAtMaximumTime) {
  const std::optional<CapacityCalendar> link =
      CapacityCalendar::create(t(kMaxNs - kSlotNs), d(kSlotNs), bytes({100}));

  ASSERT_TRUE(link.has_value());
  EXPECT_EQ(link.value().plan_transfer(t(kMaxNs - kSlotNs), Bytes{1}),
            transfer({slice(0, 1)}, kMaxNs));
}

TEST(CapacityCalendar, PlansTransferAcrossConsecutiveSlotsFromEarliestStart) {
  const CapacityCalendar link = calendar({100, 100, 100, 100});

  EXPECT_EQ(link.plan_transfer(t(0), Bytes{250}),
            transfer({slice(0, 100), slice(1, 100), slice(2, 50)}, 30));
}

TEST(CapacityCalendar, PlanSkipsSlotsWithoutRemainingCapacity) {
  const CapacityCalendar link =
      calendar({100, 0, 100, 100}).reserve(transfer({slice(2, 60)}, 30)).value();

  EXPECT_EQ(link.plan_transfer(t(0), Bytes{150}),
            transfer({slice(0, 100), slice(2, 40), slice(3, 10)}, 40));
}

TEST(CapacityCalendar, PlanStartsAtFirstSlotBoundaryAtOrAfterEarliestStart) {
  const CapacityCalendar link = calendar({100, 100, 100, 100});
  const CapacityCalendar shifted = calendar_at(t(1'000), {100, 100});

  EXPECT_EQ(link.plan_transfer(t(-5), Bytes{10}), transfer({slice(0, 10)}, 10));
  EXPECT_EQ(link.plan_transfer(t(20), Bytes{10}), transfer({slice(2, 10)}, 30));
  EXPECT_EQ(link.plan_transfer(t(15), Bytes{10}), transfer({slice(2, 10)}, 30));
  EXPECT_EQ(link.plan_transfer(t(30), Bytes{10}), transfer({slice(3, 10)}, 40));
  EXPECT_EQ(shifted.plan_transfer(t(1'005), Bytes{10}), transfer({slice(1, 10)}, 1'020));
}

TEST(CapacityCalendar, PlanRoundsUnalignedStartsUpWithNegativeOrigin) {
  const CapacityCalendar link = calendar_at(t(-25), {100, 100, 100, 100});

  EXPECT_EQ(link.plan_transfer(t(-24), Bytes{10}), transfer({slice(1, 10)}, -5));
  EXPECT_EQ(link.plan_transfer(t(-15), Bytes{10}), transfer({slice(1, 10)}, -5));
  EXPECT_EQ(link.plan_transfer(t(5), Bytes{10}), transfer({slice(3, 10)}, 15));
  EXPECT_EQ(link.plan_transfer(t(6), Bytes{10}), std::nullopt);
}

TEST(CapacityCalendar, PlanReturnsNulloptWhenHorizonCannotCarryDemand) {
  const CapacityCalendar link = calendar({100, 100, 100, 100});

  EXPECT_EQ(link.plan_transfer(t(0), Bytes{401}), std::nullopt);
  EXPECT_EQ(link.plan_transfer(t(31), Bytes{1}), std::nullopt);
  EXPECT_EQ(link.plan_transfer(t(40), Bytes{1}), std::nullopt);
  EXPECT_EQ(link.plan_transfer(t(kMaxNs), Bytes{1}), std::nullopt);
}

TEST(CapacityCalendar, PlanHandlesExtremeTimesWithoutOverflow) {
  const CapacityCalendar link = calendar_at(t(kMinNs), {100});

  EXPECT_EQ(link.plan_transfer(t(kMaxNs), Bytes{1}), std::nullopt);
  EXPECT_EQ(link.plan_transfer(t(kMinNs), Bytes{1}), transfer({slice(0, 1)}, kMinNs + kSlotNs));
}

TEST(CapacityCalendar, PlanRejectsNonPositiveDemand) {
  const CapacityCalendar link = calendar({100});

  EXPECT_EQ(link.plan_transfer(t(0), Bytes{0}), std::nullopt);
  EXPECT_EQ(link.plan_transfer(t(0), Bytes{-1}), std::nullopt);
}

TEST(CapacityCalendar, SlotStartReportsStartOfSlotsInsideHorizon) {
  const CapacityCalendar link = calendar_at(t(1'000), {100, 100, 100});

  EXPECT_EQ(link.slot_start(SlotIndex{0}), t(1'000));
  EXPECT_EQ(link.slot_start(SlotIndex{2}), t(1'020));
  EXPECT_EQ(link.slot_start(SlotIndex{3}), std::nullopt);
  EXPECT_EQ(link.slot_start(SlotIndex{-1}), std::nullopt);
}

TEST(CapacityCalendar, ReserveReturnsUpdatedCopyAndLeavesOriginalUnchanged) {
  const CapacityCalendar original = calendar({100, 100});

  const CapacityCalendar updated =
      original.reserve(transfer({slice(0, 70), slice(1, 30)}, 20)).value();

  EXPECT_EQ(updated.reserved(SlotIndex{0}), Bytes{70});
  EXPECT_EQ(updated.reserved(SlotIndex{1}), Bytes{30});
  EXPECT_EQ(original.reserved(SlotIndex{0}), Bytes{0});
  EXPECT_EQ(original.reserved(SlotIndex{1}), Bytes{0});
}

TEST(CapacityCalendar, ReserveRejectsOverCommitment) {
  const CapacityCalendar full = calendar({100}).reserve(transfer({slice(0, 100)}, 10)).value();

  EXPECT_EQ(full.reserve(transfer({slice(0, 1)}, 10)), std::nullopt);
  EXPECT_EQ(calendar({100}).reserve(transfer({slice(0, 101)}, 10)), std::nullopt);
  EXPECT_EQ(full.reserved(SlotIndex{0}), Bytes{100});
}

TEST(CapacityCalendar, ReserveRejectsSlicesOutsideHorizonOrWithoutBytes) {
  const CapacityCalendar link = calendar({100, 100});

  EXPECT_EQ(link.reserve(transfer({slice(-1, 10)}, 0)), std::nullopt);
  EXPECT_EQ(link.reserve(transfer({slice(2, 10)}, 30)), std::nullopt);
  EXPECT_EQ(link.reserve(transfer({slice(0, 0)}, 10)), std::nullopt);
  EXPECT_EQ(link.reserve(transfer({slice(0, -5)}, 10)), std::nullopt);
}

TEST(CapacityCalendar, ReserveRejectsTransferInconsistentWithCalendarGeometry) {
  const CapacityCalendar link = calendar({100, 100, 100, 100});
  const Transfer from_shifted_calendar =
      calendar_at(t(1'000), {100, 100, 100, 100}).plan_transfer(t(1'000), Bytes{10}).value();

  EXPECT_EQ(link.reserve(transfer({}, 0)), std::nullopt);
  EXPECT_EQ(link.reserve(transfer({slice(1, 10), slice(0, 10)}, 10)), std::nullopt);
  EXPECT_EQ(link.reserve(transfer({slice(0, 10), slice(0, 10)}, 10)), std::nullopt);
  EXPECT_EQ(link.reserve(transfer({slice(0, 10)}, 20)), std::nullopt);
  EXPECT_EQ(link.reserve(from_shifted_calendar), std::nullopt);
  EXPECT_TRUE(link.reserve(transfer({slice(0, 10), slice(2, 10)}, 30)).has_value());
}

TEST(CapacityCalendar, ReservedQueryOutsideHorizonThrows) {
  const CapacityCalendar link = calendar({100});

  EXPECT_THROW(static_cast<void>(link.reserved(SlotIndex{1})), std::out_of_range);
  EXPECT_THROW(static_cast<void>(link.reserved(SlotIndex{-1})), std::out_of_range);
  EXPECT_THROW(static_cast<void>(link.capacity(SlotIndex{1})), std::out_of_range);
}

TEST(CapacityCalendar, PlanBoundedByLimitUsesTheSmallerResidualAndNeedsMatchingGeometry) {
  const CapacityCalendar link = calendar({100, 100, 100});
  const CapacityCalendar limit = calendar({40, 100, 100});

  EXPECT_EQ(link.plan_transfer(t(0), Bytes{140}, limit),
            transfer({slice(0, 40), slice(1, 100)}, 20));
  EXPECT_EQ(link.plan_transfer(t(0), Bytes{10}, calendar({100, 100})), std::nullopt);
  EXPECT_EQ(link.plan_transfer(t(0), Bytes{10}, calendar_at(t(5), {100, 100, 100})), std::nullopt);
}

TEST(CapacityCalendar, PlanBoundedByEveryLimitUsesTheSmallestResidualInEachSlot) {
  const CapacityCalendar link = calendar({100, 100, 100});
  const CapacityCalendar first = calendar({40, 100, 30});
  const CapacityCalendar second = calendar({100, 20, 100});
  const std::array<const CapacityCalendar*, 2> limits{&first, &second};
  EXPECT_EQ(link.plan_transfer(t(0), Bytes{90}, limits),
            transfer({slice(0, 40), slice(1, 20), slice(2, 30)}, 30));
  EXPECT_EQ(link.plan_transfer(t(0), Bytes{91}, limits), std::nullopt);

  const CapacityCalendar shorter = calendar({100, 100});
  const std::array<const CapacityCalendar*, 2> missing{&first, nullptr};
  const std::array<const CapacityCalendar*, 2> mismatched{&first, &shorter};
  EXPECT_EQ(link.plan_transfer(t(0), Bytes{10}, missing), std::nullopt);
  EXPECT_EQ(link.plan_transfer(t(0), Bytes{10}, mismatched), std::nullopt);
}

std::vector<std::int64_t> capacities_of(const CapacityCalendar& link) {
  std::vector<std::int64_t> result;
  result.reserve(static_cast<std::size_t>(link.slot_count()));
  for (std::int64_t slot = 0; slot < link.slot_count(); ++slot) {
    result.push_back(link.capacity(SlotIndex{slot}).value());
  }
  return result;
}

std::vector<std::int64_t> reservations_of(const CapacityCalendar& link) {
  std::vector<std::int64_t> result;
  result.reserve(static_cast<std::size_t>(link.slot_count()));
  for (std::int64_t slot = 0; slot < link.slot_count(); ++slot) {
    result.push_back(link.reserved(SlotIndex{slot}).value());
  }
  return result;
}

TEST(CapacityCalendar, AdvanceDropsLeadingSlotsKeepsReservationsAndAppendsCapacity) {
  const CapacityCalendar link =
      calendar({10, 20, 30}).reserve(transfer({slice(1, 5), slice(2, 7)}, 30)).value();

  const std::optional<CapacityCalendar> shifted = link.advance(SlotIndex{1}, bytes({40, 50}));

  ASSERT_TRUE(shifted.has_value());
  EXPECT_EQ(shifted->slot_start(SlotIndex{0}), t(10));
  EXPECT_EQ(shifted->horizon_end(), t(50));
  EXPECT_EQ(capacities_of(shifted.value()), (std::vector<std::int64_t>{20, 30, 40, 50}));
  EXPECT_EQ(reservations_of(shifted.value()), (std::vector<std::int64_t>{5, 7, 0, 0}));
}

TEST(CapacityCalendar, AdvanceCoversNoShiftAndAFullShift) {
  const CapacityCalendar link = calendar({10, 20});

  const std::optional<CapacityCalendar> appended = link.advance(SlotIndex{0}, bytes({30}));
  const std::optional<CapacityCalendar> replaced = link.advance(SlotIndex{2}, bytes({40}));

  ASSERT_TRUE(appended.has_value());
  EXPECT_EQ(appended->slot_start(SlotIndex{0}), t(0));
  EXPECT_EQ(capacities_of(appended.value()), (std::vector<std::int64_t>{10, 20, 30}));
  ASSERT_TRUE(replaced.has_value());
  EXPECT_EQ(replaced->slot_start(SlotIndex{0}), t(20));
  EXPECT_EQ(capacities_of(replaced.value()), (std::vector<std::int64_t>{40}));
}

TEST(CapacityCalendar, AdvanceRejectsInvalidShiftCapacityOrHorizon) {
  const CapacityCalendar link = calendar({10, 20});

  EXPECT_EQ(link.advance(SlotIndex{-1}, bytes({30})), std::nullopt);
  EXPECT_EQ(link.advance(SlotIndex{3}, bytes({30})), std::nullopt);
  EXPECT_EQ(link.advance(SlotIndex{2}, {}), std::nullopt);
  EXPECT_EQ(link.advance(SlotIndex{1}, bytes({-1})), std::nullopt);
  EXPECT_EQ(calendar_at(t(kMaxNs - (2 * kSlotNs)), {10, 20}).advance(SlotIndex{0}, bytes({30})),
            std::nullopt);
}

}  // namespace
}  // namespace rovi::core
