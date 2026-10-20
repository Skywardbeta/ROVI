#include "rovi_core/interval_calendar.hpp"

#include <cstdint>
#include <numeric>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "printers.hpp"  // IWYU pragma: keep
#include "test_support.hpp"

namespace rovi::core {
namespace {

Interval span(std::int64_t start_ns, std::int64_t end_ns) {
  return Interval{.start = t(start_ns), .end = t(end_ns)};
}

IntervalCalendar lane_with(const std::vector<Interval>& busy) {
  return std::accumulate(busy.begin(), busy.end(), IntervalCalendar{},
                         [](const IntervalCalendar& lane, const Interval& interval) {
                           return lane.reserve(interval).value();
                         });
}

TEST(IntervalCalendar, EarliestFitOnEmptyLaneStartsAtNotBefore) {
  EXPECT_EQ(IntervalCalendar{}.earliest_fit(t(100), d(50)), span(100, 150));
}

TEST(IntervalCalendar, EarliestFitUsesGapEndingExactlyAtNextReservation) {
  EXPECT_EQ(lane_with({span(200, 300)}).earliest_fit(t(100), d(100)), span(100, 200));
}

TEST(IntervalCalendar, EarliestFitUsesGapBetweenReservations) {
  const IntervalCalendar lane = lane_with({span(0, 10), span(30, 40)});

  EXPECT_EQ(lane.earliest_fit(t(0), d(20)), span(10, 30));
  EXPECT_EQ(lane.earliest_fit(t(0), d(21)), span(40, 61));
}

TEST(IntervalCalendar, EarliestFitSkipsPastOverlappingReservations) {
  EXPECT_EQ(lane_with({span(120, 300), span(320, 400)}).earliest_fit(t(100), d(50)),
            span(400, 450));
}

TEST(IntervalCalendar, EarliestFitIgnoresReservationsEndingBeforeNotBefore) {
  EXPECT_EQ(lane_with({span(0, 50)}).earliest_fit(t(100), d(10)), span(100, 110));
}

TEST(IntervalCalendar, EarliestFitRejectsNonPositiveLength) {
  EXPECT_EQ(IntervalCalendar{}.earliest_fit(t(0), d(0)), std::nullopt);
  EXPECT_EQ(IntervalCalendar{}.earliest_fit(t(0), d(-1)), std::nullopt);
}

TEST(IntervalCalendar, EarliestFitReportsOverflowInsteadOfWrapping) {
  EXPECT_EQ(IntervalCalendar{}.earliest_fit(t(kMaxNs - 5), d(10)), std::nullopt);
  EXPECT_EQ(lane_with({span(0, kMaxNs - 5)}).earliest_fit(t(0), d(10)), std::nullopt);
  EXPECT_EQ(
      lane_with({span(0, kMaxNs - 5), span(kMaxNs - 3, kMaxNs - 2)}).earliest_fit(t(0), d(10)),
      std::nullopt);
}

TEST(IntervalCalendar, ReserveKeepsReservationsSortedAndAllowsTouchingIntervals) {
  const IntervalCalendar lane = lane_with({span(300, 400), span(100, 200), span(200, 300)});

  EXPECT_EQ(lane.intervals(),
            (std::vector<Interval>{span(100, 200), span(200, 300), span(300, 400)}));
}

TEST(IntervalCalendar, ReserveRejectsOverlapAndLeavesLaneUnchanged) {
  const IntervalCalendar lane = lane_with({span(100, 200), span(300, 400)});

  EXPECT_EQ(lane.reserve(span(150, 250)), std::nullopt);
  EXPECT_EQ(lane.reserve(span(250, 350)), std::nullopt);
  EXPECT_EQ(lane.reserve(span(100, 200)), std::nullopt);
  EXPECT_EQ(lane.reserve(span(120, 180)), std::nullopt);
  EXPECT_EQ(lane.reserve(span(50, 450)), std::nullopt);
  EXPECT_EQ(lane.intervals(), (std::vector<Interval>{span(100, 200), span(300, 400)}));
}

TEST(IntervalCalendar, ReserveRejectsEmptyOrInvertedInterval) {
  EXPECT_EQ(IntervalCalendar{}.reserve(span(100, 100)), std::nullopt);
  EXPECT_EQ(IntervalCalendar{}.reserve(span(200, 100)), std::nullopt);
}

TEST(IntervalCalendar, PrunedDropsOnlyIntervalsEndingAtOrBeforeTheCutoff) {
  const IntervalCalendar lane = lane_with({span(0, 200), span(200, 500), span(600, 900)});

  EXPECT_EQ(lane.pruned(t(500)).intervals(), (std::vector<Interval>{span(600, 900)}));
  EXPECT_EQ(lane.pruned(t(499)).intervals(),
            (std::vector<Interval>{span(200, 500), span(600, 900)}));
  EXPECT_EQ(lane.pruned(t(0)), lane);
  EXPECT_EQ(lane.pruned(t(900)), IntervalCalendar{});
}

}  // namespace
}  // namespace rovi::core
