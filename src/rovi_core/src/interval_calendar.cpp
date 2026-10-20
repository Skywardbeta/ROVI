#include "rovi_core/interval_calendar.hpp"

#include <algorithm>
#include <iterator>
#include <optional>
#include <utility>
#include <vector>

#include "rovi_core/units.hpp"

namespace rovi::core {
namespace {

std::optional<Interval> interval_from(TimePoint start, Duration length) {
  const std::optional<TimePoint> end = checked_add(start, length);
  if (!end) {
    return std::nullopt;
  }
  return Interval{.start = start, .end = *end};
}

}  // namespace

IntervalCalendar::IntervalCalendar(std::vector<Interval> intervals)
    : intervals_{std::move(intervals)} {}

std::optional<Interval> IntervalCalendar::earliest_fit(TimePoint not_before,
                                                       Duration length) const {
  if (length.ns() <= 0) {
    return std::nullopt;
  }
  TimePoint start = not_before;
  for (const Interval& busy : intervals_) {
    const std::optional<Interval> candidate = interval_from(start, length);
    if (!candidate || candidate->end <= busy.start) {
      return candidate;
    }
    start = std::max(start, busy.end);
  }
  return interval_from(start, length);
}

std::optional<IntervalCalendar> IntervalCalendar::reserve(Interval interval) const {
  if (interval.end <= interval.start) {
    return std::nullopt;
  }
  const auto next = std::ranges::lower_bound(intervals_, interval.start, {}, &Interval::start);
  const bool overlaps_next = next != intervals_.end() && next->start < interval.end;
  const bool overlaps_previous =
      next != intervals_.begin() && std::prev(next)->end > interval.start;
  if (overlaps_next || overlaps_previous) {
    return std::nullopt;
  }
  std::vector<Interval> next_intervals = intervals_;
  next_intervals.insert(next_intervals.begin() + std::distance(intervals_.begin(), next), interval);
  return IntervalCalendar{std::move(next_intervals)};
}

IntervalCalendar IntervalCalendar::pruned(TimePoint cutoff) const {
  const auto first_live = std::ranges::partition_point(
      intervals_, [cutoff](const Interval& busy) { return busy.end <= cutoff; });
  return IntervalCalendar{std::vector<Interval>{first_live, intervals_.end()}};
}

const std::vector<Interval>& IntervalCalendar::intervals() const { return intervals_; }

}  // namespace rovi::core
