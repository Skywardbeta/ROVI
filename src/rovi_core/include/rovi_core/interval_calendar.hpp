// Sorted, disjoint, half-open reservation intervals for one non-preemptive execution lane.
#ifndef ROVI_CORE_INTERVAL_CALENDAR_HPP_
#define ROVI_CORE_INTERVAL_CALENDAR_HPP_

#include <optional>
#include <vector>

#include "rovi_core/units.hpp"

namespace rovi::core {

struct Interval {
  TimePoint start;
  TimePoint end;

  bool operator==(const Interval&) const = default;
};

class IntervalCalendar {
 public:
  IntervalCalendar() = default;

  [[nodiscard]] std::optional<Interval> earliest_fit(TimePoint not_before, Duration length) const;
  [[nodiscard]] std::optional<IntervalCalendar> reserve(Interval interval) const;
  [[nodiscard]] IntervalCalendar pruned(TimePoint cutoff) const;
  [[nodiscard]] const std::vector<Interval>& intervals() const;

  bool operator==(const IntervalCalendar&) const = default;

 private:
  explicit IntervalCalendar(std::vector<Interval> intervals);

  std::vector<Interval> intervals_;
};

}  // namespace rovi::core

#endif  // ROVI_CORE_INTERVAL_CALENDAR_HPP_
