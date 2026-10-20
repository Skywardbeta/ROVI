// Strong integer time, data-size, and identifier types with overflow-checked time arithmetic.
#ifndef ROVI_CORE_UNITS_HPP_
#define ROVI_CORE_UNITS_HPP_

#include <compare>
#include <concepts>
#include <cstdint>
#include <optional>

namespace rovi::core {

template <typename Tag, std::integral Rep>
class StrongValue {
 public:
  constexpr StrongValue() = default;
  explicit constexpr StrongValue(Rep value) : value_{value} {}

  [[nodiscard]] constexpr Rep value() const { return value_; }

  constexpr auto operator<=>(const StrongValue&) const = default;

 private:
  Rep value_{};
};

using Bytes = StrongValue<struct BytesTag, std::int64_t>;
using SlotIndex = StrongValue<struct SlotIndexTag, std::int64_t>;
using JobId = StrongValue<struct JobIdTag, std::uint64_t>;
using WorkerId = StrongValue<struct WorkerIdTag, std::uint64_t>;
using ModelClass = StrongValue<struct ModelClassTag, std::uint32_t>;
using RobotId = StrongValue<struct RobotIdTag, std::uint32_t>;
using GroupId = StrongValue<struct GroupIdTag, std::uint32_t>;
using BitsPerSecond = StrongValue<struct BitsPerSecondTag, std::int64_t>;
using Permille = StrongValue<struct PermilleTag, std::int32_t>;

class Duration {
 public:
  constexpr Duration() = default;

  [[nodiscard]] static constexpr Duration from_ns(std::int64_t ns) { return Duration{ns}; }
  [[nodiscard]] constexpr std::int64_t ns() const { return ns_; }

  constexpr auto operator<=>(const Duration&) const = default;

 private:
  explicit constexpr Duration(std::int64_t ns) : ns_{ns} {}

  std::int64_t ns_{0};
};

class TimePoint {
 public:
  constexpr TimePoint() = default;

  [[nodiscard]] static constexpr TimePoint from_ns(std::int64_t ns) { return TimePoint{ns}; }
  [[nodiscard]] constexpr std::int64_t ns() const { return ns_; }

  constexpr auto operator<=>(const TimePoint&) const = default;

 private:
  explicit constexpr TimePoint(std::int64_t ns) : ns_{ns} {}

  std::int64_t ns_{0};
};

[[nodiscard]] constexpr std::optional<TimePoint> checked_add(TimePoint time, Duration offset) {
  std::int64_t sum{};
  if (__builtin_add_overflow(time.ns(), offset.ns(), &sum)) {
    return std::nullopt;
  }
  return TimePoint::from_ns(sum);
}

[[nodiscard]] constexpr std::optional<TimePoint> checked_sub(TimePoint time, Duration offset) {
  std::int64_t difference{};
  if (__builtin_sub_overflow(time.ns(), offset.ns(), &difference)) {
    return std::nullopt;
  }
  return TimePoint::from_ns(difference);
}

[[nodiscard]] constexpr std::optional<Duration> elapsed(TimePoint from, TimePoint to) {
  std::int64_t difference{};
  if (__builtin_sub_overflow(to.ns(), from.ns(), &difference)) {
    return std::nullopt;
  }
  return Duration::from_ns(difference);
}

[[nodiscard]] constexpr bool elapsed_beyond(TimePoint since, Duration span, TimePoint now) {
  const std::optional<TimePoint> limit = checked_add(since, span);
  return limit && *limit < now;
}

}  // namespace rovi::core

#endif  // ROVI_CORE_UNITS_HPP_
