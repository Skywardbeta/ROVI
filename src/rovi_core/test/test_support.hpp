// Shared integer time constructors and limits for rovi_core tests.
#ifndef ROVI_CORE_TEST_TEST_SUPPORT_HPP_
#define ROVI_CORE_TEST_TEST_SUPPORT_HPP_

#include <cstdint>
#include <limits>

#include "rovi_core/units.hpp"

namespace rovi::core {

inline constexpr std::int64_t kMinNs = std::numeric_limits<std::int64_t>::min();
inline constexpr std::int64_t kMaxNs = std::numeric_limits<std::int64_t>::max();
inline constexpr std::int64_t kNsPerMs = 1'000'000;

inline TimePoint t(std::int64_t ns) { return TimePoint::from_ns(ns); }

inline Duration d(std::int64_t ns) { return Duration::from_ns(ns); }

inline TimePoint at_ms(std::int64_t count) { return TimePoint::from_ns(count * kNsPerMs); }

inline Duration ms(std::int64_t count) { return Duration::from_ns(count * kNsPerMs); }

}  // namespace rovi::core

#endif  // ROVI_CORE_TEST_TEST_SUPPORT_HPP_
