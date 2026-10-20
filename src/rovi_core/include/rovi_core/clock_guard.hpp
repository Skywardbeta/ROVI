// Fail-stop check of one component's observed ROS time: repeated or paused time is accepted, and
// the first backward step invalidates the run for good, because state from before it is unusable.
#ifndef ROVI_CORE_CLOCK_GUARD_HPP_
#define ROVI_CORE_CLOCK_GUARD_HPP_

#include <cstdint>
#include <optional>

#include "rovi_core/units.hpp"

namespace rovi::core {

enum class ClockStep : std::uint8_t { kAdvanced, kRepeated, kBackward, kInvalid };

class ClockGuard {
 public:
  ClockStep observe(TimePoint now);
  [[nodiscard]] bool valid() const;
  [[nodiscard]] std::optional<TimePoint> latest() const;

 private:
  std::optional<TimePoint> latest_;
  bool valid_{true};
};

}  // namespace rovi::core

#endif  // ROVI_CORE_CLOCK_GUARD_HPP_
