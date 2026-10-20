#include "rovi_core/clock_guard.hpp"

#include <optional>

#include "rovi_core/units.hpp"

namespace rovi::core {

ClockStep ClockGuard::observe(TimePoint now) {
  if (!valid_) {
    return ClockStep::kInvalid;
  }
  if (latest_ && now < *latest_) {
    valid_ = false;
    return ClockStep::kBackward;
  }
  const bool repeated = latest_ == now;
  latest_ = now;
  return repeated ? ClockStep::kRepeated : ClockStep::kAdvanced;
}

bool ClockGuard::valid() const { return valid_; }

std::optional<TimePoint> ClockGuard::latest() const { return latest_; }

}  // namespace rovi::core
