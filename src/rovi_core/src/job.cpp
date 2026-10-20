#include "rovi_core/job.hpp"

#include <algorithm>
#include <optional>

#include "rovi_core/units.hpp"

namespace rovi::core {

std::optional<TimePoint> effective_deadline(const Job& job) {
  std::optional<TimePoint> deadline = checked_sub(job.buffer_empty_time, job.safety_margin);
  if (!deadline) {
    return std::nullopt;
  }
  if (job.task_deadline) {
    deadline = std::min(*deadline, *job.task_deadline);
  }
  if (job.maximum_result_age) {
    const std::optional<TimePoint> freshness_bound =
        checked_add(job.observation_time, *job.maximum_result_age);
    if (!freshness_bound) {
      return std::nullopt;
    }
    deadline = std::min(*deadline, *freshness_bound);
  }
  return deadline;
}

std::optional<TimePoint> freshness_deadline(const Job& job) {
  if (!job.maximum_result_age) {
    return std::nullopt;
  }
  return checked_add(job.observation_time, *job.maximum_result_age);
}

}  // namespace rovi::core
