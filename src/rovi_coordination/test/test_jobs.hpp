// Job and time builders shared by the coordination tests.
#ifndef ROVI_COORDINATION_TEST_JOBS_HPP_
#define ROVI_COORDINATION_TEST_JOBS_HPP_

#include <cstdint>
#include <optional>

#include "rovi_core/job.hpp"
#include "rovi_core/units.hpp"

namespace rovi::coordination {

inline constexpr std::int64_t kNsPerMs = 1'000'000;

inline core::TimePoint at_ms(std::int64_t count) {
  return core::TimePoint::from_ns(count * kNsPerMs);
}

inline core::Duration ms(std::int64_t count) { return core::Duration::from_ns(count * kNsPerMs); }

inline core::Job proposal(std::uint64_t id, std::uint32_t robot, std::int64_t deadline_ms) {
  return core::Job{.id = core::JobId{id},
                   .release_time = at_ms(0),
                   .observation_time = at_ms(0),
                   .buffer_empty_time = at_ms(deadline_ms),
                   .safety_margin = ms(0),
                   .task_deadline = std::nullopt,
                   .maximum_result_age = std::nullopt,
                   .upload_bytes = core::Bytes{1},
                   .action_chunk_bytes = core::Bytes{1},
                   .model = core::ModelClass{1},
                   .eligible_workers = {core::WorkerId{1}},
                   .robot = core::RobotId{robot},
                   .sequence = id};
}

}  // namespace rovi::coordination

#endif  // ROVI_COORDINATION_TEST_JOBS_HPP_
