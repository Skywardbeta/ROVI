// Shared reference network, worker, job, and chain fixtures for admission and scheduling tests.
#ifndef ROVI_CORE_TEST_SCENARIO_HPP_
#define ROVI_CORE_TEST_SCENARIO_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "rovi_core/admission.hpp"
#include "rovi_core/capacity_calendar.hpp"
#include "rovi_core/interval_calendar.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/units.hpp"

#include "test_support.hpp"

namespace rovi::core {

inline constexpr std::int64_t kHorizonSlots = 100;
inline constexpr std::int64_t kUplinkBytesPerSlot = 125'000;
inline constexpr std::int64_t kDownlinkBytesPerSlot = 50'000;

inline CapacityCalendar uniform_calendar(std::int64_t bytes_per_slot,
                                         std::int64_t slots = kHorizonSlots) {
  return CapacityCalendar::create(
             TimePoint{}, ms(1),
             std::vector<Bytes>(static_cast<std::size_t>(slots), Bytes{bytes_per_slot}))
      .value();
}

inline Worker worker_with(std::uint64_t id, std::vector<ExecutionProfile> profiles) {
  return Worker{.id = WorkerId{id}, .profiles = std::move(profiles), .lane = IntervalCalendar{}};
}

inline Worker worker_for(std::uint64_t id, ModelClass model, Duration inference) {
  return worker_with(id, {ExecutionProfile{.model = model, .duration = inference}});
}

inline Worker worker(std::uint64_t id, Duration inference) {
  return worker_for(id, ModelClass{0}, inference);
}

inline ResourceSnapshot empty_snapshot(std::vector<Worker> workers) {
  return ResourceSnapshot{.uplink = uniform_calendar(kUplinkBytesPerSlot),
                          .downlink = uniform_calendar(kDownlinkBytesPerSlot),
                          .workers = std::move(workers)};
}

inline Job reference_job() {
  return Job{.id = JobId{1},
             .release_time = at_ms(0),
             .observation_time = at_ms(0),
             .buffer_empty_time = at_ms(50),
             .safety_margin = ms(5),
             .task_deadline = std::nullopt,
             .maximum_result_age = ms(40),
             .upload_bytes = Bytes{250'000},
             .action_chunk_bytes = Bytes{100'000},
             .model = ModelClass{0},
             .eligible_workers = {WorkerId{0}}};
}

inline Job job_numbered(std::uint64_t id) {
  Job job = reference_job();
  job.id = JobId{id};
  job.sequence = id;
  return job;
}

inline Transfer transfer_in_slots(std::int64_t first_slot, std::int64_t bytes_per_slot) {
  return Transfer{.slices = {{.slot = SlotIndex{first_slot}, .bytes = Bytes{bytes_per_slot}},
                             {.slot = SlotIndex{first_slot + 1}, .bytes = Bytes{bytes_per_slot}}},
                  .completion = at_ms(first_slot + 2)};
}

inline Interval window_ms(std::int64_t start, std::int64_t end) {
  return Interval{.start = at_ms(start), .end = at_ms(end)};
}

inline Decision outcome(DecisionStatus status, ReasonCode reason) {
  return Decision{.status = status, .reason = reason};
}

inline Decision admitted() { return outcome(DecisionStatus::kAdmitted, ReasonCode::kNone); }

inline std::vector<std::int64_t> reserved_bytes(const CapacityCalendar& calendar) {
  std::vector<std::int64_t> bytes;
  bytes.reserve(static_cast<std::size_t>(calendar.slot_count()));
  for (std::int64_t slot = 0; slot < calendar.slot_count(); ++slot) {
    bytes.push_back(calendar.reserved(SlotIndex{slot}).value());
  }
  return bytes;
}

inline std::vector<std::int64_t> expected_reserved(
    const std::vector<std::pair<std::int64_t, std::int64_t>>& slots,
    std::int64_t slot_count = kHorizonSlots) {
  std::vector<std::int64_t> bytes(static_cast<std::size_t>(slot_count), 0);
  for (const auto& [slot, amount] : slots) {
    bytes.at(static_cast<std::size_t>(slot)) = amount;
  }
  return bytes;
}

}  // namespace rovi::core

#endif  // ROVI_CORE_TEST_SCENARIO_HPP_
