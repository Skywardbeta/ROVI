#include "rovi_core/audit.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <ranges>
#include <utility>
#include <vector>

#include "rovi_core/admission.hpp"
#include "rovi_core/capacity_calendar.hpp"
#include "rovi_core/interval_calendar.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/scheduler.hpp"
#include "rovi_core/units.hpp"

namespace rovi::core {
namespace {

using Findings = std::vector<AuditFinding>;
using Lanes = std::map<WorkerId, std::vector<Interval>>;

class Ledger {
 public:
  explicit Ledger(const CapacityCalendar& calendar) {
    for (std::int64_t slot = 0; slot < calendar.slot_count(); ++slot) {
      capacity_.push_back(calendar.capacity(SlotIndex{slot}).value());
      used_.push_back(calendar.reserved(SlotIndex{slot}).value());
    }
  }

  bool add(const Transfer& transfer) {
    const bool in_range = std::ranges::all_of(transfer.slices, [this](const ServiceSlice& slice) {
      return slice.slot.value() >= 0 && slice.slot.value() < std::ssize(used_);
    });
    if (!in_range) {
      return false;
    }
    for (const ServiceSlice& slice : transfer.slices) {
      used_.at(static_cast<std::size_t>(slice.slot.value())) += slice.bytes.value();
    }
    return true;
  }

  [[nodiscard]] bool within_capacity() const {
    return std::ranges::equal(used_, capacity_, std::ranges::less_equal{});
  }

  [[nodiscard]] bool matches(const CapacityCalendar& calendar) const {
    if (calendar.slot_count() != std::ssize(used_)) {
      return false;
    }
    for (std::int64_t slot = 0; slot < calendar.slot_count(); ++slot) {
      if (calendar.reserved(SlotIndex{slot}).value() != used_.at(static_cast<std::size_t>(slot))) {
        return false;
      }
    }
    return true;
  }

 private:
  std::vector<std::int64_t> capacity_;
  std::vector<std::int64_t> used_;
};

bool ends_at_completion(const CapacityCalendar& calendar, const Transfer& transfer) {
  if (transfer.slices.empty()) {
    return false;
  }
  const std::optional<TimePoint> last_start = calendar.slot_start(transfer.slices.back().slot);
  return last_start && checked_add(*last_start, calendar.slot_length()) == transfer.completion;
}

bool stages_in_order(const ResourceSnapshot& initial, const ChainReservation& chain) {
  if (chain.download.slices.empty()) {
    return false;
  }
  const std::optional<TimePoint> received =
      checked_add(chain.upload.completion, initial.uplink_delay);
  const std::optional<TimePoint> ready =
      received ? checked_add(*received, initial.dispatch_lead) : std::nullopt;
  const std::optional<TimePoint> chunk_ready =
      checked_add(chain.compute.end, initial.dispatch_lead);
  const std::optional<TimePoint> download_start =
      initial.downlink.slot_start(chain.download.slices.front().slot);
  return ready && chunk_ready && download_start && *ready <= chain.compute.start &&
         chain.compute.start < chain.compute.end && *chunk_ready <= *download_start;
}

bool delivery_consistent(const ResourceSnapshot& initial, const ChainReservation& chain) {
  return ends_at_completion(initial.uplink, chain.upload) &&
         ends_at_completion(initial.downlink, chain.download) &&
         checked_add(chain.download.completion, initial.downlink_delay) == chain.delivery;
}

void check_chain(const ResourceSnapshot& initial, const Job& job, const ChainReservation& chain,
                 const std::vector<JobId>& completed, Findings& findings) {
  const auto flag = [&findings, &job](Violation violation) {
    findings.push_back(AuditFinding{.violation = violation, .job = job.id});
  };
  const auto worker = std::ranges::find(initial.workers, chain.worker, &Worker::id);
  if (worker == initial.workers.end()) {
    flag(Violation::kUnknownWorker);
  } else {
    const auto profile = std::ranges::find(worker->profiles, job.model, &ExecutionProfile::model);
    if (profile == worker->profiles.end() ||
        elapsed(chain.compute.start, chain.compute.end) != profile->duration) {
      flag(Violation::kComputeLengthMismatch);
    }
  }
  if (!stages_in_order(initial, chain)) {
    flag(Violation::kStageOrder);
  }
  if (!delivery_consistent(initial, chain)) {
    flag(Violation::kDeliveryMismatch);
  }
  const std::optional<TimePoint> deadline = effective_deadline(job);
  if (!deadline || chain.delivery > *deadline) {
    flag(Violation::kDeadlineExceeded);
  }
  const bool predecessors_done = std::ranges::all_of(job.predecessors, [&](JobId predecessor) {
    return std::ranges::find(completed, predecessor) != completed.end();
  });
  if (!predecessors_done) {
    flag(Violation::kPredecessorIncomplete);
  }
}

void check_lanes(Lanes lanes, const ResourceSnapshot& result, Findings& findings) {
  for (auto& [worker_id, intervals] : lanes) {
    std::ranges::sort(intervals, {}, &Interval::start);
    for (std::size_t index = 1; index < intervals.size(); ++index) {
      if (intervals.at(index - 1).end > intervals.at(index).start) {
        findings.push_back(AuditFinding{.violation = Violation::kComputeOverlap, .job = {}});
      }
    }
    const auto worker = std::ranges::find(result.workers, worker_id, &Worker::id);
    const bool matches =
        worker == result.workers.end() ? intervals.empty() : worker->lane.intervals() == intervals;
    if (!matches) {
      findings.push_back(AuditFinding{.violation = Violation::kSnapshotMismatch, .job = {}});
    }
  }
}

void check_groups(const std::vector<Job>& pending, const std::vector<JobDecision>& decisions,
                  Findings& findings) {
  std::map<GroupId, std::vector<const Job*>> groups;
  for (const Job& job : pending) {
    if (job.synchronization_group) {
      groups[*job.synchronization_group].push_back(&job);
    }
  }
  for (const std::vector<const Job*>& members : groups | std::views::values) {
    std::vector<TimePoint> deliveries;
    for (const Job* member : members) {
      const auto decision = std::ranges::find(decisions, member->id, &JobDecision::job);
      if (decision == decisions.end() || decision->decision.status != DecisionStatus::kAdmitted) {
        continue;
      }
      if (const std::optional<ChainReservation>& reservation = decision->reservation) {
        deliveries.push_back(reservation->delivery);
      }
    }
    const JobId first = members.front()->id;
    if (!deliveries.empty() && deliveries.size() != members.size()) {
      findings.push_back(AuditFinding{.violation = Violation::kGroupNotAtomic, .job = first});
      continue;
    }
    if (deliveries.size() < 2) {
      continue;
    }
    const auto [earliest, latest] = std::ranges::minmax(deliveries);
    const Duration bound = std::ranges::min(members | std::views::transform([](const Job* job) {
                                              return job->maximum_result_ready_skew;
                                            }));
    if (elapsed(earliest, latest) > bound) {
      findings.push_back(AuditFinding{.violation = Violation::kGroupSkewExceeded, .job = first});
    }
  }
}

}  // namespace

std::vector<AuditFinding> audit_schedule(const ResourceSnapshot& initial,
                                         const std::vector<Job>& pending,
                                         const std::vector<JobId>& completed,
                                         const ScheduleResult& result) {
  Findings findings;
  const auto flag = [&findings](Violation violation, JobId job) {
    findings.push_back(AuditFinding{.violation = violation, .job = job});
  };
  if (result.digest != decision_digest(result.decisions)) {
    flag(Violation::kDigestMismatch, {});
  }
  Ledger uplink{initial.uplink};
  Ledger downlink{initial.downlink};
  std::vector<Ledger> medium;
  if (initial.shared_medium) {
    medium.emplace_back(*initial.shared_medium);
  }
  Lanes lanes;
  for (const Worker& worker : initial.workers) {
    lanes[worker.id] = worker.lane.intervals();
  }
  for (const JobDecision& decision : result.decisions) {
    const auto job = std::ranges::find(pending, decision.job, &Job::id);
    if (job == pending.end()) {
      flag(Violation::kUnknownJob, decision.job);
      continue;
    }
    if (decision.decision.status != DecisionStatus::kAdmitted || !decision.reservation) {
      continue;
    }
    const ChainReservation& chain = *decision.reservation;
    check_chain(initial, *job, chain, completed, findings);
    const bool recorded = uplink.add(chain.upload) && downlink.add(chain.download) &&
                          std::ranges::all_of(medium, [&chain](Ledger& shared) {
                            return shared.add(chain.upload) && shared.add(chain.download);
                          });
    if (!recorded) {
      flag(Violation::kCapacityExceeded, decision.job);
    }
    lanes[chain.worker].push_back(chain.compute);
  }
  if (!uplink.within_capacity() || !downlink.within_capacity() ||
      !std::ranges::all_of(medium, &Ledger::within_capacity)) {
    flag(Violation::kCapacityExceeded, {});
  }
  check_lanes(std::move(lanes), result.snapshot, findings);
  check_groups(pending, result.decisions, findings);
  const bool calendars_match =
      uplink.matches(result.snapshot.uplink) && downlink.matches(result.snapshot.downlink) &&
      medium.empty() != result.snapshot.shared_medium.has_value() &&
      (medium.empty() || medium.front().matches(*result.snapshot.shared_medium));
  if (!calendars_match) {
    flag(Violation::kSnapshotMismatch, {});
  }
  return findings;
}

}  // namespace rovi::core
