#include "rovi_core/admission.hpp"

#include <algorithm>
#include <functional>
#include <optional>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

#include "rovi_core/capacity_calendar.hpp"
#include "rovi_core/interval_calendar.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/units.hpp"

namespace rovi::core {
namespace {

struct EligibleExecution {
  std::reference_wrapper<const Worker> worker;
  Duration duration;
};

struct Candidate {
  WorkerId worker;
  Interval compute;
  Transfer download;
  TimePoint delivery;
};

std::optional<ReasonCode> invalid_input(const Job& job, TimePoint now) {
  if (job.upload_bytes.value() <= 0 || job.action_chunk_bytes.value() <= 0) {
    return ReasonCode::kInvalidPayloadSize;
  }
  const bool result_age_invalid =
      job.maximum_result_age.has_value() && job.maximum_result_age->ns() <= 0;
  if (job.safety_margin.ns() < 0 || result_age_invalid || job.maximum_result_ready_skew.ns() < 0 ||
      job.observation_time > job.release_time || job.release_time > now) {
    return ReasonCode::kInvalidTiming;
  }
  return std::nullopt;
}

std::optional<Duration> execution_duration(const Worker& worker, ModelClass model) {
  const auto profile = std::ranges::find(worker.profiles, model, &ExecutionProfile::model);
  if (profile == worker.profiles.end() || profile->duration.ns() <= 0) {
    return std::nullopt;
  }
  return profile->duration;
}

std::vector<EligibleExecution> eligible_executions(const ResourceSnapshot& snapshot,
                                                   const Job& job) {
  std::vector<EligibleExecution> executions;
  for (const WorkerId id : job.eligible_workers) {
    const auto worker = std::ranges::find(snapshot.workers, id, &Worker::id);
    if (worker == snapshot.workers.end()) {
      continue;
    }
    const std::optional<Duration> duration = execution_duration(*worker, job.model);
    if (duration) {
      executions.push_back(EligibleExecution{.worker = std::cref(*worker), .duration = *duration});
    }
  }
  return executions;
}

std::optional<Transfer> plan_on(const CapacityCalendar& link,
                                const std::optional<CapacityCalendar>& medium,
                                const std::optional<CapacityCalendar>& endpoint, TimePoint earliest,
                                Bytes demand) {
  std::vector<const CapacityCalendar*> limits;
  for (const std::optional<CapacityCalendar>* limit : {&medium, &endpoint}) {
    if (*limit) {
      limits.push_back(&**limit);
    }
  }
  return link.plan_transfer(earliest, demand, limits);
}

const EndpointBudget* endpoint_of(const ResourceSnapshot& snapshot, RobotId robot) {
  const auto endpoint = std::ranges::find(snapshot.endpoints, robot, &EndpointBudget::robot);
  return endpoint == snapshot.endpoints.end() ? nullptr : &*endpoint;
}

const std::optional<CapacityCalendar>& uncapped() {
  static const std::optional<CapacityCalendar> none;
  return none;
}

bool delivers_earlier(const Candidate& lhs, const Candidate& rhs) {
  return std::tie(lhs.delivery, lhs.worker) < std::tie(rhs.delivery, rhs.worker);
}

std::optional<Candidate> earliest_candidate(const ResourceSnapshot& snapshot, const Job& job,
                                            TimePoint compute_ready,
                                            const std::vector<EligibleExecution>& executions) {
  std::optional<Candidate> best;
  for (const EligibleExecution& execution : executions) {
    const Worker& worker = execution.worker.get();
    const std::optional<Interval> compute =
        worker.lane.earliest_fit(compute_ready, execution.duration);
    if (!compute) {
      continue;
    }
    const std::optional<TimePoint> download_ready =
        checked_add(compute->end, snapshot.dispatch_lead);
    if (!download_ready) {
      continue;
    }
    const EndpointBudget* endpoint = endpoint_of(snapshot, job.robot);
    std::optional<Transfer> download =
        plan_on(snapshot.downlink, snapshot.shared_medium,
                endpoint != nullptr ? endpoint->downlink : uncapped(), *download_ready,
                job.action_chunk_bytes);
    const std::optional<TimePoint> delivery =
        download ? checked_add(download->completion, snapshot.downlink_delay) : std::nullopt;
    if (!download || !delivery) {
      continue;
    }
    Candidate candidate{.worker = worker.id,
                        .compute = *compute,
                        .download = std::move(*download),
                        .delivery = *delivery};
    if (!best || delivers_earlier(candidate, *best)) {
      best = std::move(candidate);
    }
  }
  return best;
}

ReasonCode lateness_reason(const Job& job, TimePoint deadline) {
  const std::optional<TimePoint> freshness = freshness_deadline(job);
  return freshness && *freshness == deadline ? ReasonCode::kStaleResult : ReasonCode::kDeadlineMiss;
}

std::optional<TimePoint> compute_ready_after(const ResourceSnapshot& snapshot,
                                             TimePoint upload_completion) {
  const std::optional<TimePoint> received = checked_add(upload_completion, snapshot.uplink_delay);
  return received ? checked_add(*received, snapshot.dispatch_lead) : std::nullopt;
}

bool stages_in_order(const ResourceSnapshot& snapshot, const ChainReservation& chain) {
  if (chain.download.slices.empty()) {
    return false;
  }
  const std::optional<TimePoint> download_start =
      snapshot.downlink.slot_start(chain.download.slices.front().slot);
  const std::optional<TimePoint> compute_ready =
      compute_ready_after(snapshot, chain.upload.completion);
  const std::optional<TimePoint> download_ready =
      checked_add(chain.compute.end, snapshot.dispatch_lead);
  const std::optional<TimePoint> delivery =
      checked_add(chain.download.completion, snapshot.downlink_delay);
  return download_start && compute_ready && download_ready && delivery &&
         *compute_ready <= chain.compute.start && *download_ready <= *download_start &&
         *delivery == chain.delivery;
}

// Reserves the chain on its robot's capped directions; the other endpoints are unchanged.
std::optional<std::vector<EndpointBudget>> reserve_on_endpoint(const ResourceSnapshot& snapshot,
                                                               const ChainReservation& chain) {
  std::vector<EndpointBudget> endpoints = snapshot.endpoints;
  const auto endpoint = std::ranges::find(endpoints, chain.robot, &EndpointBudget::robot);
  if (endpoint == endpoints.end()) {
    return endpoints;
  }
  for (auto [calendar, transfer] : {std::pair{&endpoint->uplink, &chain.upload},
                                    std::pair{&endpoint->downlink, &chain.download}}) {
    if (!*calendar) {
      continue;
    }
    std::optional<CapacityCalendar> reserved = (*calendar)->reserve(*transfer);
    if (!reserved) {
      return std::nullopt;
    }
    *calendar = std::move(reserved);
  }
  return endpoints;
}

std::optional<CapacityCalendar> reserve_on_medium(const CapacityCalendar& medium,
                                                  const ChainReservation& chain) {
  const std::optional<CapacityCalendar> with_upload = medium.reserve(chain.upload);
  if (!with_upload) {
    return std::nullopt;
  }
  return with_upload->reserve(chain.download);
}

AdmissionResult admitted_result(const ResourceSnapshot& snapshot, ChainReservation chain) {
  std::optional<ResourceSnapshot> committed = commit(snapshot, chain);
  if (!committed) {
    throw std::logic_error{"a chain planned against a snapshot failed to commit to that snapshot"};
  }
  return AdmissionResult{
      .decision = {.status = DecisionStatus::kAdmitted, .reason = ReasonCode::kNone},
      .reservation = std::move(chain),
      .snapshot = std::move(*committed)};
}

}  // namespace

bool is_consistent(const ResourceSnapshot& snapshot) {
  const bool medium_matches =
      !snapshot.shared_medium || snapshot.shared_medium->same_geometry(snapshot.uplink);
  std::vector<RobotId> robots;
  for (const EndpointBudget& endpoint : snapshot.endpoints) {
    for (const std::optional<CapacityCalendar>* calendar : {&endpoint.uplink, &endpoint.downlink}) {
      if (*calendar && !(*calendar)->same_geometry(snapshot.uplink)) {
        return false;
      }
    }
    robots.push_back(endpoint.robot);
  }
  std::ranges::sort(robots);
  if (std::ranges::adjacent_find(robots) != robots.end()) {
    return false;
  }
  return snapshot.uplink.same_geometry(snapshot.downlink) && medium_matches &&
         snapshot.uplink_delay.ns() >= 0 && snapshot.downlink_delay.ns() >= 0 &&
         snapshot.dispatch_lead.ns() >= 0 &&
         checked_add(snapshot.uplink.horizon_end(), snapshot.uplink_delay).has_value() &&
         checked_add(snapshot.downlink.horizon_end(), snapshot.downlink_delay).has_value();
}

std::optional<ResourceSnapshot> commit(const ResourceSnapshot& snapshot,
                                       const ChainReservation& chain) {
  if (!is_consistent(snapshot) || !stages_in_order(snapshot, chain)) {
    return std::nullopt;
  }
  std::optional<CapacityCalendar> uplink = snapshot.uplink.reserve(chain.upload);
  std::optional<CapacityCalendar> downlink = snapshot.downlink.reserve(chain.download);
  std::optional<CapacityCalendar> medium =
      snapshot.shared_medium ? reserve_on_medium(*snapshot.shared_medium, chain) : std::nullopt;
  std::optional<std::vector<EndpointBudget>> endpoints = reserve_on_endpoint(snapshot, chain);
  if (!uplink || !downlink || (snapshot.shared_medium && !medium) || !endpoints) {
    return std::nullopt;
  }
  std::vector<Worker> workers = snapshot.workers;
  const auto worker = std::ranges::find(workers, chain.worker, &Worker::id);
  if (worker == workers.end()) {
    return std::nullopt;
  }
  std::optional<IntervalCalendar> lane = worker->lane.reserve(chain.compute);
  if (!lane) {
    return std::nullopt;
  }
  worker->lane = std::move(*lane);
  return ResourceSnapshot{.uplink = std::move(*uplink),
                          .downlink = std::move(*downlink),
                          .workers = std::move(workers),
                          .shared_medium = std::move(medium),
                          .uplink_delay = snapshot.uplink_delay,
                          .downlink_delay = snapshot.downlink_delay,
                          .dispatch_lead = snapshot.dispatch_lead,
                          .source = snapshot.source,
                          .endpoints = std::move(*endpoints)};
}

AdmissionResult admit(const ResourceSnapshot& snapshot, const Job& job, TimePoint now) {
  const auto not_admitted = [&snapshot](DecisionStatus status, ReasonCode reason) {
    return AdmissionResult{.decision = {.status = status, .reason = reason},
                           .reservation = std::nullopt,
                           .snapshot = snapshot};
  };

  if (!is_consistent(snapshot)) {
    return not_admitted(DecisionStatus::kRejected, ReasonCode::kInvalidSnapshot);
  }
  if (const std::optional<ReasonCode> invalid = invalid_input(job, now)) {
    return not_admitted(DecisionStatus::kRejected, *invalid);
  }
  const std::optional<TimePoint> deadline = effective_deadline(job);
  if (!deadline) {
    return not_admitted(DecisionStatus::kRejected, ReasonCode::kArithmeticOverflow);
  }
  const std::vector<EligibleExecution> executions = eligible_executions(snapshot, job);
  if (executions.empty()) {
    return not_admitted(DecisionStatus::kRejected, ReasonCode::kNoEligibleWorker);
  }
  const std::optional<TimePoint> upload_ready = checked_add(now, snapshot.dispatch_lead);
  if (!upload_ready) {
    return not_admitted(DecisionStatus::kRejected, ReasonCode::kArithmeticOverflow);
  }
  const EndpointBudget* endpoint = endpoint_of(snapshot, job.robot);
  const std::optional<Transfer> upload =
      plan_on(snapshot.uplink, snapshot.shared_medium,
              endpoint != nullptr ? endpoint->uplink : uncapped(), *upload_ready, job.upload_bytes);
  if (!upload) {
    return not_admitted(DecisionStatus::kDeferred, ReasonCode::kUplinkServiceUnavailable);
  }
  const std::optional<TimePoint> compute_ready = compute_ready_after(snapshot, upload->completion);
  if (!compute_ready) {
    return not_admitted(DecisionStatus::kRejected, ReasonCode::kArithmeticOverflow);
  }
  const std::optional<Candidate> best =
      earliest_candidate(snapshot, job, *compute_ready, executions);
  if (!best) {
    return not_admitted(DecisionStatus::kDeferred, ReasonCode::kNoFeasibleChain);
  }
  const std::optional<Duration> slack = elapsed(best->delivery, *deadline);
  if (!slack) {
    return not_admitted(DecisionStatus::kRejected, ReasonCode::kArithmeticOverflow);
  }
  if (slack->ns() < 0) {
    return not_admitted(DecisionStatus::kRejected, lateness_reason(job, *deadline));
  }
  return admitted_result(snapshot, ChainReservation{.job = job.id,
                                                    .worker = best->worker,
                                                    .upload = *upload,
                                                    .compute = best->compute,
                                                    .download = best->download,
                                                    .deadline = *deadline,
                                                    .slack = *slack,
                                                    .delivery = best->delivery,
                                                    .robot = job.robot});
}

}  // namespace rovi::core
