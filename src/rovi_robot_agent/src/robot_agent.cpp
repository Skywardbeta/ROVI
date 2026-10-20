#include "rovi_robot_agent/robot_agent.hpp"

#include <algorithm>
#include <cstdint>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

#include "rovi_core/admission.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/receipt.hpp"
#include "rovi_core/units.hpp"

namespace rovi::robot_agent {
namespace {

constexpr unsigned kSequenceBits = 32;
constexpr std::uint64_t kSequenceMask = 0xFFFF'FFFFULL;

}  // namespace

std::string_view verdict_name(ChunkVerdict verdict) {
  switch (verdict) {
    case ChunkVerdict::kAccepted:
      return "accepted";
    case ChunkVerdict::kLate:
      return "late";
    case ChunkVerdict::kStale:
      return "stale";
    case ChunkVerdict::kUnexpected:
      return "unexpected";
    case ChunkVerdict::kOverflow:
      return "overflow";
    case ChunkVerdict::kHeld:
      return "held";
  }
  return "unknown";
}

core::JobId job_id_for(core::RobotId robot, std::uint64_t sequence) {
  return core::JobId{(std::uint64_t{robot.value()} << kSequenceBits) | (sequence & kSequenceMask)};
}

RobotAgent::RobotAgent(AgentConfig config, core::TimePoint buffer_empty)
    : config_{std::move(config)}, buffer_empty_{buffer_empty} {}

std::optional<RobotAgent> RobotAgent::create(AgentConfig config, core::TimePoint start) {
  if (config.initial_buffer.ns() < 0 || config.chunk_horizon.ns() <= 0 ||
      config.maximum_buffer < config.chunk_horizon ||
      config.maximum_buffer < config.initial_buffer || config.safety_margin.ns() < 0 ||
      config.maximum_result_age.ns() <= 0 || config.upload_bytes.value() <= 0 ||
      config.action_chunk_bytes.value() <= 0 || config.eligible_workers.empty() ||
      config.release_grace < core::Duration{} ||
      (config.grouped &&
       config.maximum_buffer.ns() - config.chunk_horizon.ns() > config.chunk_horizon.ns())) {
    return std::nullopt;
  }
  const std::optional<core::TimePoint> buffer_empty =
      core::checked_add(start, config.initial_buffer);
  if (!buffer_empty) {
    return std::nullopt;
  }
  return RobotAgent{std::move(config), *buffer_empty};
}

std::optional<core::Job> RobotAgent::observe(core::TimePoint now) {
  refresh(now);
  if (in_flight_ && now > abandon_time(*in_flight_)) {
    abandon();
  }
  if (in_flight_ || buffer_full(now)) {
    return std::nullopt;
  }
  const std::optional<core::TimePoint> bound = buffer_bound(now);
  if (!bound) {
    return std::nullopt;
  }
  core::Job job{.id = job_id_for(config_.robot, next_sequence_),
                .release_time = now,
                .observation_time = now,
                .buffer_empty_time = *bound,
                .safety_margin = config_.safety_margin,
                .task_deadline = std::nullopt,
                .maximum_result_age = config_.maximum_result_age,
                .upload_bytes = config_.upload_bytes,
                .action_chunk_bytes = config_.action_chunk_bytes,
                .model = config_.model,
                .eligible_workers = config_.eligible_workers,
                .robot = config_.robot,
                .sequence = next_sequence_};
  const std::optional<core::TimePoint> deadline = core::effective_deadline(job);
  if (!deadline) {
    return std::nullopt;
  }
  ++next_sequence_;
  ++counters_.jobs_submitted;
  receipts_.offer(job.id, job.observation_time, *deadline);
  in_flight_ = InFlight{.job = job, .deadline = *deadline};
  return job;
}

bool RobotAgent::on_decision(core::JobId job, core::DecisionStatus status, core::WorkerId worker) {
  if (!in_flight_ || in_flight_->job.id != job || in_flight_->worker) {
    return false;
  }
  switch (status) {
    case core::DecisionStatus::kAdmitted:
      in_flight_->worker = worker;
      receipts_.assign(job, worker);
      return true;
    case core::DecisionStatus::kRejected:
      ++counters_.jobs_rejected;
      in_flight_.reset();
      return false;
    case core::DecisionStatus::kDeferred:
      return false;
  }
  return false;
}

ChunkVerdict RobotAgent::on_chunk(const ChunkReceipt& receipt, core::TimePoint now) {
  refresh(now);
  latest_receipt_ = receipts_.receive(
      {.job = receipt.job, .worker = receipt.worker, .observation = receipt.observation_time}, now);
  const auto matches = [&receipt](const InFlight& candidate) {
    return candidate.job.id == receipt.job && candidate.worker == receipt.worker &&
           candidate.job.observation_time == receipt.observation_time;
  };
  if (!in_flight_ || !matches(*in_flight_)) {
    const auto abandoned = std::ranges::find_if(abandoned_, matches);
    if (abandoned == abandoned_.end()) {
      return ChunkVerdict::kUnexpected;
    }
    const ChunkVerdict verdict = classify_late(*abandoned, now);
    abandoned_.erase(abandoned);
    return verdict;
  }
  if (!config_.grouped && now > in_flight_->deadline) {
    const InFlight current = *in_flight_;
    in_flight_.reset();
    ++counters_.chunks_missed;
    return classify_late(current, now);
  }
  if (config_.grouped && !in_flight_->release_time) {
    in_flight_->delivered = true;
    return ChunkVerdict::kHeld;
  }
  const InFlight current = *in_flight_;
  in_flight_.reset();
  if (current.release_time && *current.release_time > current.deadline) {
    ++counters_.chunks_missed;
    return classify_late(current, now);
  }
  return accept(current.release_time ? *current.release_time : std::max(now, buffer_empty_));
}

ReleaseVerdict RobotAgent::on_release(const ReleaseNotice& notice, core::TimePoint now) {
  const std::vector<core::JobId>& jobs = notice.jobs;
  const bool released = notice.released;
  refresh(now);
  if (!config_.grouped || !in_flight_ || jobs.size() > kMaximumReleaseJobs ||
      std::ranges::find(jobs, in_flight_->job.id) == jobs.end() || notice.release_time > now ||
      notice.release_time < in_flight_->job.observation_time) {
    return ReleaseVerdict::kIgnored;
  }
  if (!in_flight_->worker || !in_flight_->delivered) {
    if (released) {
      in_flight_->release_time = notice.release_time;
      return ReleaseVerdict::kPending;
    }
    if (!in_flight_->worker) {
      ++counters_.jobs_rejected;
      in_flight_.reset();
      return ReleaseVerdict::kRejected;
    }
    abandon();
    return ReleaseVerdict::kMissed;
  }
  const InFlight current = *in_flight_;
  in_flight_.reset();
  if (!released || notice.release_time > current.deadline) {
    ++counters_.chunks_missed;
    return ReleaseVerdict::kMissed;
  }
  return accept(notice.release_time) == ChunkVerdict::kAccepted ? ReleaseVerdict::kAccepted
                                                                : ReleaseVerdict::kMissed;
}

ChunkVerdict RobotAgent::accept(core::TimePoint start) {
  const std::optional<core::TimePoint> extended = core::checked_add(start, config_.chunk_horizon);
  if (!extended) {
    ++counters_.chunks_missed;
    return ChunkVerdict::kOverflow;
  }
  buffer_empty_ = std::max(buffer_empty_, *extended);
  mode_ = Mode::kExecuting;
  ++counters_.chunks_accepted;
  return ChunkVerdict::kAccepted;
}

void RobotAgent::refresh(core::TimePoint now) {
  if (mode_ == Mode::kExecuting && now >= buffer_empty_) {
    mode_ = Mode::kFallback;
    ++counters_.fallback_entries;
  }
}

void RobotAgent::abandon() {
  if (!in_flight_) {
    return;
  }
  ++counters_.chunks_missed;
  if (in_flight_->worker) {
    abandoned_.push_back(*in_flight_);
    if (abandoned_.size() > kAbandonedMemory) {
      abandoned_.pop_front();
    }
  }
  in_flight_.reset();
}

ChunkVerdict RobotAgent::classify_late(const InFlight& job, core::TimePoint now) {
  const std::optional<core::TimePoint> fresh_until = core::freshness_deadline(job.job);
  if (fresh_until && now > *fresh_until) {
    ++counters_.chunks_stale;
    return ChunkVerdict::kStale;
  }
  ++counters_.chunks_late;
  return ChunkVerdict::kLate;
}

core::TimePoint RobotAgent::abandon_time(const InFlight& job) const {
  if (!config_.grouped) {
    return job.deadline;
  }
  return core::checked_add(job.deadline, config_.release_grace).value_or(job.deadline);
}

core::RobotId RobotAgent::robot() const { return config_.robot; }

Mode RobotAgent::mode() const { return mode_; }

core::TimePoint RobotAgent::buffer_empty_time() const { return buffer_empty_; }

const std::optional<InFlight>& RobotAgent::in_flight() const { return in_flight_; }

const Counters& RobotAgent::counters() const { return counters_; }

const core::ReceiptLedger& RobotAgent::receipts() const { return receipts_; }

const std::optional<core::Receipt>& RobotAgent::latest_receipt() const { return latest_receipt_; }

bool RobotAgent::buffer_full(core::TimePoint now) const {
  if (mode_ != Mode::kExecuting) {
    return false;
  }
  const core::Duration headroom =
      core::Duration::from_ns(config_.maximum_buffer.ns() - config_.chunk_horizon.ns());
  const std::optional<core::TimePoint> refill_point = core::checked_add(now, headroom);
  return refill_point && buffer_empty_ > *refill_point;
}

std::optional<core::TimePoint> RobotAgent::buffer_bound(core::TimePoint now) const {
  if (mode_ == Mode::kExecuting) {
    return buffer_empty_;
  }
  const std::optional<core::TimePoint> fresh_until =
      core::checked_add(now, config_.maximum_result_age);
  if (!fresh_until) {
    return std::nullopt;
  }
  return core::checked_add(*fresh_until, config_.safety_margin);
}

}  // namespace rovi::robot_agent
