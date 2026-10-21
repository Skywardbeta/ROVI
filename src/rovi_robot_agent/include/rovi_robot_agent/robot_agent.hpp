// Robot-side action buffer, observation job generation, chunk validation, and local fallback state.
#ifndef ROVI_ROBOT_AGENT_ROBOT_AGENT_HPP_
#define ROVI_ROBOT_AGENT_ROBOT_AGENT_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string_view>
#include <vector>

#include "rovi_core/admission.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/receipt.hpp"
#include "rovi_core/units.hpp"

namespace rovi::robot_agent {

inline constexpr std::size_t kAbandonedMemory = 8;
inline constexpr std::size_t kMaximumReleaseJobs = 64;

struct AgentConfig {
  core::RobotId robot;
  core::Duration initial_buffer;
  core::Duration chunk_horizon;
  core::Duration maximum_buffer;
  core::Duration safety_margin;
  core::Duration maximum_result_age;
  core::Bytes upload_bytes;
  core::Bytes action_chunk_bytes;
  core::ModelClass model;
  std::vector<core::WorkerId> eligible_workers;
  bool grouped{false};
  core::Duration release_grace{};
};

enum class Mode : std::uint8_t { kExecuting, kFallback };

enum class ChunkVerdict : std::uint8_t { kAccepted, kLate, kStale, kUnexpected, kOverflow, kHeld };

enum class ReleaseVerdict : std::uint8_t { kIgnored, kPending, kAccepted, kMissed, kRejected };

struct Counters {
  std::uint64_t jobs_submitted{0};
  std::uint64_t jobs_rejected{0};
  std::uint64_t chunks_accepted{0};
  std::uint64_t chunks_late{0};
  std::uint64_t chunks_stale{0};
  std::uint64_t chunks_missed{0};
  std::uint64_t fallback_entries{0};

  bool operator==(const Counters&) const = default;
};

struct InFlight {
  core::Job job;
  core::TimePoint deadline;
  std::optional<core::WorkerId> worker{};
  bool delivered{false};
  std::optional<core::TimePoint> release_time{};
};

struct ReleaseNotice {
  std::vector<core::JobId> jobs;
  bool released{false};
  core::TimePoint release_time;
};

struct ChunkReceipt {
  core::JobId job;
  core::WorkerId worker;
  core::TimePoint observation_time;
};

[[nodiscard]] std::string_view verdict_name(ChunkVerdict verdict);

[[nodiscard]] core::JobId job_id_for(core::RobotId robot, std::uint64_t sequence);

class RobotAgent {
 public:
  [[nodiscard]] static std::optional<RobotAgent> create(AgentConfig config, core::TimePoint start);

  [[nodiscard]] std::optional<core::Job> observe(core::TimePoint now);
  [[nodiscard]] bool on_decision(core::JobId job, core::DecisionStatus status,
                                 core::WorkerId worker);
  [[nodiscard]] ChunkVerdict on_chunk(const ChunkReceipt& receipt, core::TimePoint now);
  [[nodiscard]] ReleaseVerdict on_release(const ReleaseNotice& notice, core::TimePoint now);
  void refresh(core::TimePoint now);

  [[nodiscard]] core::RobotId robot() const;
  [[nodiscard]] Mode mode() const;
  [[nodiscard]] core::TimePoint buffer_empty_time() const;
  // Where the newest accepted chunk starts playing; the buffer always ends one horizon later.
  [[nodiscard]] const std::optional<core::TimePoint>& latest_chunk_start() const;
  [[nodiscard]] const std::optional<InFlight>& in_flight() const;
  [[nodiscard]] const Counters& counters() const;
  [[nodiscard]] const core::ReceiptLedger& receipts() const;
  [[nodiscard]] const std::optional<core::Receipt>& latest_receipt() const;

 private:
  RobotAgent(AgentConfig config, core::TimePoint buffer_empty);

  [[nodiscard]] std::optional<core::TimePoint> buffer_bound(core::TimePoint now) const;
  [[nodiscard]] bool buffer_full(core::TimePoint now) const;
  [[nodiscard]] core::TimePoint abandon_time(const InFlight& job) const;
  void abandon();
  ChunkVerdict classify_late(const InFlight& job, core::TimePoint now);
  ChunkVerdict accept(core::TimePoint start);

  AgentConfig config_;
  core::TimePoint buffer_empty_;
  std::optional<core::TimePoint> latest_chunk_start_;
  Mode mode_{Mode::kExecuting};
  std::optional<InFlight> in_flight_;
  std::deque<InFlight> abandoned_;
  std::uint64_t next_sequence_{1};
  Counters counters_{};
  core::ReceiptLedger receipts_;
  std::optional<core::Receipt> latest_receipt_;
};

}  // namespace rovi::robot_agent

#endif  // ROVI_ROBOT_AGENT_ROBOT_AGENT_HPP_
