#include "rovi_robot_agent/robot_agent.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "rovi_core/admission.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/receipt.hpp"
#include "rovi_core/units.hpp"

namespace rovi::robot_agent {
namespace {

constexpr std::int64_t kNsPerMs = 1'000'000;
constexpr std::int64_t kMaxNs = std::numeric_limits<std::int64_t>::max();
constexpr core::WorkerId kWorker{1};

core::TimePoint at_ms(std::int64_t count) { return core::TimePoint::from_ns(count * kNsPerMs); }

core::Duration ms(std::int64_t count) { return core::Duration::from_ns(count * kNsPerMs); }

AgentConfig reference_config() {
  return AgentConfig{.robot = core::RobotId{3},
                     .initial_buffer = ms(250),
                     .chunk_horizon = ms(200),
                     .maximum_buffer = ms(1'000),
                     .safety_margin = ms(50),
                     .maximum_result_age = ms(400),
                     .upload_bytes = core::Bytes{200'000},
                     .action_chunk_bytes = core::Bytes{20'000},
                     .model = core::ModelClass{1},
                     .eligible_workers = {kWorker}};
}

RobotAgent agent_at(std::int64_t start_ms, const AgentConfig& config = reference_config()) {
  return RobotAgent::create(config, at_ms(start_ms)).value();
}

core::Job admitted_job(RobotAgent& agent, core::TimePoint now) {
  const core::Job job = agent.observe(now).value();
  EXPECT_TRUE(agent.on_decision(job.id, core::DecisionStatus::kAdmitted, kWorker));
  return job;
}

ChunkReceipt chunk_of(const core::Job& job) {
  return ChunkReceipt{.job = job.id, .worker = kWorker, .observation_time = job.observation_time};
}

ReleaseNotice release_of(std::vector<core::JobId> jobs, bool released, core::TimePoint at) {
  return ReleaseNotice{.jobs = std::move(jobs), .released = released, .release_time = at};
}

AgentConfig grouped_config() {
  AgentConfig config = reference_config();
  config.grouped = true;
  config.chunk_horizon = ms(300);
  config.maximum_buffer = ms(600);
  return config;
}

TEST(RobotAgent, CreatesAJobBoundedByTheActionBufferDeadline) {
  RobotAgent agent = agent_at(1'000);

  const std::optional<core::Job> job = agent.observe(at_ms(1'000));

  const core::Job expected{.id = job_id_for(core::RobotId{3}, 1),
                           .release_time = at_ms(1'000),
                           .observation_time = at_ms(1'000),
                           .buffer_empty_time = at_ms(1'250),
                           .safety_margin = ms(50),
                           .task_deadline = std::nullopt,
                           .maximum_result_age = ms(400),
                           .upload_bytes = core::Bytes{200'000},
                           .action_chunk_bytes = core::Bytes{20'000},
                           .model = core::ModelClass{1},
                           .eligible_workers = {kWorker},
                           .robot = core::RobotId{3},
                           .sequence = 1};
  EXPECT_EQ(job, expected);
  ASSERT_TRUE(agent.in_flight().has_value());
  EXPECT_EQ(agent.in_flight()->deadline, at_ms(1'200));
  EXPECT_FALSE(agent.in_flight()->worker.has_value());
  EXPECT_EQ(agent.counters().jobs_submitted, 1U);
  EXPECT_EQ(agent.robot(), core::RobotId{3});
}

TEST(RobotAgent, KeepsAtMostOneJobInFlight) {
  RobotAgent agent = agent_at(1'000);
  ASSERT_TRUE(agent.observe(at_ms(1'000)).has_value());

  EXPECT_EQ(agent.observe(at_ms(1'100)), std::nullopt);
  EXPECT_EQ(agent.counters().jobs_submitted, 1U);
}

TEST(RobotAgent, SkipsObservationWhileTheActionBufferIsFull) {
  AgentConfig config = reference_config();
  config.maximum_buffer = ms(400);
  RobotAgent agent = agent_at(1'000, config);

  EXPECT_EQ(agent.observe(at_ms(1'000)), std::nullopt);
  EXPECT_EQ(agent.observe(at_ms(1'049)), std::nullopt);
  EXPECT_TRUE(agent.observe(at_ms(1'050)).has_value());
}

TEST(RobotAgent, UploadsOnceOnAdmissionAndRecordsTheReservedWorker) {
  RobotAgent agent = agent_at(1'000);
  const core::JobId job = agent.observe(at_ms(1'000)).value().id;

  EXPECT_FALSE(agent.on_decision(job, core::DecisionStatus::kDeferred, kWorker));
  EXPECT_FALSE(agent.on_decision(core::JobId{99}, core::DecisionStatus::kAdmitted, kWorker));
  EXPECT_TRUE(agent.on_decision(job, core::DecisionStatus::kAdmitted, core::WorkerId{4}));
  EXPECT_FALSE(agent.on_decision(job, core::DecisionStatus::kAdmitted, kWorker));
  EXPECT_EQ(agent.in_flight()->worker, core::WorkerId{4});
}

TEST(RobotAgent, IgnoresEveryDecisionThatArrivesAfterAdmission) {
  RobotAgent agent = agent_at(1'000);
  const core::Job job = admitted_job(agent, at_ms(1'000));

  EXPECT_FALSE(agent.on_decision(job.id, core::DecisionStatus::kRejected, kWorker));

  EXPECT_TRUE(agent.in_flight().has_value());
  EXPECT_EQ(agent.counters().jobs_rejected, 0U);
}

TEST(RobotAgent, ForgetsARejectedJobAndObservesAgain) {
  RobotAgent agent = agent_at(1'000);
  const core::JobId job = agent.observe(at_ms(1'000)).value().id;

  EXPECT_FALSE(agent.on_decision(job, core::DecisionStatus::kRejected, kWorker));

  EXPECT_FALSE(agent.in_flight().has_value());
  EXPECT_EQ(agent.counters().jobs_rejected, 1U);
  EXPECT_EQ(agent.observe(at_ms(1'010)).value().sequence, 2U);
}

TEST(RobotAgent, AcceptsAnOnTimeChunkAndExtendsTheActionBuffer) {
  RobotAgent agent = agent_at(1'000);
  const core::Job job = admitted_job(agent, at_ms(1'000));

  EXPECT_EQ(agent.on_chunk(chunk_of(job), at_ms(1'150)), ChunkVerdict::kAccepted);

  EXPECT_EQ(agent.buffer_empty_time(), at_ms(1'450));
  EXPECT_EQ(agent.mode(), Mode::kExecuting);
  EXPECT_FALSE(agent.in_flight().has_value());
  EXPECT_EQ(agent.counters().chunks_accepted, 1U);
}

TEST(RobotAgent, CountsALateChunkAsLateAndMissedWithoutExtendingTheBuffer) {
  RobotAgent agent = agent_at(1'000);
  const core::Job job = admitted_job(agent, at_ms(1'000));

  EXPECT_EQ(agent.on_chunk(chunk_of(job), at_ms(1'201)), ChunkVerdict::kLate);

  EXPECT_EQ(agent.buffer_empty_time(), at_ms(1'250));
  EXPECT_FALSE(agent.in_flight().has_value());
  EXPECT_EQ(agent.counters().chunks_late, 1U);
  EXPECT_EQ(agent.counters().chunks_missed, 1U);
  EXPECT_EQ(agent.counters().chunks_accepted, 0U);
}

TEST(RobotAgent, ClassifiesAChunkBeyondTheFreshnessBoundAsStaleAndMissed) {
  AgentConfig config = reference_config();
  config.maximum_result_age = ms(100);
  RobotAgent agent = agent_at(1'000, config);
  const core::Job job = admitted_job(agent, at_ms(1'000));

  EXPECT_EQ(agent.on_chunk(chunk_of(job), at_ms(1'150)), ChunkVerdict::kStale);

  EXPECT_EQ(agent.counters().chunks_stale, 1U);
  EXPECT_EQ(agent.counters().chunks_late, 0U);
  EXPECT_EQ(agent.counters().chunks_missed, 1U);
}

TEST(RobotAgent, IgnoresChunksForUnknownOrUnadmittedJobs) {
  RobotAgent agent = agent_at(1'000);
  const core::Job job = agent.observe(at_ms(1'000)).value();

  EXPECT_EQ(agent.on_chunk(chunk_of(job), at_ms(1'100)), ChunkVerdict::kUnexpected);
  core::Job unknown = job;
  unknown.id = core::JobId{99};
  EXPECT_EQ(agent.on_chunk(chunk_of(unknown), at_ms(1'100)), ChunkVerdict::kUnexpected);

  EXPECT_TRUE(agent.in_flight().has_value());
  EXPECT_EQ(agent.counters(), (Counters{.jobs_submitted = 1}));
}

TEST(RobotAgent, IgnoresAChunkFromAnotherWorkerOrForAnotherObservation) {
  RobotAgent agent = agent_at(1'000);
  const core::Job job = admitted_job(agent, at_ms(1'000));
  ChunkReceipt other_worker = chunk_of(job);
  other_worker.worker = core::WorkerId{2};
  ChunkReceipt other_observation = chunk_of(job);
  other_observation.observation_time = at_ms(999);

  EXPECT_EQ(agent.on_chunk(other_worker, at_ms(1'100)), ChunkVerdict::kUnexpected);
  EXPECT_EQ(agent.on_chunk(other_observation, at_ms(1'100)), ChunkVerdict::kUnexpected);
  EXPECT_EQ(agent.on_chunk(chunk_of(job), at_ms(1'100)), ChunkVerdict::kAccepted);
}

TEST(RobotAgent, EntersFallbackOnceWhenTheActionBufferEmpties) {
  RobotAgent agent = agent_at(1'000);

  agent.refresh(at_ms(1'249));
  EXPECT_EQ(agent.mode(), Mode::kExecuting);
  agent.refresh(at_ms(1'250));
  agent.refresh(at_ms(1'300));

  EXPECT_EQ(agent.mode(), Mode::kFallback);
  EXPECT_EQ(agent.counters().fallback_entries, 1U);
}

TEST(RobotAgent, AbandonsAnInFlightJobOnceItsDeadlinePassesAndCountsItMissed) {
  RobotAgent agent = agent_at(1'000);
  static_cast<void>(admitted_job(agent, at_ms(1'000)));

  EXPECT_EQ(agent.observe(at_ms(1'200)), std::nullopt);
  const std::optional<core::Job> next = agent.observe(at_ms(1'201));

  ASSERT_TRUE(next.has_value());
  EXPECT_EQ(next->sequence, 2U);
  EXPECT_EQ(agent.counters().chunks_missed, 1U);
}

TEST(RobotAgent, ClassifiesALateChunkForAnAbandonedJobLikeAnInFlightOne) {
  RobotAgent agent = agent_at(1'000);
  const core::Job job = admitted_job(agent, at_ms(1'000));
  static_cast<void>(agent.observe(at_ms(1'201)));

  EXPECT_EQ(agent.on_chunk(chunk_of(job), at_ms(1'210)), ChunkVerdict::kLate);
  EXPECT_EQ(agent.on_chunk(chunk_of(job), at_ms(1'220)), ChunkVerdict::kUnexpected);

  EXPECT_EQ(agent.counters().chunks_late, 1U);
  EXPECT_EQ(agent.counters().chunks_missed, 1U);
  EXPECT_TRUE(agent.in_flight().has_value());
}

TEST(RobotAgent, RemembersOnlyTheMostRecentAbandonedJobs) {
  RobotAgent agent = agent_at(1'000);
  core::TimePoint now = at_ms(1'000);
  std::vector<core::Job> abandoned;
  for (std::size_t index = 0; index <= kAbandonedMemory; ++index) {
    abandoned.push_back(admitted_job(agent, now));
    now = core::TimePoint::from_ns(agent.in_flight()->deadline.ns() + kNsPerMs);
  }
  static_cast<void>(agent.observe(now));

  EXPECT_EQ(agent.on_chunk(chunk_of(abandoned.front()), now), ChunkVerdict::kUnexpected);
  EXPECT_EQ(agent.on_chunk(chunk_of(abandoned.back()), now), ChunkVerdict::kLate);
  EXPECT_EQ(agent.counters().chunks_missed, kAbandonedMemory + 1);
}

TEST(RobotAgent, RecoversFromFallbackWithAFreshnessBoundJob) {
  RobotAgent agent = agent_at(1'000);
  agent.refresh(at_ms(1'300));

  const std::optional<core::Job> job = agent.observe(at_ms(1'300));

  ASSERT_TRUE(job.has_value());
  EXPECT_EQ(job->buffer_empty_time, at_ms(1'750));
  EXPECT_EQ(agent.in_flight()->deadline, at_ms(1'700));
  ASSERT_TRUE(agent.on_decision(job->id, core::DecisionStatus::kAdmitted, kWorker));
  EXPECT_EQ(agent.on_chunk(chunk_of(*job), at_ms(1'400)), ChunkVerdict::kAccepted);
  EXPECT_EQ(agent.buffer_empty_time(), at_ms(1'600));
  EXPECT_EQ(agent.mode(), Mode::kExecuting);
  EXPECT_EQ(agent.counters().fallback_entries, 1U);
}

TEST(RobotAgent, CountsAFallbackEntryObservedOnlyWhenALateChunkArrives) {
  RobotAgent agent = agent_at(1'000);
  const core::Job job = admitted_job(agent, at_ms(1'000));

  EXPECT_EQ(agent.on_chunk(chunk_of(job), at_ms(1'260)), ChunkVerdict::kLate);

  EXPECT_EQ(agent.mode(), Mode::kFallback);
  EXPECT_EQ(agent.counters().fallback_entries, 1U);
}

TEST(RobotAgent, CreateRejectsAnInvalidConfigurationOrStart) {
  const auto invalid = [](auto mutate) {
    AgentConfig config = reference_config();
    mutate(config);
    return !RobotAgent::create(config, at_ms(0)).has_value();
  };

  EXPECT_TRUE(invalid([](AgentConfig& config) { config.initial_buffer = ms(-1); }));
  EXPECT_TRUE(invalid([](AgentConfig& config) { config.chunk_horizon = ms(0); }));
  EXPECT_TRUE(invalid([](AgentConfig& config) { config.maximum_buffer = ms(199); }));
  EXPECT_TRUE(invalid([](AgentConfig& config) {
    config.initial_buffer = ms(300);
    config.maximum_buffer = ms(250);
  }));
  EXPECT_TRUE(invalid([](AgentConfig& config) { config.safety_margin = ms(-1); }));
  EXPECT_TRUE(invalid([](AgentConfig& config) { config.maximum_result_age = ms(0); }));
  EXPECT_TRUE(invalid([](AgentConfig& config) { config.upload_bytes = core::Bytes{0}; }));
  EXPECT_TRUE(invalid([](AgentConfig& config) { config.action_chunk_bytes = core::Bytes{0}; }));
  EXPECT_TRUE(invalid([](AgentConfig& config) { config.eligible_workers.clear(); }));
  EXPECT_FALSE(
      RobotAgent::create(reference_config(), core::TimePoint::from_ns(kMaxNs)).has_value());
}

TEST(RobotAgent, CreatesNoJobWhoseDeadlineCannotBeRepresented) {
  RobotAgent agent = agent_at(0);
  agent.refresh(at_ms(300));
  const core::TimePoint late = core::TimePoint::from_ns(kMaxNs - kNsPerMs);

  EXPECT_EQ(agent.observe(late), std::nullopt);
  EXPECT_EQ(agent.counters().jobs_submitted, 0U);
}

TEST(RobotAgent, ReportsAnActionBufferExtensionThatCannotBeRepresented) {
  AgentConfig config = reference_config();
  config.initial_buffer = core::Duration::from_ns(50);
  config.safety_margin = core::Duration::from_ns(0);
  config.maximum_result_age = core::Duration::from_ns(10);
  const core::TimePoint start = core::TimePoint::from_ns(kMaxNs - 100);
  RobotAgent agent = RobotAgent::create(config, start).value();
  const core::Job job = admitted_job(agent, start);

  EXPECT_EQ(agent.on_chunk(chunk_of(job), core::TimePoint::from_ns(kMaxNs - 95)),
            ChunkVerdict::kOverflow);
  EXPECT_FALSE(agent.in_flight().has_value());
  EXPECT_EQ(agent.counters().chunks_missed, 1U);
}

TEST(RobotAgent, AccountsForEverySubmittedJobExactlyOnce) {
  RobotAgent agent = agent_at(1'000);
  const auto balanced = [&agent] {
    const Counters& counters = agent.counters();
    const std::uint64_t open = agent.in_flight() ? 1U : 0U;
    return counters.jobs_submitted ==
           counters.chunks_accepted + counters.jobs_rejected + counters.chunks_missed + open;
  };
  const core::Job accepted = admitted_job(agent, at_ms(1'000));
  EXPECT_TRUE(balanced());
  ASSERT_EQ(agent.on_chunk(chunk_of(accepted), at_ms(1'100)), ChunkVerdict::kAccepted);
  const core::JobId rejected = agent.observe(at_ms(1'100)).value().id;
  ASSERT_FALSE(agent.on_decision(rejected, core::DecisionStatus::kRejected, kWorker));
  EXPECT_TRUE(balanced());
  const core::Job late = admitted_job(agent, at_ms(1'110));
  ASSERT_EQ(agent.on_chunk(chunk_of(late), at_ms(1'500)), ChunkVerdict::kLate);
  EXPECT_TRUE(balanced());
  static_cast<void>(agent.observe(at_ms(1'500)));
  static_cast<void>(agent.observe(at_ms(2'000)));
  EXPECT_TRUE(balanced());
  EXPECT_EQ(agent.counters().jobs_submitted, 5U);
}

TEST(RobotAgent, NamesEveryChunkVerdict) {
  EXPECT_EQ(verdict_name(ChunkVerdict::kAccepted), "accepted");
  EXPECT_EQ(verdict_name(ChunkVerdict::kLate), "late");
  EXPECT_EQ(verdict_name(ChunkVerdict::kStale), "stale");
  EXPECT_EQ(verdict_name(ChunkVerdict::kUnexpected), "unexpected");
  EXPECT_EQ(verdict_name(ChunkVerdict::kOverflow), "overflow");
}

TEST(RobotAgent, JobIdsCombineTheRobotAndTheLow32BitsOfTheSequence) {
  EXPECT_EQ(job_id_for(core::RobotId{3}, 1), core::JobId{(std::uint64_t{3} << 32U) | 1U});
  EXPECT_EQ(job_id_for(core::RobotId{3}, (std::uint64_t{1} << 32U) + 5),
            core::JobId{(std::uint64_t{3} << 32U) | 5U});
}

TEST(RobotAgent, HoldsAGroupedChunkUntilItsGroupIsReleased) {
  RobotAgent agent = agent_at(1'000, grouped_config());
  const core::Job job = admitted_job(agent, at_ms(1'000));

  EXPECT_EQ(agent.on_chunk(chunk_of(job), at_ms(1'100)), ChunkVerdict::kHeld);
  EXPECT_EQ(agent.buffer_empty_time(), at_ms(1'250));
  EXPECT_EQ(agent.counters().chunks_accepted, 0U);
  EXPECT_TRUE(agent.in_flight().has_value());
  EXPECT_EQ(
      agent.on_release(release_of({core::JobId{7}, job.id}, true, at_ms(1'120)), at_ms(1'120)),
      ReleaseVerdict::kAccepted);

  EXPECT_EQ(agent.buffer_empty_time(), at_ms(1'420));
  EXPECT_EQ(agent.counters().chunks_accepted, 1U);
  EXPECT_FALSE(agent.in_flight().has_value());
}

TEST(RobotAgent, CountsAHeldChunkAsMissedWhenItsGroupIsRefusedOrReleasedTooLate) {
  RobotAgent refused = agent_at(1'000, grouped_config());
  const core::Job first = admitted_job(refused, at_ms(1'000));
  ASSERT_EQ(refused.on_chunk(chunk_of(first), at_ms(1'100)), ChunkVerdict::kHeld);
  RobotAgent late = agent_at(1'000, grouped_config());
  const core::Job second = admitted_job(late, at_ms(1'000));
  ASSERT_EQ(late.on_chunk(chunk_of(second), at_ms(1'100)), ChunkVerdict::kHeld);

  EXPECT_EQ(refused.on_release(release_of({first.id}, false, at_ms(1'120)), at_ms(1'120)),
            ReleaseVerdict::kMissed);
  EXPECT_EQ(late.on_release(release_of({second.id}, true, at_ms(1'201)), at_ms(1'201)),
            ReleaseVerdict::kMissed);

  for (const RobotAgent* agent : {&refused, &late}) {
    EXPECT_EQ(agent->counters().chunks_missed, 1U);
    EXPECT_EQ(agent->counters().chunks_accepted, 0U);
    EXPECT_EQ(agent->buffer_empty_time(), at_ms(1'250));
    EXPECT_FALSE(agent->in_flight().has_value());
  }
}

TEST(RobotAgent, KeepsAnOnTimeReceiptWhenItsGroupIsRefused) {
  RobotAgent agent = agent_at(1'000, grouped_config());
  const core::Job job = admitted_job(agent, at_ms(1'000));
  ASSERT_EQ(agent.on_chunk(chunk_of(job), at_ms(1'100)), ChunkVerdict::kHeld);

  EXPECT_EQ(agent.on_release(release_of({job.id}, false, at_ms(1'120)), at_ms(1'120)),
            ReleaseVerdict::kMissed);

  EXPECT_EQ(agent.counters().chunks_missed, 1U);
  EXPECT_EQ(agent.receipts().summary(at_ms(1'300)),
            (core::ReceiptSummary{.offered = 1, .on_time = 1}));
}

TEST(RobotAgent, OffersARejectedJobAgainstItsOwnDeadline) {
  RobotAgent agent = agent_at(1'000);
  const core::Job job = agent.observe(at_ms(1'000)).value();
  EXPECT_FALSE(agent.on_decision(job.id, core::DecisionStatus::kRejected, kWorker));

  EXPECT_EQ(agent.receipts().deadline_of(job.id), at_ms(1'200));
  EXPECT_EQ(agent.receipts().summary(at_ms(1'200)),
            (core::ReceiptSummary{.offered = 1, .pending = 1}));
  EXPECT_EQ(agent.receipts().summary(at_ms(1'201)),
            (core::ReceiptSummary{.offered = 1, .missing = 1}));
}

TEST(RobotAgent, RecordsALateReceiptOfAnAbandonedJobOnceAndIgnoresUnmatchedChunks) {
  RobotAgent agent = agent_at(1'000);
  const core::Job job = admitted_job(agent, at_ms(1'000));
  static_cast<void>(agent.observe(at_ms(1'201)));
  ChunkReceipt other_worker = chunk_of(job);
  other_worker.worker = core::WorkerId{9};

  EXPECT_EQ(agent.on_chunk(other_worker, at_ms(1'205)), ChunkVerdict::kUnexpected);
  EXPECT_EQ(agent.latest_receipt(), std::nullopt);
  EXPECT_EQ(agent.on_chunk(chunk_of(job), at_ms(1'210)), ChunkVerdict::kLate);
  EXPECT_EQ(agent.latest_receipt(), (core::Receipt{.job = job.id,
                                                   .deadline = at_ms(1'200),
                                                   .at = at_ms(1'210),
                                                   .outcome = core::ReceiptOutcome::kLate}));
  EXPECT_EQ(agent.on_chunk(chunk_of(job), at_ms(1'220)), ChunkVerdict::kUnexpected);
  EXPECT_EQ(agent.latest_receipt(), std::nullopt);

  EXPECT_EQ(agent.receipts().summary(at_ms(1'220)),
            (core::ReceiptSummary{.offered = 2, .late = 1, .missing = 1}));
}

TEST(RobotAgent, RecordsALateReceiptEvenAfterForgettingTheAbandonedJob) {
  RobotAgent agent = agent_at(1'000);
  const core::Job first = admitted_job(agent, at_ms(1'000));
  core::TimePoint now = at_ms(1'000);
  for (std::size_t round = 0; round <= kAbandonedMemory; ++round) {
    now = core::checked_add(agent.in_flight().value().deadline, ms(1)).value();
    static_cast<void>(admitted_job(agent, now));
  }

  EXPECT_EQ(agent.on_chunk(chunk_of(first), now), ChunkVerdict::kUnexpected);

  ASSERT_TRUE(agent.latest_receipt().has_value());
  EXPECT_EQ(agent.latest_receipt()->job, first.id);
  EXPECT_EQ(agent.latest_receipt()->outcome, core::ReceiptOutcome::kLate);
}

TEST(RobotAgent, TreatsARefusedRoundBeforeAdmissionAsARejection) {
  RobotAgent agent = agent_at(1'000, grouped_config());
  const core::Job job = agent.observe(at_ms(1'000)).value();

  EXPECT_EQ(agent.on_release(release_of({job.id}, false, at_ms(1'050)), at_ms(1'050)),
            ReleaseVerdict::kRejected);

  EXPECT_EQ(agent.counters().jobs_rejected, 1U);
  EXPECT_FALSE(agent.in_flight().has_value());
  EXPECT_TRUE(agent.observe(at_ms(1'060)).has_value());
}

TEST(RobotAgent, AbandonsAnAdmittedJobWhoseGroupIsRefusedBeforeItsChunkArrives) {
  RobotAgent agent = agent_at(1'000, grouped_config());
  const core::Job job = admitted_job(agent, at_ms(1'000));

  EXPECT_EQ(agent.on_release(release_of({job.id}, false, at_ms(1'050)), at_ms(1'050)),
            ReleaseVerdict::kMissed);
  EXPECT_FALSE(agent.in_flight().has_value());
  EXPECT_EQ(agent.on_chunk(chunk_of(job), at_ms(1'100)), ChunkVerdict::kLate);

  EXPECT_EQ(agent.counters().chunks_missed, 1U);
  EXPECT_EQ(agent.counters().chunks_late, 1U);
}

TEST(RobotAgent, RecordsAnEarlyReleaseAndIgnoresReleasesForOtherJobsOrUngroupedAgents) {
  RobotAgent grouped = agent_at(1'000, grouped_config());
  const core::Job job = admitted_job(grouped, at_ms(1'000));
  RobotAgent ungrouped = agent_at(1'000);
  const core::Job plain = admitted_job(ungrouped, at_ms(1'000));

  EXPECT_EQ(grouped.on_release(release_of({core::JobId{7}}, false, at_ms(1'050)), at_ms(1'050)),
            ReleaseVerdict::kIgnored);
  EXPECT_EQ(grouped.on_release(release_of({job.id}, true, at_ms(1'050)), at_ms(1'050)),
            ReleaseVerdict::kPending);
  EXPECT_EQ(ungrouped.on_release(release_of({plain.id}, true, at_ms(1'050)), at_ms(1'050)),
            ReleaseVerdict::kIgnored);
  EXPECT_TRUE(grouped.in_flight().has_value());
  EXPECT_EQ(ungrouped.on_chunk(chunk_of(plain), at_ms(1'100)), ChunkVerdict::kAccepted);
}

TEST(RobotAgent, AbandonsAHeldChunkWhoseReleaseNeverArrivesBeforeItsDeadline) {
  RobotAgent agent = agent_at(1'000, grouped_config());
  const core::Job job = admitted_job(agent, at_ms(1'000));
  ASSERT_EQ(agent.on_chunk(chunk_of(job), at_ms(1'100)), ChunkVerdict::kHeld);

  static_cast<void>(agent.observe(at_ms(1'201)));

  EXPECT_EQ(agent.counters().chunks_missed, 1U);
  EXPECT_EQ(agent.on_release(release_of({job.id}, true, at_ms(1'210)), at_ms(1'210)),
            ReleaseVerdict::kIgnored);
  const Counters& counters = agent.counters();
  const std::uint64_t open = agent.in_flight() ? 1U : 0U;
  EXPECT_EQ(counters.jobs_submitted,
            counters.chunks_accepted + counters.jobs_rejected + counters.chunks_missed + open);
}

TEST(RobotAgent, JudgesAReleaseByItsReleaseTimeRatherThanByWhenItIsProcessed) {
  RobotAgent agent = agent_at(1'000, grouped_config());
  const core::Job job = admitted_job(agent, at_ms(1'000));
  ASSERT_EQ(agent.on_chunk(chunk_of(job), at_ms(1'100)), ChunkVerdict::kHeld);

  EXPECT_EQ(agent.on_release(release_of({job.id}, true, at_ms(1'190)), at_ms(1'205)),
            ReleaseVerdict::kAccepted);
  EXPECT_EQ(agent.buffer_empty_time(), at_ms(1'490));
}

TEST(RobotAgent, AlignsEveryTeamMemberOnTheReleaseTimeline) {
  RobotAgent early = agent_at(1'000, grouped_config());
  RobotAgent late = agent_at(1'100, grouped_config());
  const core::Job first = admitted_job(early, at_ms(1'100));
  const core::Job second = admitted_job(late, at_ms(1'100));
  ASSERT_EQ(early.on_chunk(chunk_of(first), at_ms(1'150)), ChunkVerdict::kHeld);
  ASSERT_EQ(late.on_chunk(chunk_of(second), at_ms(1'150)), ChunkVerdict::kHeld);

  ASSERT_EQ(early.on_release(release_of({first.id, second.id}, true, at_ms(1'160)), at_ms(1'161)),
            ReleaseVerdict::kAccepted);
  ASSERT_EQ(late.on_release(release_of({first.id, second.id}, true, at_ms(1'160)), at_ms(1'164)),
            ReleaseVerdict::kAccepted);

  EXPECT_EQ(early.buffer_empty_time(), at_ms(1'460));
  EXPECT_EQ(late.buffer_empty_time(), at_ms(1'460));
}

TEST(RobotAgent, CreateRejectsAGroupedBufferThatRefillsLaterThanOneHorizon) {
  AgentConfig grouped = grouped_config();
  grouped.maximum_buffer = ms(601);
  AgentConfig ungrouped = reference_config();
  ungrouped.maximum_buffer = ms(1'000);

  EXPECT_FALSE(RobotAgent::create(grouped, at_ms(0)).has_value());
  EXPECT_TRUE(RobotAgent::create(ungrouped, at_ms(0)).has_value());
}

TEST(RobotAgent, AppliesAReleaseThatArrivesBeforeItsOwnChunk) {
  RobotAgent agent = agent_at(1'000, grouped_config());
  const core::Job job = admitted_job(agent, at_ms(1'000));

  EXPECT_EQ(agent.on_release(release_of({job.id}, true, at_ms(1'150)), at_ms(1'150)),
            ReleaseVerdict::kPending);
  EXPECT_EQ(agent.on_chunk(chunk_of(job), at_ms(1'152)), ChunkVerdict::kAccepted);

  EXPECT_EQ(agent.buffer_empty_time(), at_ms(1'450));
  EXPECT_FALSE(agent.in_flight().has_value());
  EXPECT_EQ(agent.counters().chunks_accepted, 1U);
}

TEST(RobotAgent, AppliesAReleaseThatArrivesBeforeTheAdmissionDecision) {
  RobotAgent agent = agent_at(1'000, grouped_config());
  const core::Job job = agent.observe(at_ms(1'000)).value();

  EXPECT_EQ(agent.on_release(release_of({job.id}, true, at_ms(1'150)), at_ms(1'150)),
            ReleaseVerdict::kPending);
  EXPECT_TRUE(agent.on_decision(job.id, core::DecisionStatus::kAdmitted, kWorker));
  EXPECT_EQ(agent.on_chunk(chunk_of(job), at_ms(1'152)), ChunkVerdict::kAccepted);

  EXPECT_EQ(agent.buffer_empty_time(), at_ms(1'450));
  EXPECT_EQ(agent.counters().chunks_accepted, 1U);
}

TEST(RobotAgent, MissesAnEarlyReleaseStampedAfterItsDeadline) {
  RobotAgent agent = agent_at(1'000, grouped_config());
  const core::Job job = admitted_job(agent, at_ms(1'000));

  EXPECT_EQ(agent.on_release(release_of({job.id}, true, at_ms(1'201)), at_ms(1'205)),
            ReleaseVerdict::kPending);
  EXPECT_EQ(agent.on_chunk(chunk_of(job), at_ms(1'206)), ChunkVerdict::kLate);

  EXPECT_EQ(agent.counters().chunks_missed, 1U);
  EXPECT_EQ(agent.counters().chunks_accepted, 0U);
  EXPECT_EQ(agent.buffer_empty_time(), at_ms(1'250));
}

TEST(RobotAgent, JudgesAGroupedChunkProcessedAfterItsDeadlineByItsRelease) {
  RobotAgent agent = agent_at(1'000, grouped_config());
  const core::Job job = admitted_job(agent, at_ms(1'000));

  EXPECT_EQ(agent.on_chunk(chunk_of(job), at_ms(1'210)), ChunkVerdict::kHeld);
  EXPECT_EQ(agent.on_release(release_of({job.id}, true, at_ms(1'195)), at_ms(1'212)),
            ReleaseVerdict::kAccepted);
  EXPECT_EQ(agent.counters().chunks_accepted, 1U);
}

TEST(RobotAgent, WaitsTheReleaseGraceBeforeAbandoningAGroupedJob) {
  AgentConfig config = grouped_config();
  config.release_grace = ms(100);
  RobotAgent agent = agent_at(1'000, config);
  const core::Job job = admitted_job(agent, at_ms(1'000));
  ASSERT_EQ(agent.on_chunk(chunk_of(job), at_ms(1'150)), ChunkVerdict::kHeld);

  EXPECT_FALSE(agent.observe(at_ms(1'250)).has_value());
  EXPECT_TRUE(agent.in_flight().has_value());
  EXPECT_EQ(agent.on_release(release_of({job.id}, true, at_ms(1'199)), at_ms(1'260)),
            ReleaseVerdict::kAccepted);

  const core::Job next = admitted_job(agent, at_ms(1'500));
  static_cast<void>(agent.observe(at_ms(1'801)));
  EXPECT_EQ(agent.counters().chunks_missed, 0U);
  static_cast<void>(agent.observe(at_ms((next.buffer_empty_time.ns() / 1'000'000) + 51)));
  EXPECT_EQ(agent.counters().chunks_missed, 1U);
}

TEST(RobotAgent, IgnoresImplausibleOrOversizedReleases) {
  RobotAgent agent = agent_at(1'000, grouped_config());
  const core::Job job = admitted_job(agent, at_ms(1'000));
  ASSERT_EQ(agent.on_chunk(chunk_of(job), at_ms(1'150)), ChunkVerdict::kHeld);
  std::vector<core::JobId> oversized(kMaximumReleaseJobs + 1, core::JobId{7});
  oversized.push_back(job.id);

  EXPECT_EQ(agent.on_release(release_of({job.id}, true, at_ms(1'170)), at_ms(1'160)),
            ReleaseVerdict::kIgnored);
  EXPECT_EQ(agent.on_release(release_of({job.id}, true, at_ms(999)), at_ms(1'160)),
            ReleaseVerdict::kIgnored);
  EXPECT_EQ(agent.on_release(release_of(oversized, true, at_ms(1'155)), at_ms(1'160)),
            ReleaseVerdict::kIgnored);
  EXPECT_EQ(agent.on_release(release_of({job.id}, true, at_ms(1'155)), at_ms(1'160)),
            ReleaseVerdict::kAccepted);
}

TEST(RobotAgent, CreateRejectsANegativeReleaseGrace) {
  AgentConfig config = grouped_config();
  config.release_grace = ms(-1);

  EXPECT_FALSE(RobotAgent::create(config, at_ms(0)).has_value());
}

}  // namespace
}  // namespace rovi::robot_agent
