#include "rovi_robot_agent/messages.hpp"

#include <cstdint>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "rovi_core/admission.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/units.hpp"
#include "rovi_interfaces/msg/decision.hpp"
#include "rovi_interfaces/msg/job_request.hpp"
#include "rovi_interfaces/msg/robot_state.hpp"
#include "rovi_robot_agent/robot_agent.hpp"

namespace rovi::robot_agent {
namespace {

using rovi_interfaces::msg::Decision;
using rovi_interfaces::msg::RobotState;

constexpr std::int64_t kNsPerMs = 1'000'000;

core::TimePoint at_ms(std::int64_t count) { return core::TimePoint::from_ns(count * kNsPerMs); }

core::Duration ms(std::int64_t count) { return core::Duration::from_ns(count * kNsPerMs); }

TEST(RobotAgentMessages, JobRequestCarriesEveryJobField) {
  const core::Job job{.id = core::JobId{42},
                      .release_time = core::TimePoint::from_ns(100),
                      .observation_time = core::TimePoint::from_ns(90),
                      .buffer_empty_time = core::TimePoint::from_ns(900),
                      .safety_margin = core::Duration::from_ns(50),
                      .task_deadline = core::TimePoint::from_ns(800),
                      .maximum_result_age = core::Duration::from_ns(400),
                      .upload_bytes = core::Bytes{1'000},
                      .action_chunk_bytes = core::Bytes{200},
                      .model = core::ModelClass{3},
                      .eligible_workers = {core::WorkerId{1}, core::WorkerId{2}},
                      .robot = core::RobotId{7},
                      .predecessors = {core::JobId{40}},
                      .synchronization_group = core::GroupId{5},
                      .maximum_result_ready_skew = core::Duration::from_ns(30),
                      .sequence = 9};

  const rovi_interfaces::msg::JobRequest request = to_request(job);

  EXPECT_EQ(request.job_id, 42U);
  EXPECT_EQ(request.robot_id, 7U);
  EXPECT_EQ(request.sequence, 9U);
  EXPECT_EQ(request.release_time_ns, 100);
  EXPECT_EQ(request.observation_time_ns, 90);
  EXPECT_EQ(request.buffer_empty_time_ns, 900);
  EXPECT_EQ(request.safety_margin_ns, 50);
  EXPECT_TRUE(request.has_task_deadline);
  EXPECT_EQ(request.task_deadline_ns, 800);
  EXPECT_TRUE(request.has_maximum_result_age);
  EXPECT_EQ(request.maximum_result_age_ns, 400);
  EXPECT_EQ(request.upload_bytes, 1'000);
  EXPECT_EQ(request.action_chunk_bytes, 200);
  EXPECT_EQ(request.model_class, 3U);
  EXPECT_EQ(request.eligible_workers, (std::vector<std::uint64_t>{1, 2}));
  EXPECT_EQ(request.predecessors, (std::vector<std::uint64_t>{40}));
  EXPECT_TRUE(request.has_synchronization_group);
  EXPECT_EQ(request.synchronization_group, 5U);
  EXPECT_EQ(request.maximum_result_ready_skew_ns, 30);
}

TEST(RobotAgentMessages, JobRequestMarksAbsentOptionalFields) {
  const core::Job job{.id = core::JobId{1},
                      .release_time = core::TimePoint{},
                      .observation_time = core::TimePoint{},
                      .buffer_empty_time = core::TimePoint{},
                      .safety_margin = core::Duration{},
                      .task_deadline = std::nullopt,
                      .maximum_result_age = std::nullopt,
                      .upload_bytes = core::Bytes{1},
                      .action_chunk_bytes = core::Bytes{1},
                      .model = core::ModelClass{1},
                      .eligible_workers = {core::WorkerId{1}}};

  const rovi_interfaces::msg::JobRequest request = to_request(job);

  EXPECT_FALSE(request.has_task_deadline);
  EXPECT_FALSE(request.has_maximum_result_age);
  EXPECT_FALSE(request.has_synchronization_group);
}

TEST(RobotAgentMessages, DecodesEveryDecisionStatusAndRejectsUnknownValues) {
  EXPECT_EQ(status_from(Decision::STATUS_REJECTED), core::DecisionStatus::kRejected);
  EXPECT_EQ(status_from(Decision::STATUS_DEFERRED), core::DecisionStatus::kDeferred);
  EXPECT_EQ(status_from(Decision::STATUS_ADMITTED), core::DecisionStatus::kAdmitted);
  EXPECT_EQ(status_from(3), std::nullopt);
}

TEST(RobotAgentMessages, StateReportsBufferModeInFlightJobAndCounters) {
  RobotAgent agent = RobotAgent::create(AgentConfig{.robot = core::RobotId{3},
                                                    .initial_buffer = ms(250),
                                                    .chunk_horizon = ms(200),
                                                    .maximum_buffer = ms(1'000),
                                                    .safety_margin = ms(50),
                                                    .maximum_result_age = ms(400),
                                                    .upload_bytes = core::Bytes{1},
                                                    .action_chunk_bytes = core::Bytes{1},
                                                    .model = core::ModelClass{1},
                                                    .eligible_workers = {core::WorkerId{1}}},
                                        at_ms(1'000))
                         .value();
  const core::JobId job = agent.observe(at_ms(1'000)).value().id;

  const RobotState executing = to_state(agent, at_ms(1'100));
  agent.refresh(at_ms(1'300));
  const RobotState stopped = to_state(agent, at_ms(1'300));

  EXPECT_EQ(executing.robot_id, 3U);
  EXPECT_EQ(executing.stamp_ns, 1'100 * kNsPerMs);
  EXPECT_EQ(executing.mode, RobotState::MODE_EXECUTING);
  EXPECT_EQ(executing.buffer_empty_time_ns, 1'250 * kNsPerMs);
  EXPECT_EQ(executing.buffer_remaining_ns, 150 * kNsPerMs);
  EXPECT_TRUE(executing.has_job_in_flight);
  EXPECT_EQ(executing.job_in_flight, job.value());
  EXPECT_EQ(executing.jobs_submitted, 1U);
  EXPECT_EQ(stopped.mode, RobotState::MODE_FALLBACK);
  EXPECT_EQ(stopped.buffer_remaining_ns, 0);
  EXPECT_EQ(stopped.fallback_entries, 1U);
}

TEST(RobotAgentMessages, StateReportsReceiptOutcomesAtItsStamp) {
  RobotAgent agent = RobotAgent::create(AgentConfig{.robot = core::RobotId{3},
                                                    .initial_buffer = ms(250),
                                                    .chunk_horizon = ms(200),
                                                    .maximum_buffer = ms(1'000),
                                                    .safety_margin = ms(50),
                                                    .maximum_result_age = ms(400),
                                                    .upload_bytes = core::Bytes{1},
                                                    .action_chunk_bytes = core::Bytes{1},
                                                    .model = core::ModelClass{1},
                                                    .eligible_workers = {core::WorkerId{1}}},
                                        at_ms(1'000))
                         .value();
  const core::Job first = agent.observe(at_ms(1'000)).value();
  ASSERT_TRUE(agent.on_decision(first.id, core::DecisionStatus::kAdmitted, core::WorkerId{1}));
  ASSERT_EQ(agent.on_chunk(ChunkReceipt{.job = first.id,
                                        .worker = core::WorkerId{1},
                                        .observation_time = first.observation_time},
                           at_ms(1'100)),
            ChunkVerdict::kAccepted);
  ASSERT_TRUE(agent.observe(at_ms(1'100)).has_value());

  const RobotState open = to_state(agent, at_ms(1'400));
  const RobotState closed = to_state(agent, at_ms(1'401));

  EXPECT_EQ(open.requests_offered, 2U);
  EXPECT_EQ(open.receipts_on_time, 1U);
  EXPECT_EQ(open.receipts_late, 0U);
  EXPECT_EQ(open.requests_missing, 0U);
  EXPECT_EQ(open.requests_pending, 1U);
  EXPECT_EQ(closed.requests_missing, 1U);
  EXPECT_EQ(closed.requests_pending, 0U);
}

}  // namespace
}  // namespace rovi::robot_agent
