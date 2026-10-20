#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_components/component_manager.hpp>

#include "rovi_interfaces/msg/component_settings.hpp"
#include "rovi_interfaces/msg/decision.hpp"
#include "rovi_interfaces/msg/job_request.hpp"
#include "rovi_interfaces/msg/payload.hpp"
#include "rovi_interfaces/msg/request_receipt.hpp"
#include "rovi_interfaces/msg/reservation_violation.hpp"
#include "rovi_interfaces/msg/robot_state.hpp"
#include "rovi_interfaces/msg/run_invalidation.hpp"
#include "rovi_interfaces/msg/run_metadata.hpp"
#include "rovi_interfaces/msg/scheduler_cycle.hpp"
#include "rovi_interfaces/msg/service_source.hpp"

#include "slice_fixture.hpp"

namespace rovi::tests {
namespace {

using rovi_interfaces::msg::ComponentSettings;
using rovi_interfaces::msg::Decision;
using rovi_interfaces::msg::Payload;
using rovi_interfaces::msg::RequestReceipt;
using rovi_interfaces::msg::ReservationViolation;
using rovi_interfaces::msg::RobotState;
using rovi_interfaces::msg::RunInvalidation;
using rovi_interfaces::msg::RunMetadata;
using rovi_interfaces::msg::SchedulerCycle;
using rovi_interfaces::msg::ServiceSource;

constexpr std::int64_t kRunEndNs = kStartNs + 2'500'000'000;
constexpr std::size_t kMinimumChunks = 5;

class VerticalSlice : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

const Decision* admission_of(const std::vector<Decision>& decisions, std::uint64_t job) {
  const auto found = std::ranges::find_if(decisions, [job](const Decision& decision) {
    return decision.job_id == job && decision.status == Decision::STATUS_ADMITTED;
  });
  return found == decisions.end() ? nullptr : &*found;
}

bool accounts_for_every_job(const RobotState& state) {
  const std::uint64_t open = state.has_job_in_flight ? 1U : 0U;
  return state.jobs_submitted ==
         state.chunks_accepted + state.jobs_rejected + state.chunks_missed + open;
}

bool accounts_for_every_receipt(const RobotState& state) {
  return state.requests_offered == state.receipts_on_time + state.receipts_late +
                                       state.requests_missing + state.requests_pending;
}

std::size_t count_events(const std::vector<RequestReceipt>& receipts, std::uint8_t event) {
  return static_cast<std::size_t>(std::ranges::count(receipts, event, &RequestReceipt::event));
}

std::string setting(const std::vector<ComponentSettings>& settings, const std::string& node,
                    const std::string& name) {
  const auto component = std::ranges::find(settings, node, &ComponentSettings::node);
  if (component == settings.end()) {
    return "<no component>";
  }
  const auto entry = std::ranges::find(component->names, name);
  if (entry == component->names.end()) {
    return "<no parameter>";
  }
  return component->values.at(static_cast<std::size_t>(entry - component->names.begin()));
}

TEST_F(VerticalSlice, DeliversAdmittedActionChunksWithinTheirReservedWindows) {
  Slice slice{{}};

  slice.run_until(kRunEndNs);

  ASSERT_TRUE(slice.metadata().has_value());
  const RunMetadata& metadata = slice.metadata().value();
  EXPECT_EQ(metadata.interface_version, RunMetadata::INTERFACE_VERSION);
  EXPECT_EQ(metadata.run_id, "vertical_slice");
  EXPECT_EQ(metadata.network_profile_id, "synthetic-single-path-v1");
  EXPECT_EQ(metadata.git_revision, "unknown");
  EXPECT_EQ(metadata.service_source.value, ServiceSource::LOGICAL);
  EXPECT_EQ(metadata.policy, RunMetadata::POLICY_LEAST_SLACK);
  EXPECT_EQ(metadata.epoch_ns, kStartNs);
  EXPECT_NE(metadata.config_digest, 0U);

  ASSERT_GE(slice.chunks().size(), kMinimumChunks);
  for (const Payload& chunk : slice.chunks()) {
    const Decision* decision = admission_of(slice.decisions(), chunk.job_id);
    ASSERT_NE(decision, nullptr) << "job " << chunk.job_id;
    ASSERT_TRUE(decision->has_reservation);
    EXPECT_EQ(chunk.kind, Payload::KIND_ACTION_CHUNK);
    EXPECT_EQ(chunk.robot_id, 0U);
    EXPECT_EQ(chunk.worker_id, decision->reservation.worker_id);
    EXPECT_EQ(chunk.service_source.value, ServiceSource::LOGICAL);
    EXPECT_GE(chunk.delivered_ns, decision->reservation.delivery_ns) << "job " << chunk.job_id;
    EXPECT_LE(chunk.delivered_ns, decision->reservation.delivery_ns + kStepNs)
        << "job " << chunk.job_id;
    EXPECT_LE(chunk.delivered_ns, decision->reservation.deadline_ns) << "job " << chunk.job_id;
    EXPECT_GE(decision->reservation.slack_ns, 0);
  }
  for (const Decision& decision : slice.decisions()) {
    EXPECT_EQ(decision.status, Decision::STATUS_ADMITTED) << "job " << decision.job_id;
  }
  EXPECT_TRUE(slice.violations().empty());

  ASSERT_FALSE(slice.cycles().empty());
  std::uint64_t admitted_in_cycles = 0;
  for (const SchedulerCycle& cycle : slice.cycles()) {
    EXPECT_GE(cycle.runtime_ns, 0);
    EXPECT_LE(cycle.window_start_ns, cycle.now_ns);
    admitted_in_cycles += cycle.admitted;
  }
  EXPECT_EQ(admitted_in_cycles, slice.decisions().size());

  ASSERT_TRUE(slice.last_state().has_value());
  const RobotState& state = slice.last_state().value();
  EXPECT_EQ(state.robot_id, 0U);
  EXPECT_EQ(state.mode, RobotState::MODE_EXECUTING);
  EXPECT_GT(state.buffer_remaining_ns, 0);
  EXPECT_EQ(state.fallback_entries, 0U);
  EXPECT_EQ(state.jobs_rejected, 0U);
  EXPECT_EQ(state.chunks_late, 0U);
  EXPECT_EQ(state.chunks_stale, 0U);
  EXPECT_EQ(state.chunks_missed, 0U);
  EXPECT_GE(state.chunks_accepted, kMinimumChunks);
  EXPECT_TRUE(accounts_for_every_job(state));

  EXPECT_EQ(state.requests_offered, state.jobs_submitted);
  EXPECT_EQ(state.receipts_on_time, state.chunks_accepted);
  EXPECT_EQ(state.receipts_late, 0U);
  EXPECT_EQ(state.requests_missing, 0U);
  EXPECT_TRUE(accounts_for_every_receipt(state));
  EXPECT_GE(count_events(slice.receipts(), RequestReceipt::EVENT_RECEIVED), kMinimumChunks);
  for (const RequestReceipt& received : slice.receipts()) {
    if (received.event != RequestReceipt::EVENT_RECEIVED) {
      continue;
    }
    const auto offered = std::ranges::find_if(slice.receipts(), [&](const RequestReceipt& event) {
      return event.event == RequestReceipt::EVENT_OFFERED && event.job_id == received.job_id;
    });
    ASSERT_NE(offered, slice.receipts().end()) << "job " << received.job_id;
    EXPECT_EQ(offered->deadline_ns, received.deadline_ns);
    EXPECT_LE(offered->stamp_ns, received.stamp_ns);
    EXPECT_LE(received.stamp_ns, received.deadline_ns) << "job " << received.job_id;
  }
}

TEST_F(VerticalSlice, EveryComponentPublishesItsResolvedSettingsOnce) {
  Slice slice{{{"rovi_worker_1", {"slowdown_ns:=7"}}}};

  slice.run_until(kStartNs + 10'000'000);

  std::vector<std::string> nodes;
  for (const ComponentSettings& component : slice.settings()) {
    nodes.push_back(component.node);
    EXPECT_EQ(component.names.size(), component.values.size()) << component.node;
    EXPECT_TRUE(std::ranges::is_sorted(component.names)) << component.node;
  }
  std::ranges::sort(nodes);
  EXPECT_EQ(nodes, (std::vector<std::string>{"/robot_00/robot_agent", "/rovi_link",
                                             "/rovi_scheduler", "/rovi_worker_1"}));
  EXPECT_EQ(setting(slice.settings(), "/rovi_worker_1", "slowdown_ns"), "7");
  EXPECT_EQ(setting(slice.settings(), "/rovi_scheduler", "dispatch_lead_ns"), "10000000");
  EXPECT_EQ(setting(slice.settings(), "/robot_00/robot_agent", "release_grace_ns"), "0");
  EXPECT_EQ(setting(slice.settings(), "/rovi_link", "use_sim_time"), "true");
}

// A backward clock invalidates the run, and no component consumes work from before the jump.
TEST_F(VerticalSlice, ABackwardClockInvalidatesEveryComponentAndStopsAllWork) {
  constexpr std::int64_t kJumpNs = kStartNs + 500'000'000;
  constexpr std::int64_t kRewoundNs = kStartNs + 100'000'000;
  Slice slice{{}};
  slice.run_until(kJumpNs);
  ASSERT_TRUE(slice.invalidations().empty());
  ASSERT_FALSE(slice.decisions().empty());
  ASSERT_TRUE(slice.last_state().has_value());
  const std::size_t decisions = slice.decisions().size();
  const std::size_t chunks = slice.chunks().size();
  const std::size_t cycles = slice.cycles().size();
  const std::size_t receipts = slice.receipts().size();
  const std::int64_t last_state = slice.last_state()->stamp_ns;

  slice.rewind_to(kRewoundNs);
  slice.run_until(kJumpNs + 500'000'000);

  std::vector<std::string> nodes;
  for (const RunInvalidation& invalidation : slice.invalidations()) {
    nodes.push_back(invalidation.node);
    EXPECT_GE(invalidation.observed_ns, kRewoundNs) << invalidation.node;
    EXPECT_LT(invalidation.observed_ns, invalidation.latest_ns) << invalidation.node;
    EXPECT_LE(invalidation.latest_ns, kJumpNs) << invalidation.node;
  }
  std::ranges::sort(nodes);
  EXPECT_EQ(nodes, (std::vector<std::string>{"/robot_00/robot_agent", "/rovi_link",
                                             "/rovi_scheduler", "/rovi_worker_1"}));
  EXPECT_EQ(slice.decisions().size(), decisions);
  EXPECT_EQ(slice.chunks().size(), chunks);
  EXPECT_EQ(slice.cycles().size(), cycles);
  EXPECT_EQ(slice.receipts().size(), receipts);
  EXPECT_EQ(slice.last_state()->stamp_ns, last_state);
}

// One rewound clock tick that no component callback ever reads, followed by recovery past the
// latest time, still invalidates every component.
TEST_F(VerticalSlice, ARewindRecoveredBetweenCallbacksStillStopsAllWork) {
  constexpr std::int64_t kJumpNs = kStartNs + 500'000'000;
  constexpr std::int64_t kRewoundNs = kStartNs + 100'000'000;
  Slice slice{{}};
  slice.run_until(kJumpNs);
  ASSERT_FALSE(slice.decisions().empty());
  ASSERT_TRUE(slice.last_state().has_value());
  const std::size_t decisions = slice.decisions().size();
  const std::size_t chunks = slice.chunks().size();
  const std::size_t cycles = slice.cycles().size();
  const std::int64_t last_state = slice.last_state()->stamp_ns;

  slice.rewind_to(kRewoundNs);
  slice.run_until(kRewoundNs);
  slice.rewind_to(kJumpNs + kStepNs);
  slice.run_until(kJumpNs + 500'000'000);

  std::vector<std::string> nodes;
  for (const RunInvalidation& invalidation : slice.invalidations()) {
    nodes.push_back(invalidation.node);
    EXPECT_EQ(invalidation.observed_ns, kRewoundNs) << invalidation.node;
    EXPECT_EQ(invalidation.latest_ns, kJumpNs) << invalidation.node;
  }
  std::ranges::sort(nodes);
  EXPECT_EQ(nodes, (std::vector<std::string>{"/robot_00/robot_agent", "/rovi_link",
                                             "/rovi_scheduler", "/rovi_worker_1"}));
  EXPECT_EQ(slice.decisions().size(), decisions);
  EXPECT_EQ(slice.chunks().size(), chunks);
  EXPECT_EQ(slice.cycles().size(), cycles);
  EXPECT_EQ(slice.last_state()->stamp_ns, last_state);
}

TEST_F(VerticalSlice, APeerInvalidationStopsComponentsThatNeverSawTheRewind) {
  constexpr std::int64_t kInvalidatedNs = kStartNs + 500'000'000;
  Slice slice{{}};
  slice.run_until(kInvalidatedNs);
  ASSERT_FALSE(slice.decisions().empty());
  ASSERT_TRUE(slice.last_state().has_value());
  const std::size_t decisions = slice.decisions().size();
  const std::size_t chunks = slice.chunks().size();
  const std::size_t cycles = slice.cycles().size();
  const std::int64_t last_state = slice.last_state()->stamp_ns;

  slice.invalidate_from("/simulator_bridge");
  slice.run_until(kInvalidatedNs + 500'000'000);

  ASSERT_EQ(slice.invalidations().size(), 1U);
  EXPECT_EQ(slice.invalidations().front().node, "/simulator_bridge");
  EXPECT_EQ(slice.decisions().size(), decisions);
  EXPECT_EQ(slice.chunks().size(), chunks);
  EXPECT_EQ(slice.cycles().size(), cycles);
  EXPECT_EQ(slice.last_state()->stamp_ns, last_state);
}

TEST_F(VerticalSlice, WorkerSlowdownMissesTheDownlinkLeaseAndTheRobotFallsBack) {
  Slice slice{{{"rovi_worker_1", {"slowdown_ns:=300000000"}}}};

  slice.run_until(kRunEndNs);

  EXPECT_TRUE(slice.chunks().empty());
  ASSERT_FALSE(slice.violations().empty());
  for (const ReservationViolation& violation : slice.violations()) {
    const Decision* decision = admission_of(slice.decisions(), violation.job_id);
    ASSERT_NE(decision, nullptr) << "job " << violation.job_id;
    EXPECT_EQ(violation.direction, ReservationViolation::DIRECTION_DOWNLINK);
    EXPECT_EQ(violation.reason, ReservationViolation::REASON_LATE_SEND);
    ASSERT_TRUE(violation.has_lease);
    EXPECT_EQ(violation.service_start_ns, decision->reservation.download_start_ns);
    EXPECT_GT(violation.sent_ns, violation.service_start_ns);
  }
  ASSERT_TRUE(slice.last_state().has_value());
  const RobotState& state = slice.last_state().value();
  EXPECT_EQ(state.chunks_accepted, 0U);
  EXPECT_EQ(state.chunks_late, 0U);
  EXPECT_GE(state.chunks_missed, 1U);
  EXPECT_GE(state.fallback_entries, 1U);
  EXPECT_TRUE(accounts_for_every_job(state));

  EXPECT_EQ(count_events(slice.receipts(), RequestReceipt::EVENT_RECEIVED), 0U);
  EXPECT_EQ(state.receipts_on_time, 0U);
  EXPECT_EQ(state.receipts_late, 0U);
  EXPECT_GE(state.requests_missing, 1U);
  EXPECT_TRUE(accounts_for_every_receipt(state));
  EXPECT_EQ(setting(slice.settings(), "/rovi_worker_1", "slowdown_ns"), "300000000");
}

TEST_F(VerticalSlice, IdenticalRunsProduceIdenticalDecisionsAndDeliveries) {
  std::vector<Decision> first_decisions;
  std::vector<Payload> first_chunks;
  {
    Slice slice{{}};
    slice.run_until(kRunEndNs);
    first_decisions = slice.decisions();
    first_chunks = slice.chunks();
  }
  Slice repeat{{}};

  repeat.run_until(kRunEndNs);

  ASSERT_GE(first_chunks.size(), kMinimumChunks);
  EXPECT_EQ(repeat.decisions(), first_decisions);
  EXPECT_EQ(repeat.chunks(), first_chunks);
}

rovi_interfaces::msg::JobRequest external_request(std::uint64_t job, std::int64_t release_ns) {
  constexpr std::int64_t kHorizonNs = 500'000'000;
  rovi_interfaces::msg::JobRequest request;
  request.job_id = job;
  request.robot_id = 5;
  request.sequence = job;
  request.release_time_ns = release_ns;
  request.observation_time_ns = release_ns;
  request.buffer_empty_time_ns = release_ns + kHorizonNs;
  request.has_maximum_result_age = true;
  request.maximum_result_age_ns = kHorizonNs;
  request.upload_bytes = 10'000;
  request.action_chunk_bytes = 1'000;
  request.model_class = 1;
  request.eligible_workers = {1};
  return request;
}

std::vector<std::pair<std::uint8_t, std::uint8_t>> outcomes_of(
    const std::vector<Decision>& decisions, std::uint64_t job) {
  std::vector<std::pair<std::uint8_t, std::uint8_t>> outcomes;
  for (const Decision& decision : decisions) {
    if (decision.job_id == job) {
      outcomes.emplace_back(decision.status, decision.reason);
    }
  }
  return outcomes;
}

TEST_F(VerticalSlice, SchedulerAnswersDuplicateAndFutureReleasedRequestsWithReasonCodes) {
  constexpr std::int64_t kSubmitNs = kStartNs + 20'000'000;
  constexpr std::int64_t kFarAheadNs = kSubmitNs + 1'000'000'000;
  Slice slice{{}};
  slice.run_until(kSubmitNs);

  slice.submit(external_request(9'000, kSubmitNs));
  slice.submit(external_request(9'000, kSubmitNs));
  slice.submit(external_request(9'001, kFarAheadNs));
  slice.run_until(kSubmitNs + 100'000'000);

  using Outcomes = std::vector<std::pair<std::uint8_t, std::uint8_t>>;
  EXPECT_EQ(outcomes_of(slice.decisions(), 9'000),
            (Outcomes{{Decision::STATUS_REJECTED, Decision::REASON_DUPLICATE_JOB},
                      {Decision::STATUS_ADMITTED, Decision::REASON_NONE}}));
  EXPECT_EQ(outcomes_of(slice.decisions(), 9'001),
            (Outcomes{{Decision::STATUS_REJECTED, Decision::REASON_INVALID_TIMING}}));
}

TEST_F(VerticalSlice, WaypointControllerLoadsTheShippedParametersAndRejectsInvalidOnes) {
  const Component controller{.package = "rovi_robot_agent",
                             .plugin = "rovi::robot_agent::WaypointControllerNode",
                             .name = "waypoint_controller",
                             .name_space = "/robot_00"};
  const auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  const auto manager = std::make_shared<rclcpp_components::ComponentManager>(executor);
  const auto factory = factory_for(*manager, controller);

  EXPECT_NO_THROW(static_cast<void>(
      factory->create_node_instance(options_for(controller, {}, "sim_one_robot"))));
  const std::vector<std::string> invalid_values{
      "robot_id:=-1",         "state_timeout_ns:=0", "odometry_timeout_ns:=-5",
      "control_period_ns:=0", "waypoints_xy:=[1.0]", "arrival_tolerance_m:=0.0",
      "max_linear_mps:=-1.0", "heading_gain:=0.0",   "control_period_ns:=2000000000"};
  for (const std::string& invalid : invalid_values) {
    EXPECT_ANY_THROW(static_cast<void>(
        factory->create_node_instance(options_for(controller, {invalid}, "sim_one_robot"))))
        << invalid;
  }
}

TEST_F(VerticalSlice, ArmControllerLoadsTheShippedParametersAndRejectsInvalidOnes) {
  const Component controller{.package = "rovi_robot_agent",
                             .plugin = "rovi::robot_agent::ArmControllerNode",
                             .name = "arm_controller",
                             .name_space = "/robot_00"};
  const auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  const auto manager = std::make_shared<rclcpp_components::ComponentManager>(executor);
  const auto factory = factory_for(*manager, controller);

  EXPECT_NO_THROW(static_cast<void>(
      factory->create_node_instance(options_for(controller, {}, "arm_workcell"))));
  const std::vector<std::string> invalid_values{
      "robot_id:=-1",
      "state_timeout_ns:=0",
      "feedback_timeout_ns:=-5",
      "control_period_ns:=0",
      "settle_time_ns:=0",
      "joint_names:=[a, a, b, c, d, e, f]",
      "goals_rad:=[0.1]",
      "max_joint_speed_rps:=0.0",
      "tolerance_rad:=-0.1",
      "joint_lower_rad:=[0.0]",
      "joint_upper_rad:=[0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]",
      "control_period_ns:=2000000000"};
  for (const std::string& invalid : invalid_values) {
    EXPECT_ANY_THROW(static_cast<void>(
        factory->create_node_instance(options_for(controller, {invalid}, "arm_workcell"))))
        << invalid;
  }
}

TEST_F(VerticalSlice, CoordinatorLoadsTheShippedParametersAndRejectsInvalidOnes) {
  const Component coordinator{.package = "rovi_coordination",
                              .plugin = "rovi::coordination::CoordinatorNode",
                              .name = "rovi_coordinator",
                              .name_space = "/"};
  const auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  const auto manager = std::make_shared<rclcpp_components::ComponentManager>(executor);
  const auto factory = factory_for(*manager, coordinator);

  const std::vector<std::string> boundary_values{"", "robots_per_workcell:=64",
                                                 "max_tracked_groups:=1024", "expiry_grace_ns:=0",
                                                 "expiry_period_ns:=1000000"};
  for (const std::string& boundary : boundary_values) {
    const std::vector<std::string> overrides =
        boundary.empty() ? std::vector<std::string>{} : std::vector<std::string>{boundary};
    EXPECT_NO_THROW(static_cast<void>(
        factory->create_node_instance(options_for(coordinator, overrides, "thirty_arms"))))
        << boundary;
  }
  const std::vector<std::string> invalid_values{"workcells:=0",
                                                "robots_per_workcell:=0",
                                                "robots_per_workcell:=65",
                                                "expiry_period_ns:=999999",
                                                "expiry_grace_ns:=-1",
                                                "maximum_result_ready_skew_ns:=-1",
                                                "max_tracked_groups:=0",
                                                "max_tracked_groups:=1025"};
  for (const std::string& invalid : invalid_values) {
    EXPECT_THROW(static_cast<void>(factory->create_node_instance(
                     options_for(coordinator, {invalid}, "thirty_arms"))),
                 std::invalid_argument)
        << invalid;
  }
}

TEST_F(VerticalSlice, EveryComponentLoadsWithTheShippedParametersAndRejectsAnInvalidOne) {
  const std::map<std::string, std::string> invalid{{"rovi_scheduler", "policy:=optimal"},
                                                   {"rovi_link", "poll_period_ns:=0"},
                                                   {"rovi_worker_1", "slowdown_ns:=-1"},
                                                   {"robot_agent", "chunk_horizon_ns:=0"}};
  const auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  const auto manager = std::make_shared<rclcpp_components::ComponentManager>(executor);

  for (const Component& component : vertical_slice()) {
    const auto factory = factory_for(*manager, component);
    EXPECT_NO_THROW(static_cast<void>(factory->create_node_instance(options_for(component, {}))))
        << component.plugin;
    EXPECT_ANY_THROW(static_cast<void>(
        factory->create_node_instance(options_for(component, {invalid.at(component.name)}))))
        << component.plugin;
  }
}

bool has_outcome(const std::vector<Decision>& decisions, std::uint8_t status, std::uint8_t reason) {
  return std::ranges::any_of(decisions, [status, reason](const Decision& decision) {
    return decision.robot_id == 0 && decision.status == status && decision.reason == reason;
  });
}

TEST_F(VerticalSlice, UplinkOutageDefersThenExpiresJobsAndTheRobotRecoversAfterwards) {
  constexpr std::int64_t kOutageEndNs = kStartNs + 1'100'000'000;
  Slice slice{{{"rovi_scheduler",
                {"horizon_slots:=500", "uplink.first_slots:=[0,100,1100]",
                 "uplink.rates_bps:=[100000000,0,100000000]",
                 "uplink.reductions_permille:=[100,100,100]"}}}};

  slice.run_until(kStartNs + 2'500'000'000);

  EXPECT_TRUE(has_outcome(slice.decisions(), Decision::STATUS_DEFERRED,
                          Decision::REASON_UPLINK_SERVICE_UNAVAILABLE));
  EXPECT_TRUE(
      has_outcome(slice.decisions(), Decision::STATUS_REJECTED, Decision::REASON_DEADLINE_MISS) ||
      has_outcome(slice.decisions(), Decision::STATUS_REJECTED, Decision::REASON_STALE_RESULT));
  // Once a chunk refills the empty buffer, the next request must fit: no rejection follows.
  const auto refill = std::ranges::find_if(
      slice.chunks(), [](const Payload& chunk) { return chunk.delivered_ns > kOutageEndNs; });
  ASSERT_NE(refill, slice.chunks().end());
  for (const Decision& decision : slice.decisions()) {
    EXPECT_TRUE(decision.decided_ns <= refill->delivered_ns ||
                decision.status != Decision::STATUS_REJECTED)
        << "job " << decision.job_id << " was rejected after the robot recovered";
  }
  ASSERT_TRUE(slice.last_state().has_value());
  const RobotState& state = slice.last_state().value();
  EXPECT_GE(state.fallback_entries, 1U);
  EXPECT_GE(state.jobs_rejected, 1U);
  EXPECT_EQ(state.mode, RobotState::MODE_EXECUTING);
  EXPECT_TRUE(accounts_for_every_job(state));
}

// A 100 ms initial buffer leaves the first request too little time, as a slow start does in a
// free-running launch. The first refill must restore steady admission, not alternate with misses.
TEST_F(VerticalSlice, OneMissedRequestDoesNotLockTheRobotIntoAlternatingMisses) {
  Slice slice{{{"robot_agent", {"initial_buffer_ns:=100000000"}}}};

  slice.run_until(kStartNs + 2'500'000'000);

  ASSERT_TRUE(
      has_outcome(slice.decisions(), Decision::STATUS_REJECTED, Decision::REASON_DEADLINE_MISS));
  ASSERT_FALSE(slice.chunks().empty());
  const std::int64_t refill = slice.chunks().front().delivered_ns;
  for (const Decision& decision : slice.decisions()) {
    EXPECT_TRUE(decision.decided_ns <= refill || decision.status != Decision::STATUS_REJECTED)
        << "job " << decision.job_id << " was rejected after the first refill";
  }
  ASSERT_TRUE(slice.last_state().has_value());
  EXPECT_EQ(slice.last_state()->mode, RobotState::MODE_EXECUTING);
  EXPECT_EQ(slice.last_state()->fallback_entries, 1U);
}

TEST_F(VerticalSlice, DependentJobWaitsForTheActualDeliveryOfItsPredecessor) {
  constexpr std::int64_t kSubmitNs = kStartNs + 20'000'000;
  constexpr std::uint64_t kFirst = 9'100;
  constexpr std::uint64_t kDependent = 9'101;
  Slice slice{{}};
  slice.run_until(kSubmitNs);
  rovi_interfaces::msg::JobRequest dependent = external_request(kDependent, kSubmitNs);
  dependent.predecessors = {kFirst};

  slice.submit(external_request(kFirst, kSubmitNs));
  slice.submit(dependent);
  std::int64_t now_ns = kSubmitNs;
  const Decision* first = nullptr;
  while (first == nullptr && now_ns < kSubmitNs + 100'000'000) {
    now_ns += kStepNs;
    slice.run_until(now_ns);
    first = admission_of(slice.decisions(), kFirst);
  }
  ASSERT_NE(first, nullptr);
  Payload observation;
  observation.kind = Payload::KIND_OBSERVATION;
  observation.job_id = kFirst;
  observation.robot_id = first->robot_id;
  observation.worker_id = first->reservation.worker_id;
  observation.bytes = first->reservation.upload_bytes;
  observation.observation_time_ns = kSubmitNs;
  observation.sent_ns = now_ns;
  slice.upload(observation);
  slice.run_until(now_ns + 300'000'000);

  using Outcomes = std::vector<std::pair<std::uint8_t, std::uint8_t>>;
  EXPECT_EQ(outcomes_of(slice.decisions(), kDependent),
            (Outcomes{{Decision::STATUS_DEFERRED, Decision::REASON_PREDECESSOR_INCOMPLETE},
                      {Decision::STATUS_ADMITTED, Decision::REASON_NONE}}));
  const auto delivered = std::ranges::find(slice.chunks(), kFirst, &Payload::job_id);
  ASSERT_NE(delivered, slice.chunks().end());
  const Decision* second = admission_of(slice.decisions(), kDependent);
  ASSERT_NE(second, nullptr);
  EXPECT_GE(second->decided_ns, delivered->delivered_ns);
}

}  // namespace
}  // namespace rovi::tests
