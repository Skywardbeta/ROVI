#include "rovi_core/window.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "rovi_core/admission.hpp"
#include "rovi_core/audit.hpp"
#include "rovi_core/capacity_calendar.hpp"
#include "rovi_core/interval_calendar.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/scheduler.hpp"
#include "rovi_core/units.hpp"
#include "rovi_core/wireless_profile.hpp"

#include "printers.hpp"  // IWYU pragma: keep
#include "scenario.hpp"
#include "test_support.hpp"

namespace rovi::core {
namespace {

constexpr std::int64_t kWindowSlots = 10;
constexpr std::int64_t kUplinkRate = 1'000'000'000;
constexpr std::int64_t kDownlinkRate = 400'000'000;
constexpr std::int64_t kOutageSlot = 11;

ServiceSegment rate(std::int64_t first_slot, std::int64_t bits_per_second) {
  return ServiceSegment{
      .first_slot = first_slot, .rate = BitsPerSecond{bits_per_second}, .reduction = Permille{0}};
}

ServiceProfile reference_profile() {
  return ServiceProfile{.slot_length = ms(1),
                        .uplink = {rate(0, kUplinkRate), rate(kOutageSlot, 0)},
                        .downlink = {rate(0, kDownlinkRate)},
                        .uplink_delay = ms(2),
                        .downlink_delay = ms(3)};
}

ServiceProfile profile_with_medium() {
  ServiceProfile profile = reference_profile();
  profile.medium = std::vector<ServiceSegment>{rate(0, 800'000'000)};
  return profile;
}

ExecutionProfile inference(std::uint32_t model, std::int64_t duration_ms) {
  return ExecutionProfile{.model = ModelClass{model}, .duration = ms(duration_ms)};
}

std::vector<WorkerSpec> one_worker() {
  return {WorkerSpec{.id = WorkerId{1}, .profiles = {inference(1, 10)}}};
}

ResourceSnapshot opened(const ServiceProfile& profile, std::int64_t slots = kWindowSlots) {
  return open_window(profile, one_worker(), at_ms(100), slots).value();
}

std::vector<std::int64_t> capacities_of(const CapacityCalendar& link) {
  std::vector<std::int64_t> result;
  result.reserve(static_cast<std::size_t>(link.slot_count()));
  for (std::int64_t slot = 0; slot < link.slot_count(); ++slot) {
    result.push_back(link.capacity(SlotIndex{slot}).value());
  }
  return result;
}

std::vector<std::int64_t> repeated(std::int64_t value, std::int64_t count) {
  return std::vector<std::int64_t>(static_cast<std::size_t>(count), value);
}

Transfer slices_at(const std::vector<std::pair<std::int64_t, std::int64_t>>& slots,
                   TimePoint completion) {
  Transfer transfer{.slices = {}, .completion = completion};
  for (const auto& [slot, amount] : slots) {
    transfer.slices.push_back(ServiceSlice{.slot = SlotIndex{slot}, .bytes = Bytes{amount}});
  }
  return transfer;
}

TEST(ResourceWindow, OpensProfileCapacityAndEmptyLanesAtTheEpoch) {
  const std::optional<ResourceSnapshot> window =
      open_window(reference_profile(), one_worker(), at_ms(100), kWindowSlots);

  ASSERT_TRUE(window.has_value());
  EXPECT_TRUE(is_consistent(window.value()));
  EXPECT_EQ(window->uplink.slot_start(SlotIndex{0}), at_ms(100));
  EXPECT_EQ(window->uplink.horizon_end(), at_ms(110));
  EXPECT_EQ(capacities_of(window->uplink), repeated(125'000, kWindowSlots));
  EXPECT_EQ(capacities_of(window->downlink), repeated(50'000, kWindowSlots));
  EXPECT_FALSE(window->shared_medium.has_value());
  EXPECT_EQ(window->uplink_delay, ms(2));
  EXPECT_EQ(window->downlink_delay, ms(3));
  EXPECT_EQ(window->source, ServiceSource::kLogical);
  EXPECT_EQ(window->workers, (std::vector<Worker>{worker_with(1, {inference(1, 10)})}));
}

TEST(ResourceWindow, OpensSharedMediumAndKeepsTheProfileSourceLabel) {
  ServiceProfile profile = profile_with_medium();
  profile.source = ServiceSource::kReplayed;

  const std::optional<ResourceSnapshot> window =
      open_window(profile, one_worker(), at_ms(100), kWindowSlots);

  ASSERT_TRUE(window.has_value());
  ASSERT_TRUE(window->shared_medium.has_value());
  EXPECT_EQ(capacities_of(window->shared_medium.value()), repeated(100'000, kWindowSlots));
  EXPECT_EQ(window->source, ServiceSource::kReplayed);
}

TEST(ResourceWindow, OpenRejectsInvalidGeometryProfilesOrWorkers) {
  ServiceProfile bad_segments = reference_profile();
  bad_segments.downlink = {rate(1, kDownlinkRate)};
  ServiceProfile empty_medium = reference_profile();
  empty_medium.medium = std::vector<ServiceSegment>{};
  ServiceProfile negative_delay = reference_profile();
  negative_delay.uplink_delay = ms(-1);
  const WorkerSpec first = one_worker().front();
  const WorkerSpec repeated_model{.id = WorkerId{2},
                                  .profiles = {inference(1, 5), inference(1, 6)}};
  const WorkerSpec idle_model{.id = WorkerId{3}, .profiles = {inference(1, 0)}};

  EXPECT_EQ(open_window(reference_profile(), one_worker(), at_ms(100), 0), std::nullopt);
  EXPECT_EQ(open_window(bad_segments, one_worker(), at_ms(100), kWindowSlots), std::nullopt);
  EXPECT_EQ(open_window(empty_medium, one_worker(), at_ms(100), kWindowSlots), std::nullopt);
  EXPECT_EQ(open_window(negative_delay, one_worker(), at_ms(100), kWindowSlots), std::nullopt);
  EXPECT_EQ(open_window(reference_profile(), {first, first}, at_ms(100), kWindowSlots),
            std::nullopt);
  EXPECT_EQ(open_window(reference_profile(), {repeated_model}, at_ms(100), kWindowSlots),
            std::nullopt);
  EXPECT_EQ(open_window(reference_profile(), {idle_model}, at_ms(100), kWindowSlots), std::nullopt);
  EXPECT_EQ(open_window(reference_profile(), one_worker(), t(kMaxNs - 5), kWindowSlots),
            std::nullopt);
}

TEST(ResourceWindow, CarriesTheProfileDispatchLeadAndRejectsANegativeOne) {
  ServiceProfile profile = reference_profile();
  profile.dispatch_lead = ms(4);
  ServiceProfile negative_lead = reference_profile();
  negative_lead.dispatch_lead = d(-1);

  const ResourceSnapshot window = opened(profile);
  const std::optional<ResourceSnapshot> advanced =
      advance_window(window, profile, at_ms(100), at_ms(103));

  EXPECT_EQ(window.dispatch_lead, ms(4));
  ASSERT_TRUE(advanced.has_value());
  EXPECT_EQ(advanced->dispatch_lead, ms(4));
  EXPECT_EQ(open_window(negative_lead, one_worker(), at_ms(100), kWindowSlots), std::nullopt);
}

TEST(ResourceWindow, AdvanceRefusesAWindowCarryingEndpointBudgets) {
  ResourceSnapshot window = opened(reference_profile());
  ASSERT_TRUE(advance_window(window, reference_profile(), at_ms(100), at_ms(103)).has_value());
  window.endpoints = {
      EndpointBudget{.robot = RobotId{0}, .uplink = window.uplink, .downlink = std::nullopt}};
  EXPECT_EQ(advance_window(window, reference_profile(), at_ms(100), at_ms(103)), std::nullopt);
}

TEST(ResourceWindow, AdvanceKeepsTheWindowInsideTheFirstSlotOrBeforeTheEpoch) {
  const ResourceSnapshot window = opened(reference_profile());

  EXPECT_EQ(advance_window(window, reference_profile(), at_ms(100), t((101 * kNsPerMs) - 1)),
            window);
  EXPECT_EQ(advance_window(window, reference_profile(), at_ms(100), at_ms(50)), window);
}

TEST(ResourceWindow, AdvanceCarriesReservationsAndExtendsWithAbsoluteProfileSlots) {
  ResourceSnapshot window = opened(reference_profile());
  window.uplink = window.uplink.reserve(slices_at({{2, 1'000}, {3, 2'000}}, at_ms(104))).value();
  window.downlink = window.downlink.reserve(slices_at({{4, 3'000}}, at_ms(105))).value();
  window.workers.front().lane = window.workers.front()
                                    .lane.reserve(window_ms(100, 102))
                                    .value()
                                    .reserve(window_ms(104, 108))
                                    .value();

  const std::optional<ResourceSnapshot> advanced =
      advance_window(window, reference_profile(), at_ms(100), t((102 * kNsPerMs) + 500'000));

  ASSERT_TRUE(advanced.has_value());
  EXPECT_TRUE(is_consistent(advanced.value()));
  EXPECT_EQ(advanced->uplink.slot_start(SlotIndex{0}), at_ms(102));
  EXPECT_EQ(advanced->uplink.slot_count(), kWindowSlots);
  EXPECT_EQ(reserved_bytes(advanced->uplink),
            expected_reserved({{0, 1'000}, {1, 2'000}}, kWindowSlots));
  EXPECT_EQ(reserved_bytes(advanced->downlink), expected_reserved({{2, 3'000}}, kWindowSlots));
  std::vector<std::int64_t> uplink_capacity = repeated(125'000, kWindowSlots);
  uplink_capacity.back() = 0;
  EXPECT_EQ(capacities_of(advanced->uplink), uplink_capacity);
  EXPECT_EQ(advanced->workers.front().lane.intervals(),
            (std::vector<Interval>{window_ms(104, 108)}));
  EXPECT_EQ(advanced->uplink_delay, ms(2));
  EXPECT_EQ(advanced->downlink_delay, ms(3));
}

TEST(ResourceWindow, AdvancePastTheHorizonOpensAFreshWindowAtTheCurrentSlot) {
  ResourceSnapshot window = opened(reference_profile());
  window.uplink = window.uplink.reserve(slices_at({{9, 1'000}}, at_ms(110))).value();
  window.workers.front().lane = window.workers.front().lane.reserve(window_ms(100, 108)).value();

  const std::optional<ResourceSnapshot> advanced =
      advance_window(window, reference_profile(), at_ms(100), t((125 * kNsPerMs) + 1));

  ASSERT_TRUE(advanced.has_value());
  EXPECT_EQ(advanced->uplink.slot_start(SlotIndex{0}), at_ms(125));
  EXPECT_EQ(reserved_bytes(advanced->uplink), expected_reserved({}, kWindowSlots));
  EXPECT_EQ(capacities_of(advanced->uplink), repeated(0, kWindowSlots));
  EXPECT_EQ(capacities_of(advanced->downlink), repeated(50'000, kWindowSlots));
  EXPECT_TRUE(advanced->workers.front().lane.intervals().empty());
}

TEST(ResourceWindow, AdvanceCarriesSharedMediumReservations) {
  ResourceSnapshot window = opened(profile_with_medium());
  window.shared_medium = window.shared_medium->reserve(slices_at({{5, 4'000}}, at_ms(106))).value();

  const std::optional<ResourceSnapshot> advanced =
      advance_window(window, profile_with_medium(), at_ms(100), at_ms(103));

  ASSERT_TRUE(advanced.has_value());
  ASSERT_TRUE(advanced->shared_medium.has_value());
  EXPECT_EQ(reserved_bytes(advanced->shared_medium.value()),
            expected_reserved({{2, 4'000}}, kWindowSlots));
  EXPECT_EQ(advanced->shared_medium->slot_start(SlotIndex{0}), at_ms(103));
}

TEST(ResourceWindow, AdvanceRejectsAProfileOrEpochThatDoesNotMatchTheWindow) {
  const ResourceSnapshot window = opened(reference_profile());
  ServiceProfile other_slot = reference_profile();
  other_slot.slot_length = ms(2);

  EXPECT_EQ(advance_window(window, other_slot, at_ms(100), at_ms(104)), std::nullopt);
  EXPECT_EQ(advance_window(window, profile_with_medium(), at_ms(100), at_ms(104)), std::nullopt);
  EXPECT_EQ(
      advance_window(opened(profile_with_medium()), reference_profile(), at_ms(100), at_ms(104)),
      std::nullopt);
  EXPECT_EQ(advance_window(window, reference_profile(), t((100 * kNsPerMs) + 1), at_ms(104)),
            std::nullopt);
  EXPECT_EQ(advance_window(window, reference_profile(), at_ms(200), at_ms(250)), std::nullopt);
}

TEST(ResourceWindow, AdvanceRejectsAnUnrepresentableExtension) {
  const ServiceProfile profile = reference_profile();
  const TimePoint epoch = t(kMaxNs - (20 * kNsPerMs) - 1);
  const ResourceSnapshot window = open_window(profile, one_worker(), epoch, kWindowSlots).value();

  EXPECT_EQ(advance_window(window, profile, epoch, t(epoch.ns() + (15 * kNsPerMs))), std::nullopt);
  EXPECT_EQ(advance_window(window, profile, epoch, t(kMaxNs)), std::nullopt);
}

TEST(ResourceWindow, AdvanceRejectsMalformedSegmentsForTheAppendedSlots) {
  const ResourceSnapshot window = opened(profile_with_medium());
  ServiceProfile bad_uplink = profile_with_medium();
  bad_uplink.uplink = {rate(1, kUplinkRate)};
  ServiceProfile bad_medium = profile_with_medium();
  bad_medium.medium = std::vector<ServiceSegment>{rate(2, kUplinkRate)};

  EXPECT_EQ(advance_window(window, bad_uplink, at_ms(100), at_ms(103)), std::nullopt);
  EXPECT_EQ(advance_window(window, bad_medium, at_ms(100), at_ms(103)), std::nullopt);
}

TEST(ResourceWindow, AdvanceRejectsAShiftWhosePathDelayLeavesTheTimeRange) {
  ServiceProfile profile = reference_profile();
  profile.downlink_delay = ms(10);
  const TimePoint epoch = t(kMaxNs - (20 * kNsPerMs) - 1);
  const ResourceSnapshot window = open_window(profile, one_worker(), epoch, kWindowSlots).value();

  EXPECT_EQ(advance_window(window, profile, epoch, t(epoch.ns() + (5 * kNsPerMs))), std::nullopt);
}

Job rolling_job(std::uint64_t id, TimePoint now) {
  return Job{.id = JobId{id},
             .release_time = now,
             .observation_time = now,
             .buffer_empty_time = checked_add(now, ms(60)).value(),
             .safety_margin = ms(5),
             .task_deadline = std::nullopt,
             .maximum_result_age = ms(50),
             .upload_bytes = Bytes{200'000},
             .action_chunk_bytes = Bytes{100'000},
             .model = ModelClass{1},
             .eligible_workers = {WorkerId{1}},
             .sequence = id};
}

TEST(ResourceWindow, RollingCyclesKeepEveryCommittedChainWithinCapacity) {
  constexpr std::int64_t kRollingSlots = 40;
  ServiceProfile steady = reference_profile();
  steady.uplink = {rate(0, kUplinkRate)};
  ResourceSnapshot window = opened(steady, kRollingSlots);
  int admitted_count = 0;
  int deferred_count = 0;

  for (std::uint64_t cycle = 0; cycle < 30; ++cycle) {
    const TimePoint now = at_ms(100 + static_cast<std::int64_t>(3 * cycle));
    window = advance_window(window, steady, at_ms(100), now).value();
    const std::vector<Job> pending{rolling_job(cycle + 1, now)};

    const ScheduleResult result = schedule(window, pending, {}, now, Policy::kLeastSlack);

    EXPECT_TRUE(audit_schedule(window, pending, {}, result).empty()) << "cycle " << cycle;
    const DecisionStatus status = result.decisions.front().decision.status;
    admitted_count += status == DecisionStatus::kAdmitted ? 1 : 0;
    deferred_count += status == DecisionStatus::kDeferred ? 1 : 0;
    window = result.snapshot;
  }

  EXPECT_GE(admitted_count, 5);
  EXPECT_GE(deferred_count, 5);
}

}  // namespace
}  // namespace rovi::core
