#include "rovi_network/logical_link.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "rovi_core/capacity_calendar.hpp"
#include "rovi_core/units.hpp"
#include "rovi_core/wireless_profile.hpp"

namespace rovi::network {
namespace {

using Link = LogicalLink<int>;

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kBytes = 100'000;
constexpr std::int64_t kTimeout = 1'000 * kMs;

core::TimePoint t(std::int64_t ns) { return core::TimePoint::from_ns(ns); }

core::Duration d(std::int64_t ns) { return core::Duration::from_ns(ns); }

LinkTiming timing(std::int64_t orphan_timeout_ns, std::int64_t max_lead_ns = 1'000 * kMs) {
  return LinkTiming{.orphan_timeout = d(orphan_timeout_ns), .max_lead = d(max_lead_ns)};
}

Authorization grant(std::int64_t bytes = kBytes) {
  return Authorization{
      .robot = core::RobotId{3}, .worker = core::WorkerId{1}, .bytes = core::Bytes{bytes}};
}

Lease lease_between(std::int64_t start, std::int64_t arrival) {
  return Lease{.service_start = t(start), .arrival = t(arrival), .authorized = grant()};
}

Lease lease_of(const core::CapacityCalendar& calendar, const core::Transfer& transfer) {
  return Lease{.service_start = calendar.slot_start(transfer.slices.front().slot).value(),
               .arrival = transfer.completion,
               .authorized = grant()};
}

void send(Link& link, std::uint64_t job, Direction direction, int payload, std::int64_t at) {
  link.send(core::JobId{job}, direction, grant(), payload, t(at));
}

std::vector<int> payloads(const Link::Events& events) {
  std::vector<int> result(events.deliveries.size());
  std::ranges::transform(events.deliveries, result.begin(),
                         [](const Link::Delivery& delivery) { return delivery.payload; });
  return result;
}

std::vector<Violation> reasons(const Link::Events& events) {
  std::vector<Violation> result(events.refusals.size());
  std::ranges::transform(events.refusals, result.begin(), &Refusal::reason);
  return result;
}

core::ServiceSegment rate_from(std::int64_t first_slot, std::int64_t bits_per_second) {
  return core::ServiceSegment{.first_slot = first_slot,
                              .rate = core::BitsPerSecond{bits_per_second},
                              .reduction = core::Permille{0}};
}

core::CapacityCalendar calendar_of(const std::vector<core::ServiceSegment>& segments) {
  return core::build_service_calendar(t(0), d(kMs), 40, segments).value();
}

std::int64_t residual_bytes(const core::CapacityCalendar& calendar, std::int64_t first,
                            std::int64_t end) {
  std::int64_t total = 0;
  for (std::int64_t slot = first; slot < end; ++slot) {
    total += calendar.capacity(core::SlotIndex{slot}).value() -
             calendar.reserved(core::SlotIndex{slot}).value();
  }
  return total;
}

TEST(LogicalLink, ReleaseKeepsTheReservedArrivalWhenSentByTheServiceStart) {
  EXPECT_EQ(release_time(lease_between(10, 30), t(5)), t(30));
  EXPECT_EQ(release_time(lease_between(10, 30), t(10)), t(30));
}

// A late payload gets no release: shifting it to sent + (arrival - service start), here 35, would
// use service the calendars never reserved.
TEST(LogicalLink, ReleaseRefusesALateSendInsteadOfShiftingItsService) {
  EXPECT_EQ(release_time(lease_between(10, 30), t(11)), std::nullopt);
  EXPECT_EQ(release_time(lease_between(10, 30), t(15)), std::nullopt);
  EXPECT_EQ(release_time(lease_between(10, 30), t(std::numeric_limits<std::int64_t>::max())),
            std::nullopt);
}

TEST(LogicalLink, ReleaseRejectsAnInvertedLease) {
  EXPECT_EQ(release_time(lease_between(30, 10), t(5)), std::nullopt);
}

TEST(LogicalLink, JoinsLeaseAndPayloadInEitherOrder) {
  Link link{timing(kTimeout)};

  EXPECT_TRUE(link.lease(core::JobId{1}, Direction::kUplink, lease_between(10, 30), t(0)));
  send(link, 1, Direction::kUplink, 11, 5);
  send(link, 2, Direction::kDownlink, 22, 6);
  EXPECT_TRUE(link.lease(core::JobId{2}, Direction::kDownlink, lease_between(10, 40), t(7)));

  EXPECT_TRUE(link.deliver(t(29)).deliveries.empty());
  const Link::Events first = link.deliver(t(30));
  ASSERT_EQ(first.deliveries.size(), 1U);
  EXPECT_EQ(first.deliveries.front().payload, 11);
  EXPECT_EQ(first.deliveries.front().direction, Direction::kUplink);
  EXPECT_EQ(first.deliveries.front().release, t(30));
  EXPECT_EQ(payloads(link.deliver(t(45))), (std::vector<int>{22}));
  const Link::Events rest = link.deliver(t(100));
  EXPECT_TRUE(rest.deliveries.empty());
  EXPECT_TRUE(rest.refusals.empty());
  EXPECT_EQ(link.unused_leases(), 0U);
}

TEST(LogicalLink, KeepsThePlannedReleaseWhenTheDeliveryPollRunsLate) {
  Link link{timing(kTimeout)};
  EXPECT_TRUE(link.lease(core::JobId{1}, Direction::kUplink, lease_between(10, 30), t(0)));
  send(link, 1, Direction::kUplink, 1, 5);

  const Link::Events events = link.deliver(t(45));

  ASSERT_EQ(events.deliveries.size(), 1U);
  EXPECT_EQ(events.deliveries.front().release, t(30));
}

TEST(LogicalLink, DeliversDuePayloadsInReleaseOrder) {
  Link link{timing(kTimeout)};
  EXPECT_TRUE(link.lease(core::JobId{1}, Direction::kUplink, lease_between(0, 50), t(0)));
  EXPECT_TRUE(link.lease(core::JobId{2}, Direction::kUplink, lease_between(0, 20), t(0)));
  EXPECT_TRUE(link.lease(core::JobId{3}, Direction::kUplink, lease_between(0, 20), t(0)));

  send(link, 1, Direction::kUplink, 1, 0);
  send(link, 3, Direction::kUplink, 3, 0);
  send(link, 2, Direction::kUplink, 2, 0);

  EXPECT_EQ(payloads(link.deliver(t(60))), (std::vector<int>{3, 2, 1}));
}

TEST(LogicalLink, KeepsDirectionsOfOneJobIndependent) {
  Link link{timing(kTimeout)};
  EXPECT_TRUE(link.lease(core::JobId{1}, Direction::kUplink, lease_between(0, 10), t(0)));
  EXPECT_TRUE(link.lease(core::JobId{1}, Direction::kDownlink, lease_between(20, 30), t(0)));

  send(link, 1, Direction::kDownlink, 2, 20);

  EXPECT_EQ(payloads(link.deliver(t(29))), (std::vector<int>{}));
  EXPECT_EQ(payloads(link.deliver(t(30))), (std::vector<int>{2}));
}

TEST(LogicalLink, HoldsAnEarlyPayloadAndRefusesOneReadyAfterItsServiceStart) {
  Link link{timing(kTimeout)};
  for (const std::uint64_t job : {1U, 2U, 3U}) {
    EXPECT_TRUE(
        link.lease(core::JobId{job}, Direction::kUplink, lease_between(10 * kMs, 20 * kMs), t(0)));
  }

  send(link, 1, Direction::kUplink, 1, 9 * kMs);
  send(link, 2, Direction::kUplink, 2, 10 * kMs);
  send(link, 3, Direction::kUplink, 3, (10 * kMs) + 1);

  const Link::Events early = link.deliver(t((20 * kMs) - 1));
  EXPECT_TRUE(early.deliveries.empty());
  ASSERT_EQ(early.refusals.size(), 1U);
  EXPECT_EQ(early.refusals.front(), (Refusal{.job = core::JobId{3},
                                             .direction = Direction::kUplink,
                                             .reason = Violation::kLateSend,
                                             .claimed = grant(),
                                             .sent = t((10 * kMs) + 1),
                                             .detected = t((10 * kMs) + 1),
                                             .lease = lease_between(10 * kMs, 20 * kMs)}));
  EXPECT_EQ(payloads(link.deliver(t(20 * kMs))), (std::vector<int>{1, 2}));
  EXPECT_TRUE(link.deliver(t(100 * kMs)).deliveries.empty());
}

// Shifted to [5, 15) ms, A would need 200,000 bytes from [5, 20) ms, which holds
// 150,000, and its last 5 ms are already reserved for B.
TEST(LogicalLink, ALateUploadCannotTakeServiceReservedForTheNextLease) {
  const core::CapacityCalendar empty = calendar_of({rate_from(0, 80'000'000)});
  const core::Transfer upload_a = empty.plan_transfer(t(0), core::Bytes{kBytes}).value();
  const core::CapacityCalendar with_a = empty.reserve(upload_a).value();
  const core::Transfer upload_b = with_a.plan_transfer(t(10 * kMs), core::Bytes{kBytes}).value();
  const core::CapacityCalendar with_both = with_a.reserve(upload_b).value();
  ASSERT_EQ(upload_a.completion, t(10 * kMs));
  ASSERT_EQ(upload_b.completion, t(20 * kMs));
  EXPECT_EQ(residual_bytes(empty, 5, 20), 150'000);
  EXPECT_EQ(residual_bytes(with_both, 10, 20), 0);

  Link link{timing(kTimeout)};
  EXPECT_TRUE(link.lease(core::JobId{1}, Direction::kUplink, lease_of(empty, upload_a), t(0)));
  EXPECT_TRUE(link.lease(core::JobId{2}, Direction::kUplink, lease_of(with_a, upload_b), t(0)));
  send(link, 1, Direction::kUplink, 1, 5 * kMs);
  send(link, 2, Direction::kUplink, 2, 10 * kMs);

  const Link::Events shifted = link.deliver(t(15 * kMs));
  EXPECT_TRUE(shifted.deliveries.empty());
  EXPECT_EQ(reasons(shifted), (std::vector<Violation>{Violation::kLateSend}));
  EXPECT_EQ(shifted.refusals.front().job, core::JobId{1});
  EXPECT_EQ(payloads(link.deliver(t(20 * kMs))), (std::vector<int>{2}));
  EXPECT_TRUE(link.deliver(t(100 * kMs)).deliveries.empty());
}

// Sent at 5 ms, the payload cannot complete before 25 ms through the outage.
TEST(LogicalLink, ALateUploadIsNeverDeliveredAcrossAnOutage) {
  const core::CapacityCalendar calendar =
      calendar_of({rate_from(0, 80'000'000), rate_from(10, 0), rate_from(20, 80'000'000)});
  const core::Transfer reserved = calendar.plan_transfer(t(0), core::Bytes{kBytes}).value();
  EXPECT_EQ(calendar.plan_transfer(t(5 * kMs), core::Bytes{kBytes})->completion, t(25 * kMs));

  Link link{timing(kTimeout)};
  EXPECT_TRUE(link.lease(core::JobId{1}, Direction::kUplink, lease_of(calendar, reserved), t(0)));
  send(link, 1, Direction::kUplink, 1, 5 * kMs);

  const Link::Events events = link.deliver(t(15 * kMs));
  EXPECT_TRUE(events.deliveries.empty());
  EXPECT_EQ(reasons(events), (std::vector<Violation>{Violation::kLateSend}));
  EXPECT_TRUE(link.deliver(t(100 * kMs)).deliveries.empty());
}

TEST(LogicalLink, AChunkFinishedAfterItsDownlinkStartIsRefusedNotShifted) {
  Link link{timing(kTimeout)};
  EXPECT_TRUE(
      link.lease(core::JobId{1}, Direction::kDownlink, lease_between(60 * kMs, 66 * kMs), t(0)));

  send(link, 1, Direction::kDownlink, 1, 61 * kMs);

  const Link::Events events = link.deliver(t(67 * kMs));
  EXPECT_TRUE(events.deliveries.empty());
  ASSERT_EQ(events.refusals.size(), 1U);
  EXPECT_EQ(events.refusals.front().direction, Direction::kDownlink);
  EXPECT_EQ(events.refusals.front().reason, Violation::kLateSend);
  EXPECT_TRUE(link.deliver(t(1'000 * kMs)).deliveries.empty());
}

TEST(LogicalLink, RefusesAPayloadWhoseIdentityOrSizeDiffersFromItsLease) {
  Link link{timing(kTimeout)};
  Authorization other_robot = grant();
  other_robot.robot = core::RobotId{4};
  Authorization other_worker = grant();
  other_worker.worker = core::WorkerId{2};
  const std::vector<Authorization> claims{other_robot, other_worker, grant(kBytes - 1),
                                          grant(kBytes + 1)};
  for (std::uint64_t job = 1; job <= claims.size(); ++job) {
    EXPECT_TRUE(link.lease(core::JobId{job}, Direction::kUplink, lease_between(10, 30), t(0)));
    link.send(core::JobId{job}, Direction::kUplink, claims.at(job - 1), static_cast<int>(job),
              t(5));
  }

  const Link::Events events = link.deliver(t(1'000));

  EXPECT_TRUE(events.deliveries.empty());
  EXPECT_EQ(reasons(events),
            (std::vector<Violation>{Violation::kIdentityMismatch, Violation::kIdentityMismatch,
                                    Violation::kSizeMismatch, Violation::kSizeMismatch}));
  EXPECT_EQ(events.refusals.back().claimed, grant(kBytes + 1));
  EXPECT_EQ(events.refusals.back().lease, lease_between(10, 30));
}

TEST(LogicalLink, AnInvalidPayloadLeavesTheLeaseForTheValidOne) {
  Link link{timing(kTimeout)};
  Authorization other_robot = grant();
  other_robot.robot = core::RobotId{4};
  EXPECT_TRUE(link.lease(core::JobId{1}, Direction::kUplink, lease_between(10, 30), t(0)));

  link.send(core::JobId{1}, Direction::kUplink, other_robot, 1, t(2));
  link.send(core::JobId{1}, Direction::kUplink, grant(kBytes - 1), 2, t(3));
  send(link, 1, Direction::kUplink, 3, 5);

  const Link::Events events = link.deliver(t(30));
  EXPECT_EQ(payloads(events), (std::vector<int>{3}));
  EXPECT_EQ(reasons(events),
            (std::vector<Violation>{Violation::kIdentityMismatch, Violation::kSizeMismatch}));
}

TEST(LogicalLink, AnInvalidPayloadWaitingForItsLeaseDoesNotBlockTheValidOne) {
  Link link{timing(kTimeout)};
  Authorization other_robot = grant();
  other_robot.robot = core::RobotId{4};
  link.send(core::JobId{1}, Direction::kUplink, other_robot, 1, t(0));
  send(link, 1, Direction::kUplink, 2, 1);
  link.send(core::JobId{1}, Direction::kUplink, grant(kBytes - 1), 3, t(1));
  send(link, 1, Direction::kUplink, 4, 1);

  EXPECT_TRUE(link.lease(core::JobId{1}, Direction::kUplink, lease_between(10, 30), t(2)));

  const Link::Events events = link.deliver(t(30));
  EXPECT_EQ(payloads(events), (std::vector<int>{2}));
  EXPECT_EQ(reasons(events),
            (std::vector<Violation>{Violation::kIdentityMismatch, Violation::kSizeMismatch,
                                    Violation::kDuplicate}));
  EXPECT_EQ(link.unused_leases(), 0U);
}

TEST(LogicalLink, AValidLatePayloadConsumesItsLease) {
  Link link{timing(kTimeout)};
  EXPECT_TRUE(link.lease(core::JobId{1}, Direction::kUplink, lease_between(10, 30), t(0)));

  send(link, 1, Direction::kUplink, 1, 11);
  send(link, 1, Direction::kUplink, 2, 12);

  const Link::Events events = link.deliver(t(30));
  EXPECT_TRUE(events.deliveries.empty());
  EXPECT_EQ(reasons(events), (std::vector<Violation>{Violation::kLateSend, Violation::kDuplicate}));
}

TEST(LogicalLink, DeliversOnceAndRefusesARepeatedPayloadOrLease) {
  Link link{timing(kTimeout)};
  EXPECT_TRUE(link.lease(core::JobId{1}, Direction::kUplink, lease_between(10, 30), t(0)));
  send(link, 1, Direction::kUplink, 1, 5);
  EXPECT_EQ(payloads(link.deliver(t(30))), (std::vector<int>{1}));

  send(link, 1, Direction::kUplink, 2, 31);
  EXPECT_FALSE(link.lease(core::JobId{1}, Direction::kUplink, lease_between(40, 60), t(31)));
  send(link, 1, Direction::kUplink, 3, 32);

  const Link::Events repeated = link.deliver(t(100));
  EXPECT_TRUE(repeated.deliveries.empty());
  EXPECT_EQ(reasons(repeated),
            (std::vector<Violation>{Violation::kDuplicate, Violation::kDuplicate}));
}

TEST(LogicalLink, AReplayAfterTheSettledMemoryExpiresStillCannotDeliverAgain) {
  Link link{timing(100)};
  EXPECT_TRUE(link.lease(core::JobId{1}, Direction::kUplink, lease_between(10, 30), t(0)));
  send(link, 1, Direction::kUplink, 1, 5);
  EXPECT_EQ(payloads(link.deliver(t(30))), (std::vector<int>{1}));
  EXPECT_TRUE(link.deliver(t(131)).refusals.empty());

  EXPECT_TRUE(link.lease(core::JobId{1}, Direction::kUplink, lease_between(10, 30), t(131)));
  send(link, 1, Direction::kUplink, 2, 132);

  const Link::Events replayed = link.deliver(t(200));
  EXPECT_TRUE(replayed.deliveries.empty());
  EXPECT_EQ(reasons(replayed), (std::vector<Violation>{Violation::kLateSend}));
}

TEST(LogicalLink, KeepsTheFirstOfTwoPayloadsWaitingForOneLease) {
  Link link{timing(kTimeout)};
  send(link, 1, Direction::kUplink, 1, 0);
  send(link, 1, Direction::kUplink, 2, 1);
  EXPECT_TRUE(link.lease(core::JobId{1}, Direction::kUplink, lease_between(10, 30), t(2)));

  const Link::Events events = link.deliver(t(30));

  EXPECT_EQ(payloads(events), (std::vector<int>{1}));
  EXPECT_EQ(reasons(events), (std::vector<Violation>{Violation::kDuplicate}));
}

TEST(LogicalLink, RejectsALeaseArrivingBeyondTheMaximumLead) {
  Link link{timing(kTimeout, 50)};

  EXPECT_TRUE(link.lease(core::JobId{1}, Direction::kUplink, lease_between(0, 150), t(100)));
  EXPECT_FALSE(link.lease(core::JobId{2}, Direction::kUplink, lease_between(0, 151), t(100)));
  EXPECT_TRUE(link.lease(core::JobId{3}, Direction::kUplink,
                         lease_between(0, std::numeric_limits<std::int64_t>::max()),
                         t(std::numeric_limits<std::int64_t>::max() - 10)));
}

TEST(LogicalLink, RejectsADuplicateInvertedOrEmptyLease) {
  Link link{timing(kTimeout)};
  Lease empty = lease_between(0, 10);
  empty.authorized.bytes = core::Bytes{0};

  EXPECT_TRUE(link.lease(core::JobId{1}, Direction::kUplink, lease_between(0, 10), t(0)));
  EXPECT_FALSE(link.lease(core::JobId{1}, Direction::kUplink, lease_between(0, 10), t(0)));
  EXPECT_FALSE(link.lease(core::JobId{2}, Direction::kUplink, lease_between(10, 0), t(0)));
  EXPECT_FALSE(link.lease(core::JobId{3}, Direction::kUplink, empty, t(0)));
}

TEST(LogicalLink, RefusesAPayloadForAnUnknownRequestOrDirectionAfterTheOrphanTimeout) {
  Link link{timing(100)};
  EXPECT_TRUE(link.lease(core::JobId{1}, Direction::kUplink, lease_between(0, 10), t(0)));
  send(link, 2, Direction::kUplink, 2, 0);
  send(link, 1, Direction::kDownlink, 1, 0);

  EXPECT_TRUE(link.deliver(t(100)).refusals.empty());
  const Link::Events expired = link.deliver(t(111));

  EXPECT_TRUE(expired.deliveries.empty());
  EXPECT_EQ(reasons(expired), (std::vector<Violation>{Violation::kUnleased, Violation::kUnleased}));
  EXPECT_EQ(expired.refusals.front().detected, t(111));
  EXPECT_EQ(expired.refusals.front().lease, std::nullopt);
  EXPECT_EQ(link.unused_leases(), 1U);
}

TEST(LogicalLink, PrunesLeasesAndPayloadsThatOutliveTheOrphanTimeout) {
  Link link{timing(100)};
  EXPECT_TRUE(link.lease(core::JobId{1}, Direction::kUplink, lease_between(0, 10), t(0)));
  send(link, 2, Direction::kDownlink, 2, 20);

  EXPECT_TRUE(link.deliver(t(110)).refusals.empty());
  EXPECT_EQ(link.unused_leases(), 0U);
  EXPECT_EQ(reasons(link.deliver(t(121))), (std::vector<Violation>{Violation::kUnleased}));
  EXPECT_EQ(link.unused_leases(), 1U);

  send(link, 1, Direction::kUplink, 1, 122);
  EXPECT_TRUE(link.lease(core::JobId{2}, Direction::kDownlink, lease_between(0, 10), t(0)));
  EXPECT_TRUE(link.deliver(t(200)).deliveries.empty());
}

TEST(LogicalLink, NeverPrunesWhenTheTimeoutCannotBeRepresented) {
  Link link{timing(std::numeric_limits<std::int64_t>::max())};
  EXPECT_TRUE(link.lease(core::JobId{1}, Direction::kUplink, lease_between(0, 10), t(0)));

  const Link::Events events = link.deliver(t(std::numeric_limits<std::int64_t>::max()));
  EXPECT_TRUE(events.deliveries.empty());
  EXPECT_TRUE(events.refusals.empty());
  EXPECT_EQ(link.unused_leases(), 0U);
}

}  // namespace
}  // namespace rovi::network
