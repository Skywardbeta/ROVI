#include "rovi_scheduler/job_book.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "rovi_core/admission.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/scheduler.hpp"
#include "rovi_core/units.hpp"

namespace rovi::scheduler {
namespace {

constexpr std::int64_t kNsPerMs = 1'000'000;

core::TimePoint at_ms(std::int64_t count) { return core::TimePoint::from_ns(count * kNsPerMs); }

core::Duration ms(std::int64_t count) { return core::Duration::from_ns(count * kNsPerMs); }

core::Job job(std::uint64_t id, core::TimePoint release) {
  return core::Job{.id = core::JobId{id},
                   .release_time = release,
                   .observation_time = release,
                   .buffer_empty_time = release,
                   .safety_margin = ms(0),
                   .task_deadline = std::nullopt,
                   .maximum_result_age = std::nullopt,
                   .upload_bytes = core::Bytes{1},
                   .action_chunk_bytes = core::Bytes{1},
                   .model = core::ModelClass{1},
                   .eligible_workers = {core::WorkerId{1}},
                   .sequence = id};
}

core::JobDecision decision(std::uint64_t id, core::DecisionStatus status, core::ReasonCode reason) {
  return core::JobDecision{.job = core::JobId{id},
                           .decision = core::Decision{.status = status, .reason = reason},
                           .reservation = std::nullopt};
}

core::JobDecision admitted(std::uint64_t id) {
  return decision(id, core::DecisionStatus::kAdmitted, core::ReasonCode::kNone);
}

core::JobDecision deferred(std::uint64_t id, core::ReasonCode reason) {
  return decision(id, core::DecisionStatus::kDeferred, reason);
}

Notice notice(const core::Job& job, const core::JobDecision& decision) {
  return Notice{.job = job, .decision = decision};
}

std::vector<std::uint64_t> ids(const std::vector<core::Job>& jobs) {
  std::vector<std::uint64_t> result(jobs.size());
  std::ranges::transform(jobs, result.begin(),
                         [](const core::Job& pending) { return pending.id.value(); });
  return result;
}

JobBook book_with(std::size_t max_pending) {
  return JobBook{BookLimits{.release_tolerance = ms(50), .max_pending = max_pending}};
}

core::JobDecision rejection(std::uint64_t id, core::ReasonCode reason) {
  return decision(id, core::DecisionStatus::kRejected, reason);
}

core::Job deadline_job(std::uint64_t id, std::int64_t buffer_ms,
                       std::optional<std::int64_t> age_ms) {
  core::Job deferred_job = job(id, at_ms(0));
  deferred_job.buffer_empty_time = at_ms(buffer_ms);
  deferred_job.maximum_result_age =
      age_ms ? std::optional{ms(*age_ms)} : std::optional<core::Duration>{};
  return deferred_job;
}

core::Job member(std::uint64_t id, std::uint32_t group, core::TimePoint release,
                 core::TimePoint buffer_empty) {
  core::Job grouped = job(id, release);
  grouped.buffer_empty_time = buffer_empty;
  grouped.synchronization_group = core::GroupId{group};
  grouped.maximum_result_ready_skew = ms(50);
  return grouped;
}

TEST(JobBook, HoldsAJobUntilItsReleaseTimeHasPassedOnTheSchedulerClock) {
  JobBook book = book_with(4);

  EXPECT_EQ(book.submit(job(1, at_ms(100)), at_ms(90)), std::nullopt);

  EXPECT_TRUE(book.released(at_ms(99)).empty());
  EXPECT_EQ(ids(book.released(at_ms(100))), (std::vector<std::uint64_t>{1}));
  EXPECT_EQ(book.pending_count(), 1U);
}

TEST(JobBook, RejectsAJobReleasedBeyondTheClockSkewTolerance) {
  JobBook book = book_with(4);

  EXPECT_EQ(book.submit(job(1, at_ms(151)), at_ms(100)),
            decision(1, core::DecisionStatus::kRejected, core::ReasonCode::kInvalidTiming));
  EXPECT_EQ(book.pending_count(), 0U);
  EXPECT_EQ(book.submit(job(1, at_ms(100)), at_ms(100)),
            decision(1, core::DecisionStatus::kRejected, core::ReasonCode::kDuplicateJob));
}

TEST(JobBook, AcceptsAnyReleaseWhenTheToleranceLimitIsUnrepresentable) {
  JobBook book = book_with(4);
  const core::TimePoint late = core::TimePoint::from_ns(std::numeric_limits<std::int64_t>::max());

  EXPECT_EQ(book.submit(job(1, late), core::TimePoint::from_ns(late.ns() - 10)), std::nullopt);
  EXPECT_EQ(book.pending_count(), 1U);
}

TEST(JobBook, RejectsAnIdThatIsPendingOrAlreadyDecided) {
  JobBook book = book_with(4);
  const core::JobDecision duplicate =
      decision(1, core::DecisionStatus::kRejected, core::ReasonCode::kDuplicateJob);

  EXPECT_EQ(book.submit(job(1, at_ms(0)), at_ms(0)), std::nullopt);
  EXPECT_EQ(book.submit(job(1, at_ms(0)), at_ms(0)), duplicate);
  EXPECT_EQ(book.pending_count(), 1U);
  EXPECT_EQ(book.settle({admitted(1)}),
            (std::vector<Notice>{notice(job(1, at_ms(0)), admitted(1))}));
  EXPECT_EQ(book.submit(job(1, at_ms(0)), at_ms(0)), duplicate);
}

TEST(JobBook, SettleRemovesFinalDecisionsAndReportsEachDeferralReasonOnce) {
  JobBook book = book_with(4);
  for (std::uint64_t id = 1; id <= 3; ++id) {
    EXPECT_EQ(book.submit(job(id, at_ms(0)), at_ms(0)), std::nullopt);
  }
  const core::JobDecision waiting = deferred(1, core::ReasonCode::kPredecessorIncomplete);
  const core::JobDecision rejected =
      decision(3, core::DecisionStatus::kRejected, core::ReasonCode::kDeadlineMiss);

  EXPECT_EQ(
      book.settle({waiting, admitted(2), rejected}),
      (std::vector<Notice>{notice(job(1, at_ms(0)), waiting), notice(job(2, at_ms(0)), admitted(2)),
                           notice(job(3, at_ms(0)), rejected)}));
  EXPECT_EQ(ids(book.released(at_ms(0))), (std::vector<std::uint64_t>{1}));
  EXPECT_TRUE(book.settle({waiting}).empty());

  const core::JobDecision unplaceable = deferred(1, core::ReasonCode::kNoFeasibleChain);
  EXPECT_EQ(book.settle({unplaceable}),
            (std::vector<Notice>{notice(job(1, at_ms(0)), unplaceable)}));
  EXPECT_EQ(book.settle({admitted(1)}),
            (std::vector<Notice>{notice(job(1, at_ms(0)), admitted(1))}));
  EXPECT_TRUE(book.released(at_ms(0)).empty());
}

TEST(JobBook, IgnoresDecisionsForUnknownJobs) {
  JobBook book = book_with(4);

  EXPECT_TRUE(book.settle({admitted(9)}).empty());
}

TEST(JobBook, RejectsRequestsBeyondThePendingCapacityUntilOneIsDecided) {
  JobBook book = book_with(2);
  EXPECT_EQ(book.submit(job(1, at_ms(0)), at_ms(0)), std::nullopt);
  EXPECT_EQ(book.submit(job(2, at_ms(0)), at_ms(0)), std::nullopt);

  EXPECT_EQ(book.submit(job(3, at_ms(0)), at_ms(0)), rejection(3, core::ReasonCode::kQueueFull));
  EXPECT_EQ(book.settle({admitted(1)}).size(), 1U);
  EXPECT_EQ(book.submit(job(4, at_ms(0)), at_ms(0)), std::nullopt);
  EXPECT_EQ(book.pending_count(), 2U);
}

TEST(JobBook, RejectsOverlongWorkerOrPredecessorLists) {
  JobBook book = book_with(4);
  core::Job many_workers = job(1, at_ms(0));
  many_workers.eligible_workers.assign(kMaximumRequestListLength + 1, core::WorkerId{1});
  core::Job many_predecessors = job(2, at_ms(0));
  many_predecessors.predecessors.assign(kMaximumRequestListLength + 1, core::JobId{9});
  core::Job longest_allowed = job(3, at_ms(0));
  longest_allowed.predecessors.assign(kMaximumRequestListLength, core::JobId{9});

  EXPECT_EQ(book.submit(many_workers, at_ms(0)), rejection(1, core::ReasonCode::kMalformedRequest));
  EXPECT_EQ(book.submit(many_predecessors, at_ms(0)),
            rejection(2, core::ReasonCode::kMalformedRequest));
  EXPECT_EQ(book.submit(longest_allowed, at_ms(0)), std::nullopt);
}

TEST(JobBook, ExpiresDeferredJobsOnceTheirEffectiveDeadlinePasses) {
  JobBook book = book_with(4);
  const core::Job buffer_bound = deadline_job(1, 100, std::nullopt);
  const core::Job freshness_bound = deadline_job(2, 500, 80);
  const core::Job not_yet_deferred = deadline_job(3, 50, std::nullopt);
  for (const core::Job& pending : {buffer_bound, freshness_bound, not_yet_deferred}) {
    ASSERT_EQ(book.submit(pending, at_ms(0)), std::nullopt);
  }
  ASSERT_EQ(book.settle({deferred(1, core::ReasonCode::kUplinkServiceUnavailable),
                         deferred(2, core::ReasonCode::kNoFeasibleChain)})
                .size(),
            2U);

  EXPECT_TRUE(book.expire(at_ms(79)).empty());
  EXPECT_EQ(
      book.expire(at_ms(80)),
      (std::vector<Notice>{notice(freshness_bound, rejection(2, core::ReasonCode::kStaleResult))}));
  EXPECT_EQ(
      book.expire(at_ms(100)),
      (std::vector<Notice>{notice(buffer_bound, rejection(1, core::ReasonCode::kDeadlineMiss))}));
  EXPECT_EQ(ids(book.released(at_ms(100))), (std::vector<std::uint64_t>{3}));
  EXPECT_EQ(book.submit(buffer_bound, at_ms(100)), rejection(1, core::ReasonCode::kDuplicateJob));
}

TEST(JobBook, ExpiresADeferredJobWhoseDeadlineCannotBeRepresented) {
  JobBook book = book_with(4);
  core::Job overflowing = job(1, at_ms(0));
  overflowing.safety_margin = core::Duration::from_ns(std::numeric_limits<std::int64_t>::min());
  ASSERT_EQ(book.submit(overflowing, at_ms(0)), std::nullopt);
  ASSERT_EQ(book.settle({deferred(1, core::ReasonCode::kNoFeasibleChain)}).size(), 1U);

  EXPECT_EQ(book.expire(at_ms(0)),
            (std::vector<Notice>{
                notice(overflowing, rejection(1, core::ReasonCode::kArithmeticOverflow))}));
}

TEST(JobBook, AcceptsCompletionsOnlyForAdmittedJobs) {
  JobBook book = book_with(4);
  ASSERT_EQ(book.submit(job(1, at_ms(0)), at_ms(0)), std::nullopt);

  EXPECT_FALSE(book.complete(core::JobId{1}));
  EXPECT_FALSE(book.complete(core::JobId{7}));
  ASSERT_EQ(book.settle({admitted(1)}).size(), 1U);
  EXPECT_TRUE(book.complete(core::JobId{1}));
  EXPECT_FALSE(book.complete(core::JobId{1}));
}

TEST(JobBook, ReportsOnlyTheCompletionsThatTheGivenJobsDependOn) {
  JobBook book = book_with(8);
  for (std::uint64_t id = 1; id <= 3; ++id) {
    ASSERT_EQ(book.submit(job(id, at_ms(0)), at_ms(0)), std::nullopt);
  }
  ASSERT_EQ(book.settle({admitted(1), admitted(2), admitted(3)}).size(), 3U);
  for (std::uint64_t id = 3; id >= 1; --id) {
    ASSERT_TRUE(book.complete(core::JobId{id}));
  }
  core::Job dependent = job(10, at_ms(0));
  dependent.predecessors = {core::JobId{3}, core::JobId{9}, core::JobId{1}};
  core::Job also_dependent = job(11, at_ms(0));
  also_dependent.predecessors = {core::JobId{1}};

  EXPECT_EQ(book.completed_for({dependent, also_dependent}),
            (std::vector<core::JobId>{core::JobId{1}, core::JobId{3}}));
  EXPECT_TRUE(book.completed_for({job(12, at_ms(0))}).empty());
}

TEST(JobBook, HoldsAGroupBackFromTheCoreUntilEveryMemberIsPending) {
  JobBook book = book_with(8);
  ASSERT_FALSE(book.submit(member(1, 7, at_ms(0), at_ms(500)), at_ms(0), 3));
  ASSERT_FALSE(book.submit(member(2, 7, at_ms(0), at_ms(500)), at_ms(0), 3));
  ASSERT_FALSE(book.submit(job(9, at_ms(0)), at_ms(0)));

  EXPECT_EQ(ids(book.released(at_ms(1))), (std::vector<std::uint64_t>{9}));

  ASSERT_FALSE(book.submit(member(3, 7, at_ms(0), at_ms(500)), at_ms(0), 3));
  EXPECT_EQ(ids(book.released(at_ms(1))), (std::vector<std::uint64_t>{1, 2, 9, 3}));
}

TEST(JobBook, ReleasesAGroupOnlyOnceEveryMemberIsReleased) {
  JobBook book = book_with(8);
  ASSERT_FALSE(book.submit(member(1, 7, at_ms(0), at_ms(500)), at_ms(0), 2));
  ASSERT_FALSE(book.submit(member(2, 7, at_ms(10), at_ms(500)), at_ms(0), 2));

  EXPECT_TRUE(book.released(at_ms(5)).empty());
  EXPECT_EQ(ids(book.released(at_ms(10))), (std::vector<std::uint64_t>{1, 2}));
}

TEST(JobBook, RejectsMalformedGroupMembership) {
  JobBook book = book_with(8);
  const auto malformed = [](std::uint64_t id) {
    return rejection(id, core::ReasonCode::kMalformedRequest);
  };

  EXPECT_EQ(book.submit(member(1, 7, at_ms(0), at_ms(500)), at_ms(0), 0), malformed(1));
  EXPECT_EQ(book.submit(job(2, at_ms(0)), at_ms(0), 2), malformed(2));
  EXPECT_EQ(book.submit(member(3, 7, at_ms(0), at_ms(500)), at_ms(0),
                        static_cast<std::uint32_t>(kMaximumRequestListLength + 1)),
            malformed(3));
  ASSERT_FALSE(book.submit(member(4, 7, at_ms(0), at_ms(500)), at_ms(0), 3));
  EXPECT_EQ(book.submit(member(5, 7, at_ms(0), at_ms(500)), at_ms(0), 2), malformed(5));
  ASSERT_FALSE(book.submit(member(6, 8, at_ms(0), at_ms(500)), at_ms(0), 1));
  EXPECT_EQ(book.submit(member(7, 8, at_ms(0), at_ms(500)), at_ms(0), 1), malformed(7));
  EXPECT_EQ(book.pending_count(), 2U);
}

TEST(JobBook, RefusesMembersOfAGroupThatWasAlreadyDecided) {
  JobBook book = book_with(8);
  ASSERT_FALSE(book.submit(member(1, 7, at_ms(0), at_ms(500)), at_ms(0), 1));
  static_cast<void>(book.settle({admitted(1)}));

  EXPECT_EQ(book.submit(member(2, 7, at_ms(0), at_ms(500)), at_ms(0), 1),
            rejection(2, core::ReasonCode::kGroupMemberUnavailable));
}

TEST(JobBook, ExpiresAnIncompleteGroupTogetherOnceAnyMemberDeadlinePasses) {
  JobBook book = book_with(8);
  const core::Job first = member(1, 7, at_ms(0), at_ms(20));
  const core::Job second = member(2, 7, at_ms(0), at_ms(40));
  ASSERT_FALSE(book.submit(first, at_ms(0), 3));
  ASSERT_FALSE(book.submit(second, at_ms(0), 3));

  EXPECT_TRUE(book.expire(at_ms(10)).empty());
  const std::vector<Notice> expected{
      notice(first, rejection(1, core::ReasonCode::kGroupMemberUnavailable)),
      notice(second, rejection(2, core::ReasonCode::kGroupMemberUnavailable))};
  EXPECT_EQ(book.expire(at_ms(25)), expected);
  EXPECT_EQ(book.pending_count(), 0U);
  EXPECT_EQ(book.submit(member(3, 7, at_ms(25), at_ms(500)), at_ms(25), 3),
            rejection(3, core::ReasonCode::kGroupMemberUnavailable));
}

TEST(JobBook, ExpiresEveryPendingMemberWhenOneMemberOfADeferredGroupExpires) {
  JobBook book = book_with(8);
  const core::Job first = member(1, 7, at_ms(0), at_ms(20));
  const core::Job second = member(2, 7, at_ms(0), at_ms(100));
  ASSERT_FALSE(book.submit(first, at_ms(0), 2));
  ASSERT_FALSE(book.submit(second, at_ms(0), 2));
  static_cast<void>(book.settle({deferred(1, core::ReasonCode::kUplinkServiceUnavailable),
                                 deferred(2, core::ReasonCode::kGroupMemberUnavailable)}));

  const std::vector<Notice> expected{
      notice(first, rejection(1, core::ReasonCode::kDeadlineMiss)),
      notice(second, rejection(2, core::ReasonCode::kGroupMemberUnavailable))};
  EXPECT_EQ(book.expire(at_ms(25)), expected);
  EXPECT_EQ(book.pending_count(), 0U);
}

TEST(JobBook, RemembersOnlyTheMostRecentClosedGroups) {
  JobBook book = book_with(8);
  const auto close = [&book](std::uint64_t id, std::uint32_t group) {
    ASSERT_FALSE(book.submit(member(id, group, at_ms(0), at_ms(500)), at_ms(0), 1));
    static_cast<void>(book.settle({admitted(id)}));
  };
  for (std::uint32_t group = 1; group <= kClosedGroupMemory + 1; ++group) {
    close(group, group);
  }

  EXPECT_FALSE(book.submit(member(100'000, 1, at_ms(0), at_ms(500)), at_ms(0), 1));
  EXPECT_EQ(book.submit(member(100'001, 2, at_ms(0), at_ms(500)), at_ms(0), 1),
            rejection(100'001, core::ReasonCode::kGroupMemberUnavailable));
}

}  // namespace
}  // namespace rovi::scheduler
