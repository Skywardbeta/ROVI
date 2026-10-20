#include "rovi_core/receipt.hpp"

#include <cstdint>
#include <optional>
#include <utility>

#include <gtest/gtest.h>

#include "rovi_core/units.hpp"

#include "test_support.hpp"

namespace rovi::core {
namespace {

// A receipt at the deadline is on time, and absence is only established
// once the horizon is past the deadline.
TEST(ReceiptOutcome, IsOnTimeAtTheDeadlineAndMissingOnlyAfterIt) {
  EXPECT_EQ(classify_receipt(at_ms(100), at_ms(100), at_ms(100)), ReceiptOutcome::kOnTime);
  EXPECT_EQ(classify_receipt(at_ms(100), t((100 * kNsPerMs) + 1), at_ms(120)),
            ReceiptOutcome::kLate);
  EXPECT_EQ(classify_receipt(at_ms(100), std::nullopt, at_ms(100)), ReceiptOutcome::kPending);
  EXPECT_EQ(classify_receipt(at_ms(100), std::nullopt, t((100 * kNsPerMs) + 1)),
            ReceiptOutcome::kMissing);
}

TEST(ReceiptOutcome, IgnoresAReceiptAfterTheHorizon) {
  EXPECT_EQ(classify_receipt(at_ms(100), at_ms(90), at_ms(80)), ReceiptOutcome::kPending);
  EXPECT_EQ(classify_receipt(at_ms(100), at_ms(130), at_ms(120)), ReceiptOutcome::kMissing);
}

constexpr WorkerId kWorker{7};

void admit(ReceiptLedger& ledger, std::uint64_t job, std::int64_t deadline_ms) {
  ASSERT_TRUE(ledger.offer(JobId{job}, at_ms(0), at_ms(deadline_ms)));
  ASSERT_TRUE(ledger.assign(JobId{job}, kWorker));
}

std::optional<Receipt> respond(ReceiptLedger& ledger, std::uint64_t job, std::int64_t at) {
  const Response response{.job = JobId{job}, .worker = kWorker, .observation = at_ms(0)};
  return ledger.receive(response, at_ms(at));
}

// One on time, one late, one missing, and one pending at 120 ms.
TEST(ReceiptLedger, SummarizesEveryOfferedRequestAndBoundsAnUnsettledRatio) {
  ReceiptLedger ledger;
  admit(ledger, 1, 100);
  admit(ledger, 2, 100);
  admit(ledger, 3, 100);
  admit(ledger, 4, 200);

  EXPECT_EQ(respond(ledger, 1, 90), (Receipt{.job = JobId{1},
                                             .deadline = at_ms(100),
                                             .at = at_ms(90),
                                             .outcome = ReceiptOutcome::kOnTime}));
  EXPECT_EQ(respond(ledger, 2, 105)->outcome, ReceiptOutcome::kLate);
  const ReceiptSummary summary = ledger.summary(at_ms(120));

  EXPECT_EQ(summary,
            (ReceiptSummary{.offered = 4, .on_time = 1, .late = 1, .missing = 1, .pending = 1}));
  EXPECT_EQ(summary.on_time_ratio(), std::nullopt);
  EXPECT_EQ(
      summary.on_time_bounds(),
      (std::optional{std::pair{Fraction{.count = 1, .of = 4}, Fraction{.count = 2, .of = 4}}}));
}

TEST(ReceiptLedger, SettlesTheRatioOnceNothingIsPending) {
  ReceiptLedger ledger;
  admit(ledger, 1, 100);
  admit(ledger, 2, 100);
  ASSERT_TRUE(respond(ledger, 1, 100).has_value());

  const ReceiptSummary summary = ledger.summary(at_ms(101));

  EXPECT_EQ(summary.on_time_ratio(), (Fraction{.count = 1, .of = 2}));
  EXPECT_EQ(
      summary.on_time_bounds(),
      (std::optional{std::pair{Fraction{.count = 1, .of = 2}, Fraction{.count = 1, .of = 2}}}));
}

TEST(ReceiptLedger, LeavesTheRatioUndefinedWhenNothingWasOffered) {
  const ReceiptSummary summary = ReceiptLedger{}.summary(at_ms(100));

  EXPECT_EQ(summary, ReceiptSummary{});
  EXPECT_EQ(summary.on_time_ratio(), std::nullopt);
  EXPECT_EQ(summary.on_time_bounds(), std::nullopt);
}

TEST(ReceiptLedger, CountsOnlyTheFirstResponseMatchingAnAdmittedRequest) {
  ReceiptLedger ledger;
  ASSERT_TRUE(ledger.offer(JobId{1}, at_ms(0), at_ms(100)));
  EXPECT_FALSE(ledger.offer(JobId{1}, at_ms(0), at_ms(300)));
  EXPECT_EQ(respond(ledger, 1, 50), std::nullopt);
  EXPECT_FALSE(ledger.assign(JobId{2}, kWorker));
  ASSERT_TRUE(ledger.assign(JobId{1}, kWorker));
  EXPECT_FALSE(ledger.assign(JobId{1}, WorkerId{8}));

  EXPECT_EQ(respond(ledger, 2, 50), std::nullopt);
  EXPECT_EQ(
      ledger.receive({.job = JobId{1}, .worker = WorkerId{8}, .observation = at_ms(0)}, at_ms(50)),
      std::nullopt);
  EXPECT_EQ(
      ledger.receive({.job = JobId{1}, .worker = kWorker, .observation = at_ms(1)}, at_ms(50)),
      std::nullopt);
  EXPECT_EQ(respond(ledger, 1, 150)->outcome, ReceiptOutcome::kLate);
  EXPECT_EQ(respond(ledger, 1, 160), std::nullopt);

  EXPECT_EQ(ledger.deadline_of(JobId{1}), std::nullopt);
  EXPECT_EQ(ledger.summary(at_ms(200)), (ReceiptSummary{.offered = 1, .late = 1}));
}

TEST(ReceiptLedger, KeepsAnUnansweredRequestOpenWithItsDeadline) {
  ReceiptLedger ledger;
  admit(ledger, 1, 100);

  EXPECT_EQ(ledger.deadline_of(JobId{1}), at_ms(100));
  EXPECT_EQ(ledger.summary(at_ms(100)), (ReceiptSummary{.offered = 1, .pending = 1}));
  EXPECT_EQ(ledger.summary(at_ms(101)), (ReceiptSummary{.offered = 1, .missing = 1}));
}

}  // namespace
}  // namespace rovi::core
