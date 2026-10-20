#include "rovi_core/job.hpp"

#include <optional>

#include <gtest/gtest.h>

#include "rovi_core/units.hpp"

#include "printers.hpp"  // IWYU pragma: keep
#include "test_support.hpp"

namespace rovi::core {
namespace {

Job job_with(TimePoint buffer_empty_time, Duration safety_margin,
             std::optional<TimePoint> task_deadline, std::optional<Duration> maximum_result_age) {
  return Job{.id = JobId{1},
             .release_time = t(150),
             .observation_time = t(100),
             .buffer_empty_time = buffer_empty_time,
             .safety_margin = safety_margin,
             .task_deadline = task_deadline,
             .maximum_result_age = maximum_result_age,
             .upload_bytes = Bytes{1},
             .action_chunk_bytes = Bytes{1},
             .model = ModelClass{0},
             .eligible_workers = {}};
}

TEST(EffectiveDeadline, UsesBufferBoundWhenNoOtherBoundApplies) {
  EXPECT_EQ(effective_deadline(job_with(t(1'000), d(100), std::nullopt, std::nullopt)), t(900));
}

TEST(EffectiveDeadline, FreshnessBoundIsMeasuredFromObservationTime) {
  EXPECT_EQ(effective_deadline(job_with(t(1'000), d(100), std::nullopt, d(300))), t(400));
}

TEST(EffectiveDeadline, TaskDeadlineBindsWhenEarliest) {
  EXPECT_EQ(effective_deadline(job_with(t(1'000), d(100), t(350), d(300))), t(350));
}

TEST(EffectiveDeadline, BufferBoundBindsWhenEarlierThanTaskAndFreshness) {
  EXPECT_EQ(effective_deadline(job_with(t(1'000), d(700), t(500), d(500))), t(300));
}

TEST(EffectiveDeadline, ReportsArithmeticOverflowInsteadOfWrapping) {
  EXPECT_EQ(effective_deadline(job_with(t(kMinNs), d(1), std::nullopt, std::nullopt)),
            std::nullopt);
  EXPECT_EQ(effective_deadline(job_with(t(1'000), d(100), std::nullopt, d(kMaxNs))), std::nullopt);
}

}  // namespace
}  // namespace rovi::core
