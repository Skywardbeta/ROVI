#include "rovi_core/cohort.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "rovi_core/units.hpp"

namespace rovi::core {
namespace {

constexpr std::int64_t kMillisecond = 1'000'000;

CohortSpec spec(std::uint32_t robots, std::int64_t spread_ns, std::uint64_t seed) {
  return CohortSpec{.robots = robots,
                    .requests_per_robot = 2,
                    .period = Duration::from_ns(100 * kMillisecond),
                    .phase_spread = Duration::from_ns(spread_ns),
                    .relative_deadline = Duration::from_ns(40 * kMillisecond),
                    .upload = Bytes{100'000},
                    .response = Bytes{10'000},
                    .seed = seed};
}

TimePoint first_release_of(const std::vector<Request>& cohort, std::uint32_t robot) {
  const auto found = std::ranges::find(cohort, RobotId{robot}, &Request::robot);
  return found == cohort.end() ? TimePoint::from_ns(-1) : found->release;
}

TEST(Cohort, ASynchronizedBurstReleasesEveryRobotTogether) {
  const std::optional<std::vector<Request>> cohort = make_cohort(spec(3, 0, 7));
  ASSERT_TRUE(cohort.has_value());
  ASSERT_EQ(cohort->size(), 6U);
  for (std::size_t index = 0; index < cohort->size(); ++index) {
    const Request& request = cohort->at(index);
    const auto round = static_cast<std::int64_t>(index / 3);
    EXPECT_EQ(request.robot, RobotId{static_cast<std::uint32_t>(index % 3)});
    EXPECT_EQ(request.release, TimePoint::from_ns(round * 100 * kMillisecond));
    EXPECT_EQ(request.deadline, TimePoint::from_ns(((round * 100) + 40) * kMillisecond));
    EXPECT_EQ(request.id, JobId{(std::uint64_t{request.robot.value()} << 32U) |
                                static_cast<std::uint64_t>(round)});
    EXPECT_EQ(request.upload, Bytes{100'000});
    EXPECT_EQ(request.response, Bytes{10'000});
  }
}

TEST(Cohort, PhasesDependOnlyOnTheSeedAndTheRobot) {
  const std::optional<std::vector<Request>> three = make_cohort(spec(3, 20 * kMillisecond, 7));
  const std::optional<std::vector<Request>> five = make_cohort(spec(5, 20 * kMillisecond, 7));
  const std::optional<std::vector<Request>> other = make_cohort(spec(5, 20 * kMillisecond, 8));
  ASSERT_TRUE(three && five && other);
  bool seed_matters = false;
  for (std::uint32_t robot = 0; robot < 5; ++robot) {
    const TimePoint phase = first_release_of(*five, robot);
    EXPECT_GE(phase, TimePoint::from_ns(0));
    EXPECT_LT(phase, TimePoint::from_ns(20 * kMillisecond));
    if (robot < 3) {
      EXPECT_EQ(first_release_of(*three, robot), phase) << "robot " << robot;
    }
    seed_matters = seed_matters || first_release_of(*other, robot) != phase;
  }
  EXPECT_TRUE(seed_matters);
}

TEST(Cohort, IsOrderedByReleaseAndReproducible) {
  const std::optional<std::vector<Request>> cohort = make_cohort(spec(5, 20 * kMillisecond, 7));
  const std::optional<std::vector<Request>> again = make_cohort(spec(5, 20 * kMillisecond, 7));
  const std::optional<std::vector<Request>> other = make_cohort(spec(5, 20 * kMillisecond, 8));
  ASSERT_TRUE(cohort && again && other);
  EXPECT_EQ(*cohort, *again);
  EXPECT_EQ(cohort_digest(*cohort), cohort_digest(*again));
  EXPECT_NE(cohort_digest(*cohort), cohort_digest(*other));
  for (std::size_t index = 1; index < cohort->size(); ++index) {
    EXPECT_LE(cohort->at(index - 1).release, cohort->at(index).release);
  }
}

TEST(Cohort, RejectsEmptyNonPositiveOversizedAndOverflowingSpecifications) {
  const CohortSpec valid = spec(2, 0, 1);
  ASSERT_TRUE(make_cohort(valid).has_value());
  CohortSpec bad = valid;
  bad.robots = 0;
  EXPECT_FALSE(make_cohort(bad).has_value());
  bad = valid;
  bad.requests_per_robot = 0;
  EXPECT_FALSE(make_cohort(bad).has_value());
  bad = valid;
  bad.period = Duration{};
  EXPECT_FALSE(make_cohort(bad).has_value());
  bad = valid;
  bad.phase_spread = Duration::from_ns(-1);
  EXPECT_FALSE(make_cohort(bad).has_value());
  bad = valid;
  bad.relative_deadline = Duration{};
  EXPECT_FALSE(make_cohort(bad).has_value());
  bad = valid;
  bad.upload = Bytes{0};
  EXPECT_FALSE(make_cohort(bad).has_value());
  bad = valid;
  bad.response = Bytes{0};
  EXPECT_FALSE(make_cohort(bad).has_value());
  bad = valid;
  bad.robots = 1'001;
  bad.requests_per_robot = 1'000;
  EXPECT_FALSE(make_cohort(bad).has_value());
  bad = valid;
  bad.requests_per_robot = 3;
  bad.period = Duration::from_ns(INT64_MAX / 2);
  EXPECT_FALSE(make_cohort(bad).has_value());
}

TEST(Cohort, ASendDelayChangesTheDigestOnlyWhenItIsNonZero) {
  const std::optional<std::vector<Request>> cohort = make_cohort(spec(3, 0, 1));
  ASSERT_TRUE(cohort.has_value());
  std::vector<Request> delayed = *cohort;
  delayed.front().send_delay = Duration::from_ns(1);
  EXPECT_NE(cohort_digest(delayed), cohort_digest(*cohort));
  delayed.front().send_delay = Duration{};
  EXPECT_EQ(cohort_digest(delayed), cohort_digest(*cohort));
}

}  // namespace
}  // namespace rovi::core
