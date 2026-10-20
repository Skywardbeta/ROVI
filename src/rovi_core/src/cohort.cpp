#include "rovi_core/cohort.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <tuple>
#include <vector>

#include "rovi_core/digest.hpp"
#include "rovi_core/units.hpp"

namespace rovi::core {
namespace {

constexpr unsigned kSequenceBits = 32;
constexpr std::uint64_t kGoldenGamma = 0x9E37'79B9'7F4A'7C15ULL;
constexpr std::uint64_t kFirstMultiplier = 0xBF58'476D'1CE4'E5B9ULL;
constexpr std::uint64_t kSecondMultiplier = 0x94D0'49BB'1331'11EBULL;
constexpr unsigned kFirstShift = 30;
constexpr unsigned kSecondShift = 27;
constexpr unsigned kLastShift = 31;

// SplitMix64 finalizer: a stable pseudo-random word for one (seed, robot) identity.
constexpr std::uint64_t mixed(std::uint64_t seed, std::uint32_t robot) {
  std::uint64_t word = seed + ((std::uint64_t{robot} + 1) * kGoldenGamma);
  word = (word ^ (word >> kFirstShift)) * kFirstMultiplier;
  word = (word ^ (word >> kSecondShift)) * kSecondMultiplier;
  return word ^ (word >> kLastShift);
}

bool valid(const CohortSpec& spec) {
  return spec.robots > 0 && spec.requests_per_robot > 0 && spec.period > Duration{} &&
         spec.phase_spread >= Duration{} && spec.relative_deadline > Duration{} &&
         spec.upload > Bytes{0} && spec.response > Bytes{0} &&
         std::uint64_t{spec.robots} * spec.requests_per_robot <= kMaximumCohort;
}

}  // namespace

std::optional<std::vector<Request>> make_cohort(const CohortSpec& spec) {
  if (!valid(spec)) {
    return std::nullopt;
  }
  std::vector<Request> cohort;
  cohort.reserve(std::size_t{spec.robots} * spec.requests_per_robot);
  for (std::uint32_t robot = 0; robot < spec.robots; ++robot) {
    const std::int64_t phase =
        spec.phase_spread == Duration{}
            ? 0
            : static_cast<std::int64_t>(mixed(spec.seed, robot) %
                                        static_cast<std::uint64_t>(spec.phase_spread.ns()));
    std::optional<TimePoint> release = TimePoint::from_ns(phase);
    for (std::uint32_t sequence = 0; sequence < spec.requests_per_robot; ++sequence) {
      const std::optional<TimePoint> deadline =
          release ? checked_add(*release, spec.relative_deadline) : std::nullopt;
      if (!deadline) {
        return std::nullopt;
      }
      cohort.push_back(Request{.id = JobId{(std::uint64_t{robot} << kSequenceBits) | sequence},
                               .robot = RobotId{robot},
                               .release = *release,
                               .deadline = *deadline,
                               .upload = spec.upload,
                               .response = spec.response});
      release = checked_add(*release, spec.period);
    }
  }
  std::ranges::sort(cohort, [](const Request& lhs, const Request& rhs) {
    return std::tie(lhs.release, lhs.robot, lhs.id) < std::tie(rhs.release, rhs.robot, rhs.id);
  });
  return cohort;
}

std::uint64_t cohort_digest(const std::vector<Request>& cohort) {
  Fnv1aDigest digest;
  for (const Request& request : cohort) {
    digest.add(request.id.value());
    digest.add(std::uint64_t{request.robot.value()});
    digest.add(request.release.ns());
    digest.add(request.deadline.ns());
    digest.add(request.upload.value());
    digest.add(request.response.value());
    if (request.send_delay != Duration{}) {
      digest.add(request.send_delay.ns());
    }
  }
  return digest.value();
}

}  // namespace rovi::core
