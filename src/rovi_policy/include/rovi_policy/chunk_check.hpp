// Shape, finiteness, and joint-limit checks of an action chunk in policy units.
#ifndef ROVI_POLICY_CHUNK_CHECK_HPP_
#define ROVI_POLICY_CHUNK_CHECK_HPP_

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace rovi::policy {

struct ChunkShape {
  std::uint32_t steps{0};
  std::uint32_t dim{0};

  bool operator==(const ChunkShape&) const = default;
};

// Per-joint limits; a value may exceed them by at most tolerance.
struct JointLimits {
  std::vector<float> lower;
  std::vector<float> upper;
  float tolerance{0.0F};
};

// What a robot accepts: the shape, playback period, joint limits, and producing policy (SHA-256
// hex).
struct ChunkContract {
  ChunkShape shape;
  std::int64_t period_ns{0};
  JointLimits limits;
  std::string policy_id;
};

enum class ChunkFault : std::uint8_t { kShape, kNonFinite, kOutOfRange, kPeriod, kPolicy };

[[nodiscard]] std::string_view fault_name(ChunkFault fault);

[[nodiscard]] bool limits_fit(const JointLimits& limits, std::uint32_t dim);

[[nodiscard]] std::optional<ChunkFault> check_chunk(std::span<const float> actions,
                                                    ChunkShape actual, ChunkShape expected,
                                                    const JointLimits* limits);

// How long a chunk plays: steps * period, or nullopt if the period is not positive or it overflows.
[[nodiscard]] std::optional<std::int64_t> playback_ns(ChunkShape shape, std::int64_t period_ns);
[[nodiscard]] bool contract_fits(const ChunkContract& contract);
[[nodiscard]] std::optional<ChunkFault> check_delivery(const ChunkContract& contract,
                                                       std::span<const float> actions,
                                                       ChunkShape actual, std::int64_t period_ns,
                                                       std::string_view policy_id);

}  // namespace rovi::policy

#endif  // ROVI_POLICY_CHUNK_CHECK_HPP_
