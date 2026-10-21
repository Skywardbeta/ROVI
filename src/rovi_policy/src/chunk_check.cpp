#include "rovi_policy/chunk_check.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

#include "rovi_policy/wire.hpp"

namespace rovi::policy {

std::string_view fault_name(ChunkFault fault) {
  switch (fault) {
    case ChunkFault::kShape:
      return "shape";
    case ChunkFault::kNonFinite:
      return "non-finite";
    case ChunkFault::kOutOfRange:
      return "out-of-range";
    case ChunkFault::kPeriod:
      return "period";
    case ChunkFault::kPolicy:
      return "policy";
  }
  return "unknown";
}

bool limits_fit(const JointLimits& limits, std::uint32_t dim) {
  if (limits.lower.size() != dim || limits.upper.size() != dim ||
      !std::isfinite(limits.tolerance) || limits.tolerance < 0.0F) {
    return false;
  }
  for (std::size_t joint = 0; joint < dim; ++joint) {
    if (!std::isfinite(limits.lower.at(joint)) || !std::isfinite(limits.upper.at(joint)) ||
        limits.lower.at(joint) > limits.upper.at(joint)) {
      return false;
    }
  }
  return true;
}

std::optional<ChunkFault> check_chunk(std::span<const float> actions, ChunkShape actual,
                                      ChunkShape expected, const JointLimits* limits) {
  if (actual != expected || expected.dim == 0 ||
      actions.size() != std::size_t{actual.steps} * actual.dim ||
      (limits != nullptr && !limits_fit(*limits, expected.dim))) {
    return ChunkFault::kShape;
  }
  if (std::ranges::any_of(actions, [](float value) { return !std::isfinite(value); })) {
    return ChunkFault::kNonFinite;
  }
  if (limits == nullptr) {
    return std::nullopt;
  }
  std::size_t joint = 0;
  for (const float value : actions) {
    if (value < limits->lower.at(joint) - limits->tolerance ||
        value > limits->upper.at(joint) + limits->tolerance) {
      return ChunkFault::kOutOfRange;
    }
    joint = (joint + 1) % expected.dim;
  }
  return std::nullopt;
}

std::optional<std::int64_t> playback_ns(ChunkShape shape, std::int64_t period_ns) {
  std::int64_t total = 0;
  if (period_ns <= 0 || __builtin_mul_overflow(std::int64_t{shape.steps}, period_ns, &total)) {
    return std::nullopt;
  }
  return total;
}

bool contract_fits(const ChunkContract& contract) {
  return contract.shape.steps > 0 && limits_fit(contract.limits, contract.shape.dim) &&
         playback_ns(contract.shape, contract.period_ns).has_value() &&
         policy_from_hex(contract.policy_id).has_value();
}

std::optional<ChunkFault> check_delivery(const ChunkContract& contract,
                                         std::span<const float> actions, ChunkShape actual,
                                         std::int64_t period_ns, std::string_view policy_id) {
  if (policy_id != contract.policy_id) {
    return ChunkFault::kPolicy;
  }
  if (period_ns != contract.period_ns) {
    return ChunkFault::kPeriod;
  }
  return check_chunk(actions, actual, contract.shape, &contract.limits);
}

}  // namespace rovi::policy
