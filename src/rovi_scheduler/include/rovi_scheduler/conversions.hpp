// Conversions between rovi_interfaces messages and rovi_core values at the scheduler boundary.
#ifndef ROVI_SCHEDULER_CONVERSIONS_HPP_
#define ROVI_SCHEDULER_CONVERSIONS_HPP_

#include <cstdint>
#include <optional>

#include "rovi_core/admission.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/scheduler.hpp"
#include "rovi_core/units.hpp"
#include "rovi_interfaces/msg/decision.hpp"
#include "rovi_interfaces/msg/job_request.hpp"
#include "rovi_interfaces/msg/reservation.hpp"
#include "rovi_interfaces/msg/service_source.hpp"

namespace rovi::scheduler {

struct DecisionContext {
  std::uint64_t cycle{0};
  core::TimePoint decided;
  core::ServiceSource source{core::ServiceSource::kLogical};
};

[[nodiscard]] core::Job to_job(const rovi_interfaces::msg::JobRequest& request);

[[nodiscard]] std::uint8_t to_status(core::DecisionStatus status);

[[nodiscard]] std::uint8_t to_reason(core::ReasonCode reason);

[[nodiscard]] std::uint8_t to_policy(core::Policy policy);

[[nodiscard]] rovi_interfaces::msg::ServiceSource to_source(core::ServiceSource source);

[[nodiscard]] rovi_interfaces::msg::Decision to_decision(const core::Job& job,
                                                         const core::Decision& decision,
                                                         const DecisionContext& context);

[[nodiscard]] std::optional<rovi_interfaces::msg::Reservation> to_reservation(
    const core::ChainReservation& reservation, const core::ResourceSnapshot& snapshot);

}  // namespace rovi::scheduler

#endif  // ROVI_SCHEDULER_CONVERSIONS_HPP_
