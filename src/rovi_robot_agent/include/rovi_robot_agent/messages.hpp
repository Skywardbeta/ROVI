// Message construction and decoding for the robot agent's job requests, decisions, and state.
#ifndef ROVI_ROBOT_AGENT_MESSAGES_HPP_
#define ROVI_ROBOT_AGENT_MESSAGES_HPP_

#include <cstdint>
#include <optional>

#include "rovi_core/admission.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/units.hpp"
#include "rovi_interfaces/msg/job_request.hpp"
#include "rovi_interfaces/msg/robot_state.hpp"
#include "rovi_robot_agent/robot_agent.hpp"

namespace rovi::robot_agent {

[[nodiscard]] rovi_interfaces::msg::JobRequest to_request(const core::Job& job);

[[nodiscard]] std::optional<core::DecisionStatus> status_from(std::uint8_t status);

[[nodiscard]] rovi_interfaces::msg::RobotState to_state(const RobotAgent& agent,
                                                        core::TimePoint now);

}  // namespace rovi::robot_agent

#endif  // ROVI_ROBOT_AGENT_MESSAGES_HPP_
