// Run-wide clock discipline and settings provenance shared by every ROVI component.
#ifndef ROVI_TELEMETRY_RUN_CLOCK_HPP_
#define ROVI_TELEMETRY_RUN_CLOCK_HPP_

#include <algorithm>
#include <atomic>
#include <exception>
#include <optional>
#include <string>
#include <vector>

#include <rcl/time.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>

#include "rovi_core/clock_guard.hpp"
#include "rovi_core/units.hpp"
#include "rovi_interfaces/msg/component_settings.hpp"
#include "rovi_interfaces/msg/run_invalidation.hpp"

namespace rovi::telemetry {

inline rclcpp::QoS latched() {
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

// Fail-stop ROS time for one component. ROS time moving backward invalidates the run, whether a
// callback reads the earlier time or the clock's jump handler sees /clock step back between
// callbacks. The component then publishes one RunInvalidation, logs that a clean restart is
// required, and returns no time from then on. Another component's invalidation stops it as well,
// so a component whose own clock never moved backward cannot continue the abandoned run.
class RunClock {
 public:
  explicit RunClock(rclcpp::Node& node)
      : clock_{node.get_clock()},
        logger_{node.get_logger()},
        node_{node.get_fully_qualified_name()},
        invalidations_{node.create_publisher<rovi_interfaces::msg::RunInvalidation>(
            "/rovi/run/invalidations", latched())},
        peers_{node.create_subscription<rovi_interfaces::msg::RunInvalidation>(
            "/rovi/run/invalidations", latched(),
            [this](const rovi_interfaces::msg::RunInvalidation& message) { on_peer(message); })},
        jumps_{clock_->create_jump_callback([this] { invalid_ = true; },
                                            [this](const rcl_time_jump_t& jump) { on_jump(jump); },
                                            kAnyBackwardStep)} {}

  [[nodiscard]] std::optional<core::TimePoint> now() {
    if (invalid_) {
      return std::nullopt;
    }
    const core::TimePoint now = core::TimePoint::from_ns(clock_->now().nanoseconds());
    const std::optional<core::TimePoint> latest = guard_.latest();
    if (guard_.observe(now) == core::ClockStep::kBackward) {
      report(latest.value_or(now), now);
    }
    // The jump handler marks the run invalid before it stores a rewound time, so a time read
    // while the handler runs on the clock thread is caught here.
    if (invalid_) {
      return std::nullopt;
    }
    return now;
  }

 private:
  static constexpr rcl_jump_threshold_t kAnyBackwardStep{.on_clock_change = false,
                                                         .min_forward = {.nanoseconds = 0},
                                                         .min_backward = {.nanoseconds = -1}};

  void on_jump(const rcl_time_jump_t& jump) {
    try {
      const core::TimePoint observed = core::TimePoint::from_ns(clock_->now().nanoseconds());
      report(core::checked_sub(observed, core::Duration::from_ns(jump.delta.nanoseconds))
                 .value_or(observed),
             observed);
    } catch (const std::exception& error) {
      RCLCPP_ERROR_STREAM(logger_, "could not report a backward ROS time jump: " << error.what());
    }
  }

  void on_peer(const rovi_interfaces::msg::RunInvalidation& message) {
    if (message.node == node_ || invalid_.exchange(true)) {
      return;
    }
    RCLCPP_ERROR_STREAM(logger_, message.node << " saw ROS time move backward; this run is invalid "
                                                 "and must be restarted");
  }

  void report(core::TimePoint latest, core::TimePoint observed) {
    invalid_ = true;
    if (reported_.exchange(true)) {
      return;
    }
    rovi_interfaces::msg::RunInvalidation message;
    message.node = node_;
    message.latest_ns = latest.ns();
    message.observed_ns = observed.ns();
    try {
      invalidations_->publish(message);
    } catch (const std::exception& error) {
      RCLCPP_ERROR_STREAM(logger_, "could not publish the run invalidation: " << error.what());
    }
    RCLCPP_ERROR_STREAM(logger_, "ROS time moved backward from "
                                     << latest.ns() << " to " << observed.ns()
                                     << " ns; this run is invalid and must be restarted");
  }

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;
  std::string node_;
  rclcpp::Publisher<rovi_interfaces::msg::RunInvalidation>::SharedPtr invalidations_;
  core::ClockGuard guard_;
  std::atomic<bool> invalid_{false};
  std::atomic<bool> reported_{false};
  rclcpp::Subscription<rovi_interfaces::msg::RunInvalidation>::SharedPtr peers_;
  // Declared last so the jump handler is removed before anything it uses is destroyed.
  rclcpp::JumpHandler::SharedPtr jumps_;
};

// Publishes the node's declared parameters once, after defaults and overrides; the caller keeps
// the publisher alive so the transient-local message reaches a late recorder. Overrides the node
// never declares, and later runtime changes, are not captured.
inline rclcpp::Publisher<rovi_interfaces::msg::ComponentSettings>::SharedPtr announce_settings(
    rclcpp::Node& node) {
  auto publisher = node.create_publisher<rovi_interfaces::msg::ComponentSettings>(
      "/rovi/run/components", latched());
  std::vector<std::string> names = node.list_parameters({}, 0).names;
  std::ranges::sort(names);
  rovi_interfaces::msg::ComponentSettings message;
  message.node = node.get_fully_qualified_name();
  for (const rclcpp::Parameter& parameter : node.get_parameters(names)) {
    message.names.push_back(parameter.get_name());
    message.values.push_back(parameter.value_to_string());
  }
  publisher->publish(message);
  return publisher;
}

}  // namespace rovi::telemetry

#endif  // ROVI_TELEMETRY_RUN_CLOCK_HPP_
