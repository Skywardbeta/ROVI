#include "rovi_telemetry/run_clock.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rcl/time.h>
#include <rcl/types.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>

#include "rovi_core/units.hpp"
#include "rovi_interfaces/msg/run_invalidation.hpp"

namespace rovi::telemetry {
namespace {

using rovi_interfaces::msg::RunInvalidation;

constexpr std::chrono::seconds kDeliveryBudget{5};

class RunClockTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    probe_ = std::make_shared<rclcpp::Node>("probe");
    heard_ = probe_->create_subscription<RunInvalidation>(
        "/rovi/run/invalidations", latched(),
        [this](const RunInvalidation& message) { invalidations_.push_back(message); });
    executor_.add_node(probe_);
  }

  // A node whose ROS time is set directly, as the time source sets it from /clock.
  std::shared_ptr<rclcpp::Node> component(const std::string& name) {
    auto node = std::make_shared<rclcpp::Node>(name);
    EXPECT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()), RCL_RET_OK);
    executor_.add_node(node);
    return node;
  }

  static void set_time(rclcpp::Node& node, std::int64_t ns) {
    ASSERT_EQ(rcl_set_ros_time_override(node.get_clock()->get_clock_handle(), ns), RCL_RET_OK);
  }

  bool spin_until(const std::function<bool()>& done) {
    const auto deadline = std::chrono::steady_clock::now() + kDeliveryBudget;
    while (!done() && std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some(std::chrono::milliseconds{10});
    }
    return done();
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr probe_;
  rclcpp::Subscription<RunInvalidation>::SharedPtr heard_;
  std::vector<RunInvalidation> invalidations_;
};

TEST_F(RunClockTest, ARewindRecoveredBeforeTheNextCallbackStillInvalidatesTheRun) {
  const auto node = component("rewound");
  RunClock clock{*node};
  set_time(*node, 2'000);
  ASSERT_EQ(clock.now(), core::TimePoint::from_ns(2'000));

  set_time(*node, 1'000);
  set_time(*node, 3'000);

  EXPECT_EQ(clock.now(), std::nullopt);
  ASSERT_TRUE(spin_until([this] { return !invalidations_.empty(); }));
  EXPECT_EQ(invalidations_.front().node, "/rewound");
  EXPECT_EQ(invalidations_.front().latest_ns, 2'000);
  EXPECT_EQ(invalidations_.front().observed_ns, 1'000);
}

TEST_F(RunClockTest, AComponentThatNeverSeesTheRewindStopsOnAPeerInvalidation) {
  const auto rewound = component("rewound");
  const auto steady = component("steady");
  RunClock rewound_clock{*rewound};
  RunClock steady_clock{*steady};
  set_time(*rewound, 2'000);
  set_time(*steady, 2'000);
  ASSERT_TRUE(rewound_clock.now().has_value());
  ASSERT_TRUE(steady_clock.now().has_value());

  set_time(*rewound, 1'000);
  ASSERT_EQ(rewound_clock.now(), std::nullopt);
  set_time(*steady, 3'000);

  EXPECT_TRUE(spin_until([this, &steady_clock] {
    return !invalidations_.empty() && !steady_clock.now().has_value();
  }));
  ASSERT_EQ(invalidations_.size(), 1U);
  EXPECT_EQ(invalidations_.front().node, "/rewound");
}

}  // namespace
}  // namespace rovi::telemetry
