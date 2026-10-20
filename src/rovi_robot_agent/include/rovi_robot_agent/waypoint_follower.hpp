// Heading-proportional waypoint following and odometry pose extraction for a diff-drive robot.
#ifndef ROVI_ROBOT_AGENT_WAYPOINT_FOLLOWER_HPP_
#define ROVI_ROBOT_AGENT_WAYPOINT_FOLLOWER_HPP_

#include <cstddef>
#include <optional>
#include <vector>

namespace rovi::robot_agent {

struct Point2D {
  double x{0.0};
  double y{0.0};

  bool operator==(const Point2D&) const = default;
};

struct Pose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Velocity {
  double linear{0.0};
  double angular{0.0};

  bool operator==(const Velocity&) const = default;
};

struct FollowerConfig {
  std::vector<Point2D> waypoints;
  double arrival_tolerance{0.0};
  double max_linear{0.0};
  double max_angular{0.0};
  double heading_gain{0.0};
};

[[nodiscard]] double yaw_from_quaternion(double x, double y, double z, double w);

[[nodiscard]] std::optional<Pose2D> planar_pose(double x, double y, double qx, double qy, double qz,
                                                double qw);

class WaypointFollower {
 public:
  [[nodiscard]] static std::optional<WaypointFollower> create(FollowerConfig config);

  [[nodiscard]] Velocity command(const Pose2D& pose);
  [[nodiscard]] std::size_t target() const;

 private:
  explicit WaypointFollower(FollowerConfig config);

  FollowerConfig config_;
  std::size_t target_{0};
};

}  // namespace rovi::robot_agent

#endif  // ROVI_ROBOT_AGENT_WAYPOINT_FOLLOWER_HPP_
