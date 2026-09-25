#include "control_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) : logger_(logger) {}

void ControlCore::configure(const ControlConfig& config) {
  config_ = config;
}

double ControlCore::computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

geometry_msgs::msg::Point ControlCore::findLookaheadPoint(const nav_msgs::msg::Path& path,
                                                            double robot_x, double robot_y) const {
  geometry_msgs::msg::Point robot_point;
  robot_point.x = robot_x;
  robot_point.y = robot_y;

  // start from the closest pose, not index 0 -- otherwise a path that loops
  // back near itself could pick a lookahead point behind the robot
  size_t closest_index = 0;
  double closest_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const double d = computeDistance(robot_point, path.poses[i].pose.position);
    if (d < closest_distance) {
      closest_distance = d;
      closest_index = i;
    }
  }

  for (size_t i = closest_index; i < path.poses.size(); ++i) {
    if (computeDistance(robot_point, path.poses[i].pose.position) >= config_.lookahead_distance) {
      return path.poses[i].pose.position;
    }
  }

  // No point far enough ahead was found: aim at the final waypoint.
  return path.poses.back().pose.position;
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(const nav_msgs::msg::Path& path,
                                                         double robot_x, double robot_y, double robot_yaw) const {
  geometry_msgs::msg::Twist cmd_vel;

  if (path.poses.empty()) {
    return cmd_vel;
  }

  geometry_msgs::msg::Point robot_point;
  robot_point.x = robot_x;
  robot_point.y = robot_y;

  if (computeDistance(robot_point, path.poses.back().pose.position) < config_.goal_tolerance) {
    return cmd_vel;
  }

  const auto target = findLookaheadPoint(path, robot_x, robot_y);

  // to robot-local frame
  const double dx = target.x - robot_x;
  const double dy = target.y - robot_y;
  const double local_x = dx * std::cos(robot_yaw) + dy * std::sin(robot_yaw);
  const double local_y = -dx * std::sin(robot_yaw) + dy * std::cos(robot_yaw);

  const double lookahead = std::max(std::hypot(local_x, local_y), 1e-3);
  const double curvature = 2.0 * local_y / (lookahead * lookahead);

  cmd_vel.linear.x = config_.linear_speed;
  cmd_vel.angular.z = std::clamp(curvature * config_.linear_speed,
                                  -config_.max_angular_speed, config_.max_angular_speed);

  return cmd_vel;
}

}  // namespace robot
