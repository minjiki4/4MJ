#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot
{

struct ControlConfig {
  double lookahead_distance = 1.0;  // meters
  double goal_tolerance = 0.2;      // meters
  double linear_speed = 0.5;        // m/s
  double max_angular_speed = 2.0;   // rad/s
};

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    void configure(const ControlConfig& config);

    // Computes the velocity command that steers the robot along path.
    // Returns a zero Twist when the path is empty or the robot has reached its final pose.
    geometry_msgs::msg::Twist computeVelocity(const nav_msgs::msg::Path& path,
                                               double robot_x, double robot_y, double robot_yaw) const;

  private:
    geometry_msgs::msg::Point findLookaheadPoint(const nav_msgs::msg::Path& path,
                                                  double robot_x, double robot_y) const;
    static double computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b);

    rclcpp::Logger logger_;
    ControlConfig config_;
};

}  // namespace robot

#endif
