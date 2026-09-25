#include <chrono>
#include <cmath>

#include "control_node.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace {
double quaternionToYaw(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  robot::ControlConfig config;
  config.lookahead_distance = this->declare_parameter<double>("lookahead_distance", config.lookahead_distance);
  config.goal_tolerance = this->declare_parameter<double>("goal_tolerance", config.goal_tolerance);
  config.linear_speed = this->declare_parameter<double>("linear_speed", config.linear_speed);
  config.max_angular_speed = this->declare_parameter<double>("max_angular_speed", config.max_angular_speed);
  const double control_frequency = this->declare_parameter<double>("control_frequency", 10.0);

  control_.configure(config);

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  const auto period = std::chrono::duration<double>(1.0 / control_frequency);
  timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  current_path_ = *msg;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  robot_yaw_ = quaternionToYaw(msg->pose.pose.orientation);
  odom_received_ = true;
}

void ControlNode::controlLoop() {
  if (!odom_received_) return;

  const auto cmd_vel = control_.computeVelocity(current_path_, robot_x_, robot_y_, robot_yaw_);
  cmd_vel_pub_->publish(cmd_vel);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
