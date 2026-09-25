#include <chrono>
#include <cmath>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  robot::PlannerConfig config;
  config.occupancy_threshold = this->declare_parameter<int>("occupancy_threshold", config.occupancy_threshold);
  goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.5);
  const double timer_period = this->declare_parameter<double>("timer_period", 0.5);
  global_frame_ = this->declare_parameter<std::string>("global_frame", "sim_world");

  planner_.configure(config);

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(timer_period)),
      std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  planner_.setMap(*msg);
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planAndPublish();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f)", goal_.point.x, goal_.point.y);
  planAndPublish();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  odom_received_ = true;
}

bool PlannerNode::goalReached() const {
  if (!odom_received_ || !goal_received_) return false;
  const double dx = goal_.point.x - robot_x_;
  const double dy = goal_.point.y - robot_y_;
  return std::hypot(dx, dy) < goal_tolerance_;
}

void PlannerNode::timerCallback() {
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) return;

  if (goalReached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    state_ = State::WAITING_FOR_GOAL;

    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = this->get_clock()->now();
    empty_path.header.frame_id = global_frame_;
    path_pub_->publish(empty_path);
    return;
  }

  planAndPublish();
}

void PlannerNode::planAndPublish() {
  if (!planner_.hasMap() || !odom_received_ || !goal_received_) return;

  const auto poses = planner_.planPath(robot_x_, robot_y_, goal_.point.x, goal_.point.y);
  if (poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "No path found from (%.2f, %.2f) to (%.2f, %.2f)",
                robot_x_, robot_y_, goal_.point.x, goal_.point.y);
    return;
  }

  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = this->get_clock()->now();
  path_msg.header.frame_id = global_frame_;
  path_msg.poses = poses;
  for (auto& pose : path_msg.poses) {
    pose.header = path_msg.header;
  }

  path_pub_->publish(path_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
