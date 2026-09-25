#include <chrono>
#include <cmath>

#include "geometry_msgs/msg/quaternion.hpp"
#include "map_memory_node.hpp"

namespace {
double quaternionToYaw(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  robot::MapMemoryConfig config;
  config.resolution = this->declare_parameter<double>("resolution", config.resolution);
  config.width = this->declare_parameter<int>("width", config.width);
  config.height = this->declare_parameter<int>("height", config.height);
  config.update_distance = this->declare_parameter<double>("update_distance", config.update_distance);
  const double publish_rate = this->declare_parameter<double>("publish_rate", 1.0);
  global_frame_ = this->declare_parameter<std::string>("global_frame", "sim_world");

  map_memory_.configure(config);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  const auto period = std::chrono::duration<double>(1.0 / publish_rate);
  timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = msg;
  costmap_received_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  robot_yaw_ = quaternionToYaw(msg->pose.pose.orientation);
  odom_received_ = true;
}

void MapMemoryNode::updateMap() {
  if (!costmap_received_ || !odom_received_) return;

  const double dx = robot_x_ - last_fused_x_;
  const double dy = robot_y_ - last_fused_y_;
  const double distance = std::hypot(dx, dy);

  if (!has_fused_once_ || distance >= map_memory_.config().update_distance) {
    map_memory_.fuseCostmap(*latest_costmap_, robot_x_, robot_y_, robot_yaw_);
    last_fused_x_ = robot_x_;
    last_fused_y_ = robot_y_;
    has_fused_once_ = true;
  }

  auto map_msg = map_memory_.buildOccupancyGrid(this->get_clock()->now(), global_frame_);
  map_pub_->publish(map_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
