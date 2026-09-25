#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  robot::CostmapConfig config;
  config.resolution = this->declare_parameter<double>("resolution", config.resolution);
  config.width = this->declare_parameter<int>("width", config.width);
  config.height = this->declare_parameter<int>("height", config.height);
  config.inflation_radius = this->declare_parameter<double>("inflation_radius", config.inflation_radius);
  config.max_cost = static_cast<int8_t>(this->declare_parameter<int>("max_cost", config.max_cost));
  robot_frame_ = this->declare_parameter<std::string>("robot_frame", "robot/chassis/lidar");

  costmap_.configure(config);

  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  costmap_.initializeCostmap();

  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    const double range = msg->ranges[i];
    if (range < msg->range_min || range > msg->range_max) continue;

    const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
    int grid_x, grid_y;
    if (costmap_.rangeToGrid(range, angle, grid_x, grid_y)) {
      costmap_.markObstacle(grid_x, grid_y);
    }
  }

  costmap_.inflateObstacles();

  const auto& config = costmap_.config();

  nav_msgs::msg::OccupancyGrid grid_msg;
  grid_msg.header.stamp = msg->header.stamp;
  grid_msg.header.frame_id = robot_frame_;

  grid_msg.info.resolution = static_cast<float>(config.resolution);
  grid_msg.info.width = config.width;
  grid_msg.info.height = config.height;
  grid_msg.info.origin.position.x = -config.width / 2.0 * config.resolution;
  grid_msg.info.origin.position.y = -config.height / 2.0 * config.resolution;
  grid_msg.info.origin.position.z = 0.0;
  grid_msg.info.origin.orientation.w = 1.0;

  grid_msg.data = costmap_.data();

  costmap_pub_->publish(grid_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
