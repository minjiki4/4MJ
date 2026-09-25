#include "map_memory_core.hpp"

#include <cmath>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) : logger_(logger) {}

void MapMemoryCore::configure(const MapMemoryConfig& config) {
  config_ = config;
  grid_.assign(static_cast<size_t>(config_.width) * config_.height, 0);
}

bool MapMemoryCore::worldToGrid(double x, double y, int& grid_x, int& grid_y) const {
  const double origin_x = -config_.width / 2.0 * config_.resolution;
  const double origin_y = -config_.height / 2.0 * config_.resolution;

  grid_x = static_cast<int>(std::floor((x - origin_x) / config_.resolution));
  grid_y = static_cast<int>(std::floor((y - origin_y) / config_.resolution));

  return grid_x >= 0 && grid_x < config_.width && grid_y >= 0 && grid_y < config_.height;
}

void MapMemoryCore::fuseCostmap(const nav_msgs::msg::OccupancyGrid& local_costmap,
                                 double robot_x, double robot_y, double robot_yaw) {
  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);
  const auto& info = local_costmap.info;

  for (int row = 0; row < static_cast<int>(info.height); ++row) {
    for (int col = 0; col < static_cast<int>(info.width); ++col) {
      const int8_t value = local_costmap.data[row * info.width + col];
      if (value < 0) continue;

      const double local_x = info.origin.position.x + (col + 0.5) * info.resolution;
      const double local_y = info.origin.position.y + (row + 0.5) * info.resolution;

      const double global_x = robot_x + local_x * cos_yaw - local_y * sin_yaw;
      const double global_y = robot_y + local_x * sin_yaw + local_y * cos_yaw;

      int grid_x, grid_y;
      if (worldToGrid(global_x, global_y, grid_x, grid_y)) {
        grid_[index(grid_x, grid_y)] = value;
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::buildOccupancyGrid(const rclcpp::Time& stamp,
                                                                  const std::string& frame_id) const {
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;

  msg.info.resolution = static_cast<float>(config_.resolution);
  msg.info.width = config_.width;
  msg.info.height = config_.height;
  msg.info.origin.position.x = -config_.width / 2.0 * config_.resolution;
  msg.info.origin.position.y = -config_.height / 2.0 * config_.resolution;
  msg.info.origin.orientation.w = 1.0;

  msg.data = grid_;
  return msg;
}

}
