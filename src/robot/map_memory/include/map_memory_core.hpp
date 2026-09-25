#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <cstdint>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot
{

struct MapMemoryConfig {
  double resolution = 0.1;
  int width = 300;
  int height = 300;
  double update_distance = 1.5;
};

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void configure(const MapMemoryConfig& config);

    void fuseCostmap(const nav_msgs::msg::OccupancyGrid& local_costmap,
                      double robot_x, double robot_y, double robot_yaw);

    nav_msgs::msg::OccupancyGrid buildOccupancyGrid(const rclcpp::Time& stamp,
                                                      const std::string& frame_id) const;

    const MapMemoryConfig& config() const { return config_; }

  private:
    bool worldToGrid(double x, double y, int& grid_x, int& grid_y) const;
    int index(int grid_x, int grid_y) const { return grid_y * config_.width + grid_x; }

    rclcpp::Logger logger_;
    MapMemoryConfig config_;
    std::vector<int8_t> grid_;
};

}

#endif
