#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <cstdint>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace robot
{

struct CostmapConfig {
  double resolution = 0.1;   // meters per cell
  int width = 300;           // cells
  int height = 300;          // cells
  double inflation_radius = 1.0;  // meters
  int8_t max_cost = 100;
};

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    void configure(const CostmapConfig& config);

    // Resets every cell to free space (0).
    void initializeCostmap();

    // Converts a polar laser reading into grid coordinates centered on the robot.
    // Returns false if the resulting cell falls outside the grid.
    bool rangeToGrid(double range, double angle, int& grid_x, int& grid_y) const;

    void markObstacle(int grid_x, int grid_y);

    void inflateObstacles();

    const std::vector<int8_t>& data() const { return grid_; }
    const CostmapConfig& config() const { return config_; }

  private:
    int index(int grid_x, int grid_y) const { return grid_y * config_.width + grid_x; }
    bool inBounds(int grid_x, int grid_y) const;

    rclcpp::Logger logger_;
    CostmapConfig config_;
    std::vector<int8_t> grid_;
};

}  // namespace robot

#endif
