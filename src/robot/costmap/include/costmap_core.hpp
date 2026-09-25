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
    explicit CostmapCore(const rclcpp::Logger& logger);

    void configure(const CostmapConfig& config);

    void initializeCostmap();

    // false if (grid_x, grid_y) lands off the grid
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
