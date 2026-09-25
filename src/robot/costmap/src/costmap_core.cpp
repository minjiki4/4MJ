#include "costmap_core.hpp"

#include <algorithm>
#include <cmath>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

void CostmapCore::configure(const CostmapConfig& config) {
  config_ = config;
  initializeCostmap();
}

void CostmapCore::initializeCostmap() {
  grid_.assign(static_cast<size_t>(config_.width) * config_.height, 0);
}

bool CostmapCore::inBounds(int grid_x, int grid_y) const {
  return grid_x >= 0 && grid_x < config_.width && grid_y >= 0 && grid_y < config_.height;
}

bool CostmapCore::rangeToGrid(double range, double angle, int& grid_x, int& grid_y) const {
  const double x = range * std::cos(angle);
  const double y = range * std::sin(angle);

  grid_x = static_cast<int>(std::floor(x / config_.resolution)) + config_.width / 2;
  grid_y = static_cast<int>(std::floor(y / config_.resolution)) + config_.height / 2;

  return inBounds(grid_x, grid_y);
}

void CostmapCore::markObstacle(int grid_x, int grid_y) {
  if (inBounds(grid_x, grid_y)) {
    grid_[index(grid_x, grid_y)] = config_.max_cost;
  }
}

void CostmapCore::inflateObstacles() {
  const int radius_cells = static_cast<int>(std::ceil(config_.inflation_radius / config_.resolution));

  // Snapshot the obstacle cells first so inflated cells never act as new sources.
  std::vector<std::pair<int, int>> obstacles;
  for (int y = 0; y < config_.height; ++y) {
    for (int x = 0; x < config_.width; ++x) {
      if (grid_[index(x, y)] == config_.max_cost) {
        obstacles.emplace_back(x, y);
      }
    }
  }

  for (const auto& [ox, oy] : obstacles) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        const int nx = ox + dx;
        const int ny = oy + dy;
        if (!inBounds(nx, ny)) continue;

        const double distance = std::hypot(dx, dy) * config_.resolution;
        if (distance > config_.inflation_radius) continue;

        const int8_t cost = static_cast<int8_t>(
            std::lround(config_.max_cost * (1.0 - distance / config_.inflation_radius)));

        int8_t& cell = grid_[index(nx, ny)];
        cell = std::max(cell, cost);
      }
    }
  }
}

}  // namespace robot
