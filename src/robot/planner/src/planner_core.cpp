#include "planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_set>

namespace robot
{

namespace {
constexpr int kNeighborOffsets[8][2] = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
}

PlannerCore::PlannerCore(const rclcpp::Logger& logger) : logger_(logger) {}

void PlannerCore::configure(const PlannerConfig& config) {
  config_ = config;
}

void PlannerCore::setMap(const nav_msgs::msg::OccupancyGrid& map) {
  map_ = map;
  has_map_ = true;
}

CellIndex PlannerCore::worldToGrid(double x, double y) const {
  const int grid_x = static_cast<int>(std::floor((x - map_.info.origin.position.x) / map_.info.resolution));
  const int grid_y = static_cast<int>(std::floor((y - map_.info.origin.position.y) / map_.info.resolution));
  return CellIndex(grid_x, grid_y);
}

geometry_msgs::msg::PoseStamped PlannerCore::gridToPose(const CellIndex& cell) const {
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = map_.info.origin.position.x + (cell.x + 0.5) * map_.info.resolution;
  pose.pose.position.y = map_.info.origin.position.y + (cell.y + 0.5) * map_.info.resolution;
  pose.pose.orientation.w = 1.0;
  return pose;
}

bool PlannerCore::isFree(const CellIndex& cell) const {
  if (cell.x < 0 || cell.x >= static_cast<int>(map_.info.width) ||
      cell.y < 0 || cell.y >= static_cast<int>(map_.info.height)) {
    return false;
  }
  const int8_t value = map_.data[cell.y * map_.info.width + cell.x];
  return value < 0 || value < config_.occupancy_threshold;
}

double PlannerCore::heuristic(const CellIndex& a, const CellIndex& b) const {
  return std::hypot(a.x - b.x, a.y - b.y);
}

std::vector<geometry_msgs::msg::PoseStamped> PlannerCore::planPath(double start_x, double start_y,
                                                                      double goal_x, double goal_y) const {
  std::vector<geometry_msgs::msg::PoseStamped> path;
  if (!has_map_) return path;

  const CellIndex start = worldToGrid(start_x, start_y);
  const CellIndex goal = worldToGrid(goal_x, goal_y);

  if (goal.x < 0 || goal.x >= static_cast<int>(map_.info.width) ||
      goal.y < 0 || goal.y >= static_cast<int>(map_.info.height)) {
    return path;
  }

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_set<CellIndex, CellIndexHash> closed;

  g_score[start] = 0.0;
  open_set.emplace(start, heuristic(start, goal));

  bool found = false;
  while (!open_set.empty()) {
    const CellIndex current = open_set.top().index;
    open_set.pop();

    if (current == goal) {
      found = true;
      break;
    }

    if (closed.count(current)) continue;
    closed.insert(current);

    for (const auto& offset : kNeighborOffsets) {
      const CellIndex neighbor(current.x + offset[0], current.y + offset[1]);
      if (!isFree(neighbor) || closed.count(neighbor)) continue;

      const double step_cost = std::hypot(offset[0], offset[1]);
      const double tentative_g = g_score[current] + step_cost;

      const auto it = g_score.find(neighbor);
      if (it == g_score.end() || tentative_g < it->second) {
        g_score[neighbor] = tentative_g;
        came_from[neighbor] = current;
        open_set.emplace(neighbor, tentative_g + heuristic(neighbor, goal));
      }
    }
  }

  if (!found) return path;

  std::vector<CellIndex> cells;
  CellIndex current = goal;
  while (!(current == start)) {
    cells.push_back(current);
    current = came_from.at(current);
  }
  cells.push_back(start);
  std::reverse(cells.begin(), cells.end());

  path.reserve(cells.size());
  for (const auto& cell : cells) {
    path.push_back(gridToPose(cell));
  }
  return path;
}

}
