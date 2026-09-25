#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot
{

// ------------------- Supporting Structures -------------------

// 2D grid index
struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};

// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    return a.f_score > b.f_score;
  }
};

struct PlannerConfig {
  int occupancy_threshold = 50;  // cost >= this value is treated as an obstacle
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    void configure(const PlannerConfig& config);

    void setMap(const nav_msgs::msg::OccupancyGrid& map);
    bool hasMap() const { return has_map_; }

    // Runs A* from (start_x, start_y) to (goal_x, goal_y) in world coordinates.
    // Returns an empty vector if no map is available or no path exists.
    std::vector<geometry_msgs::msg::PoseStamped> planPath(double start_x, double start_y,
                                                             double goal_x, double goal_y) const;

  private:
    CellIndex worldToGrid(double x, double y) const;
    geometry_msgs::msg::PoseStamped gridToPose(const CellIndex& cell) const;
    bool isFree(const CellIndex& cell) const;
    double heuristic(const CellIndex& a, const CellIndex& b) const;

    rclcpp::Logger logger_;
    PlannerConfig config_;

    nav_msgs::msg::OccupancyGrid map_;
    bool has_map_ = false;
};

}  // namespace robot

#endif
