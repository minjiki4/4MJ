# WATonomous ASD Admissions Assignment

A simulated differential-drive robot that navigates a bounded arena and avoids static obstacles, built on ROS 2 Humble and Gazebo. Full assignment description: https://wiki.watonomous.ca/

## Running

Requires Docker.

1. Set `ACTIVE_MODULES="robot gazebo vis_tools"` in `watod-config.local.sh`
2. `./watod up --build`
3. Publish a goal on `/goal_point` (`geometry_msgs/PointStamped`) and watch it navigate -- either in Foxglove (connect to `ws://localhost:<FOXGLOVE_BRIDGE_PORT>`) or via `ros2 topic echo /cmd_vel` / `/path` on the command line.

## Nodes

`src/robot/` has five nodes, wired together by `bringup_robot`:

- **costmap**: turns each `/lidar` scan into a local occupancy grid, marking hit cells and inflating cost around them within a configurable radius.
- **map_memory**: fuses successive local costmaps (transformed into the world frame using odometry) into a persistent global map, published on `/map`.
- **planner**: runs A* over the global occupancy grid from the current position to the last point received on `/goal_point`, replanning whenever the map updates.
- **control**: a pure pursuit controller that follows the planned `/path` and publishes `/cmd_vel`.
- **odometry_spoof**: republishes ground-truth TF as an `Odometry` message on `/odom/filtered`, standing in for a real localization stack.

## Parameter tuning

The starting parameter values (from the assignment's pseudo-code examples) were checked by running the simulation end-to-end, sending goals through `/goal_point`, and comparing the resulting trajectory against the known obstacle geometry:

- `costmap.inflation_radius`: `1.0 -> 1.6` m. At 1.0 m, the planner's "blocked" boundary sat exactly at the robot's own half-width, so planned paths grazed obstacles with close to zero clearance. 1.6 m restores a real margin without adding much path length.
- `planner.goal_tolerance`: `0.5 -> 0.3` m. The planner was clearing the path (and stopping the robot) well before control's own tighter tolerance ever mattered; tightening it gives a more precise stop.

Costmap/map resolution and grid size, map update distance, and control speeds were tested against several goals and obstacles and left at their defaults -- no smoothness, clearance, or timing issues showed up.
