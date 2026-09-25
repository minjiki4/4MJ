# WATonomous ASD Admissions Assignment

## Prerequisite Installation
These steps are to setup the monorepo to work on your own PC. We utilize docker to enable ease of reproducibility and deployability.

> Why docker? It's so that you don't need to download any coding libraries on your bare metal pc, saving headache :3

1. This assignment is supported on Linux Ubuntu >= 22.04, Windows (WSL), and MacOS. This is standard practice that roboticists can't get around. To setup, you can either setup an [Ubuntu Virtual Machine](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview), setting up [WSL](https://learn.microsoft.com/en-us/windows/wsl/install), or setting up your computer to [dual boot](https://opensource.com/article/18/5/dual-boot-linux). You can find online resources for all three approaches.
2. Once inside Linux, [Download Docker Engine using the `apt` repository](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
3. You're all set! You can begin the assignment by visiting the WATonomous Wiki.

Link to Onboarding Assignment: https://wiki.watonomous.ca/

## Implementation

The section below covers the submitted implementation, separate from the assignment template above.

The robot stack (`src/robot/`) is split into four nodes that run together via `bringup_robot`:

- **costmap**: converts each `/lidar` scan into a local occupancy grid, marking obstacle cells and inflating costs around them within a configurable radius.
- **map_memory**: fuses successive local costmaps (transformed into the world frame using odometry) into a persistent global map, published on `/map`.
- **planner**: runs A* over the global occupancy grid from the robot's current position to the last `/goal_point` received, replanning whenever the map updates.
- **control**: a pure pursuit controller that follows the planned `/path` and publishes `/cmd_vel`.

### Parameter tuning

The default parameters (based on the assignment's pseudo-code examples) were verified by running the full simulation end-to-end (`./watod up`), sending goals via `/goal_point`, and checking the robot's actual trajectory against the known obstacle geometry:

- `costmap.inflation_radius` (`1.0 → 1.6` m): the original value put the planner's "blocked" boundary exactly at the robot's own half-width, so the robot grazed obstacles with near-zero clearance. Increasing it restored a real safety margin without noticeably lengthening paths.
- `planner.goal_tolerance` (`0.5 → 0.3` m): the planner was clearing the path (and stopping the robot) well before the controller's own, tighter tolerance ever mattered. Tightening it makes the robot stop closer to the requested point while still avoiding a planner/controller deadlock.

Other parameters (costmap/map resolution and grid size, map update distance, control speeds) were tested against multiple goals and obstacles and left at their defaults, since no smoothness, clearance, or timing issues were observed.
