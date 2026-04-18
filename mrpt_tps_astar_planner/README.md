# mrpt_tps_astar_planner

## Overview

This package provides a ROS 2 node that uses the PTG-based A\* planner from
`mrpt_path_planning` to compute collision-free waypoint sequences for a
non-holonomic robot, respecting its real shape, orientation, and kinematic
constraints.

Planning is performed on a SE(2) lattice using Parameterized Trajectory
Generator (PTG) families that encode the robot's motion primitives. The result
is published as a `mrpt_msgs/WaypointSequence` (and a `nav_msgs/Path` for
visualization) and/or returned as the response of a ROS 2 service call.

The node supports **concurrent service requests**: each executor thread owns a
lazily-initialized planner instance, so multiple clients can request plans
simultaneously without blocking each other.

## How to cite

<details>
    TBD!
</details>


## Configuration

Key configuration files passed as ROS 2 parameters:

| Parameter | Description |
|---|---|
| `ptg_ini` | INI file describing PTG families (robot kinematics) |
| `planner_parameters` | YAML file with A\* algorithm parameters |
| `global_costmap_parameters` | YAML file for costmap obstacle-inflation parameters |
| `prefer_waypoints_parameters` | YAML file for waypoint-preference cost weights |


## Demos

See the `path-planner-sandbox/` subdirectory for standalone test scripts and
sample maps.


## Node: `mrpt_tps_astar_planner_node`

### Working rationale

1. Obstacle data are maintained from subscribed gridmaps and/or point-cloud
   topics (updated asynchronously, protected by a mutex).
2. On each planning request (topic goal or service call) the node snapshots the
   current obstacle data, builds cost evaluators, and runs the A\* planner.
3. The planned path is interpolated at a fixed time step and converted to a
   `WaypointSequence`.

The A\* implementation is an anytime algorithm: it improves the solution while
time allows, then returns the best found. An optional refinement pass
(`astar_skip_refine: false`) further smooths the result.

### ROS 2 parameters

| Parameter | Default | Description |
|---|---|---|
| `show_gui` | `false` | Open an MRPT 3D window showing maps and the planned path |
| `frame_id_map` | `map` | TF frame of the global map |
| `frame_id_robot` | `base_link` | TF frame of the robot |
| `topic_goal_sub` | `tps_astar_nav_goal` | `geometry_msgs/PoseStamped` goal subscription |
| `topic_obstacles_gridmap_sub` | _(empty)_ | Comma-separated occupancy-grid topics for obstacles |
| `topic_obstacles_sub` | _(empty)_ | Comma-separated `PointCloud2` topics for obstacles |
| `topic_static_maps` | `/map` | Subset of the above topics to subscribe with transient-local QoS |
| `topic_wp_seq_pub` | `/waypoints` | Topic on which to publish the resulting waypoint sequence |
| `topic_costmaps_pub` | `/costmap` | Prefix for costmap debug publishers |
| `ptg_ini` | _(required)_ | Path to PTG `.ini` file |
| `planner_parameters` | _(required)_ | Path to planner YAML file |
| `global_costmap_parameters` | _(required)_ | Path to costmap YAML file |
| `prefer_waypoints_parameters` | _(required)_ | Path to waypoint-preference YAML file |
| `problem_world_bbox_margin` | `2.0` | Extra margin [m] added around the planning bounding box |
| `problem_world_bbox_ignore_obstacles` | `false` | If true, obstacle extents are excluded from the bounding box |
| `astar_skip_refine` | `false` | If true, skip the post-A\* trajectory refinement pass |
| `mid_waypoints_allowed_distance` | `0.5` | Acceptance radius [m] for intermediate waypoints |
| `final_waypoint_allowed_distance` | `0.4` | Acceptance radius [m] for the goal waypoint |
| `mid_waypoints_allow_skip` | `true` | Whether intermediate waypoints may be skipped |
| `final_waypoint_allow_skip` | `false` | Whether the final waypoint may be skipped |
| `mid_waypoints_ignore_heading` | `false` | Whether heading is ignored at intermediate waypoints |
| `final_waypoint_ignore_heading` | `false` | Whether heading is ignored at the final waypoint |

### Subscribed topics

| Topic | Type | Description |
|---|---|---|
| `<topic_goal_sub>` | `geometry_msgs/PoseStamped` | Goal pose; triggers a plan from current TF robot pose |
| `<topic_obstacles_gridmap_sub>` (one per entry) | `nav_msgs/OccupancyGrid` | Occupancy grid(s) used as static obstacles |
| `<topic_obstacles_sub>` (one per entry) | `sensor_msgs/PointCloud2` | Point cloud(s) used as dynamic obstacles |

### Published topics

| Topic | Type | Description |
|---|---|---|
| `<topic_wp_seq_pub>` (default `/waypoints`) | `mrpt_msgs/WaypointSequence` | Full waypoint sequence with per-waypoint tolerances and flags |
| `<topic_wp_seq_pub>_path` (default `/waypoints_path`) | `nav_msgs/Path` | Same path as `nav_msgs/Path`, mainly for RViz visualization |
| `<topic_costmaps_pub>_0`, `_1`, … | `nav_msgs/OccupancyGrid` | Inflated costmaps (one per obstacle source), published after each plan |

### Services

| Service | Type | Description |
|---|---|---|
| `<node_fqn>/make_plan_to` | `mrpt_nav_interfaces/MakePlanTo` | Plan from the current robot TF pose to a given goal `PoseStamped`; returns `valid_path_found` and `waypoints` |
| `<node_fqn>/make_plan_from_to` | `mrpt_nav_interfaces/MakePlanFromTo` | Plan between two explicitly given `Pose` values; no TF lookup needed |

Both services are registered in a **reentrant callback group** so multiple
clients can be served simultaneously by the `MultiThreadedExecutor`.

### Template ROS 2 launch file

```bash
ros2 launch mrpt_tps_astar_planner tps_astar_planner.launch.py \
    ptg_ini:=/path/to/ptgs.ini \
    planner_parameters:=/path/to/planner-params.yaml \
    global_costmap_parameters:=/path/to/costmap-obstacles.yaml \
    prefer_waypoints_parameters:=/path/to/costmap-prefer-waypoints.yaml \
    topic_obstacles_gridmap_sub:=/map \
    topic_static_maps:=/map
```

All parameters have defaults in the launch file; only the config-file paths
are typically required to be overridden for a real deployment.
