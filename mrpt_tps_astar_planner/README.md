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

At startup the PTG lookup tables are built in the background, which can take
tens of seconds. The planning services and the `navigate_goal` action are only
offered once they are ready, so clients see them as unavailable (and can wait
for them) instead of getting a failed plan. Topic goals received meanwhile are
rejected with a warning in the log (`PTGs initialized` marks when the node is
ready).

The A\* implementation is an anytime algorithm: it improves the solution while
time allows, then returns the best found. An optional refinement pass
(`astar_skip_refine: false`) further smooths the result.

### Navigation goals: anytime and continuous planning

Goals from `<topic_goal_sub>` (e.g. clicked in RViz/Foxglove) or the
`~/navigate_goal` action are handled by a goal manager running in a background
thread, so the robot starts moving soon and reacts to obstacles in advance:

1. **Anytime:** if the search has not reached the goal after
   `anytime_first_path_time`, the best partial path so far (the tree node
   closest to the goal, if it gets at least `anytime_min_progress` closer) is
   published, and the robot starts following it while the search goes on.
2. **Continuous:** every `replan_period`, the search root moves to a node
   `stitch_lookahead_time` ahead of the robot along its path. The path up to
   it is *committed* (it never changes), and the search grows the same tree from
   there with the latest obstacles until it reaches the goal. Path updates keep
   the committed part, so the follower switches to them without slowing down.
3. Each cycle, the rest of the path is checked against the current obstacles
   (robot footprint). If it is blocked, the path is cut short before the
   obstacle (the follower stops there if needed) and a new search starts from
   scratch: from the end of the committed part, or from the robot if the
   obstacle is before it (cutting the path there, so the robot does not drive
   past it meanwhile). Follower failures (`Blocked`, `OffPathExceeded`,
   `MissedGoal`, or staying `Blocked` for 5 s) restart it from the robot pose.
   A search that ends with no path to follow (no motion possible from the
   robot pose, or no way with the current obstacles, maybe transient) is
   retried every 2 s. Obstacle points that blocked a path of the goal are
   kept in its searches for `blocked_obstacles_memory`, so it does not plan
   through them again as soon as they are out of sight.
4. New searches from the robot (a new goal while driving, or a restart) start
   from where it will be after `start_lookahead_time` on the path it is
   following (at its current speed, from TF), and keep that path up to there,
   so a fast robot is not already past the start of the new path when it is
   published, and the follower switches to it without stopping.
5. The goal ends when the follower (`topic_follower_status_sub`) reaches it
   after following a full path. A new goal preempts the current one. Goals
   whose footprint hits the current obstacles are rejected at once. A goal
   fails when the robot does not get closer to it for `no_progress_timeout`
   (in a straight line, or along the full path it follows), however many
   times it was replanned meanwhile (`max_planning_restarts`, if set, also
   limits the restarts in a row).

The goal manager's anytime and continuous planning require
`mrpt_trajectory_follower`: goals end with its status, and a path follower
that does not report one (e.g. the deprecated `mrpt_reactivenav2d` fed with the waypoint
sequence) never ends them. The launch file therefore disables both by default
(one-shot plans), and the trajectory follower demos enable them. Paths cut short
because of an obstacle are published on every path topic.

Deployments with a path follower that publishes only the legacy status topic
(not `status_ex`) must set `topic_follower_status_ex_sub` to an empty string, so
the planner uses `topic_follower_status_sub` instead; otherwise goals never
receive their completion status (this also applies to `NavigateGoal` actions).
Both topics are not used at once, since that would defeat the goal id and
revision filtering.

Paths are published as `mrpt_nav_interfaces/ReferencePath` (for
`mrpt_trajectory_follower`), and also as before (`WaypointSequence`, `Path`).
With `continuous_planning: false` and `anytime_first_path_time: 0`, topic goals
are planned once, as one-shot plans. The services are always one-shot.

Each planning thread uses its own copy of the PTGs, so services and the goal
manager plan concurrently.

### Navigation status and feedback

Whatever the source of a goal (topic or action), the planner publishes a
`mrpt_nav_interfaces/NavigationStatus` on `~/navigation_status` (latched, on
every change and at 2 Hz): the planner state (`INITIALIZING` with the PTG build
progress, `INIT_FAILED`, `READY`, `PLANNING`, `FOLLOWING_PARTIAL`, `FOLLOWING`,
`REPLANNING`, `BLOCKED`, `STALLED`, and `SUCCEEDED`, `FAILED` or `CANCELED` for
a few seconds once a goal ends), a severity `level` to color views, a stable
`reason` code, a one-line `message` and the measurements of the goal (distance,
remaining path length, a rough ETA, time without progress, replans). Topic goals
received while the PTGs are being built are queued (the latest one) instead of
dropped. A goal that makes no progress for `no_progress_warn_fraction` of
`no_progress_timeout` turns the state to `STALLED` with the likely cause in
`reason`, and a warning is logged, before it is given up. The action feedback
carries the same state name and measurements, and the action result a `reason`
and `message` telling why it ended.

`~/navigation_markers` (`visualization_msgs/MarkerArray`, latched, disabled with
`publish_markers: false`) shows it in RViz and Foxglove 3D views: a text above
the robot colored by severity (namespace `status_text`), the goal pose and
footprint colored by state (`goal`), the obstacle points that blocked a path
and where it got blocked (`blocking`), and where a partial path ends
(`partial_end`). Each namespace can be toggled. The tutorial RViz and Foxglove
layouts include them.

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
| `reverse_motion_cost_factor` | `1.0` | Extra cost per second driven in reverse, so paths only back up when it pays off (0: same cost as forward) |
| `turning_cost_factor` | `1.0` | Extra cost per radian of heading change, so paths drive straight instead of weaving, unless turning pays off (0: no penalty; needs a `mrpt_path_planning` with `MPP_HAS_TURNING_COST`) |
| `max_computation_time` | `0.0` | Maximum A\* computation time per plan request [s]; `<=0` uses `maximumComputationTime` from `planner_parameters` |
| `mid_waypoints_allowed_distance` | `0.5` | Acceptance radius [m] for intermediate waypoints |
| `final_waypoint_allowed_distance` | `0.4` | Acceptance radius [m] for the goal waypoint |
| `mid_waypoints_allow_skip` | `true` | Whether intermediate waypoints may be skipped |
| `final_waypoint_allow_skip` | `false` | Whether the final waypoint may be skipped |
| `mid_waypoints_ignore_heading` | `false` | Whether heading is ignored at intermediate waypoints |
| `final_waypoint_ignore_heading` | `false` | Whether heading is ignored at the final waypoint |
| `anytime_first_path_time` | `0.0` (launch file) | Publish the best partial path after this time [s] if the goal is not reached yet (<=0: only full paths) |
| `anytime_min_progress` | `1.0` | Minimum progress toward the goal [m] of a partial path |
| `anytime_max_partial_length` | `10.0` | Partial paths are cut to this length [m], so a dead end (the node closest to the goal may be one) does not take the robot far before the search finds the way around (`<=0`: no limit) |
| `continuous_planning` | `false` (launch file) | Keep replanning while driving (see above) |
| `replan_period` | `1.0` | Period of the replanning cycles [s] |
| `stitch_lookahead_time` | `3.0` | The search goes on from this far ahead [s] of the robot (at max PTG speed) |
| `start_lookahead_time` | `3.0` | New searches from the robot (new goals, restarts) start this far ahead [s], at its current speed, on the path it is following (<=0: from the robot pose) |
| `no_progress_timeout` | `60.0` | A goal fails if the robot does not get closer to it for this long [s], in a straight line or along the full path it follows; meanwhile, blocked paths and failed searches are replanned (`<=0`: never) |
| `no_progress_warn_fraction` | `0.5` | The status turns to `STALLED` (and a warning is logged) after this fraction of `no_progress_timeout` without progress (`<=0`: never) |
| `publish_markers` | `true` | Publish `~/navigation_markers`, to show the navigation status in 3D views |
| `blocked_obstacles_memory` | `30.0` | Obstacle points that blocked a path of a goal are kept in its searches for this long [s], also when out of sight (`<=0`: not kept) |
| `max_planning_restarts` | `0` | If `>0`, a goal also fails after this many searches restarted from scratch in a row without reaching it (`<=0`: no limit) |
| `topic_follower_status_sub` | `/mrpt_trajectory_follower/status` | Path follower status (goal reached or failed), used only if the next one is empty |
| `topic_follower_status_ex_sub` | `/mrpt_trajectory_follower/status_ex` | Path follower status with the goal id and path revision it refers to: only statuses of the latest published path are acted on |

### Subscribed topics

| Topic | Type | Description |
|---|---|---|
| `<topic_goal_sub>` | `geometry_msgs/PoseStamped` | Goal pose (preempts the current goal); planned from the current TF robot pose |
| `<topic_follower_status_sub>` | `std_msgs/String` | Path follower status |
| `<topic_obstacles_gridmap_sub>` (one per entry) | `nav_msgs/OccupancyGrid` | Occupancy grid(s) used as static obstacles |
| `<topic_obstacles_sub>` (one per entry) | `sensor_msgs/PointCloud2` | Point cloud(s) used as dynamic obstacles |

### Published topics

| Topic | Type | Description |
|---|---|---|
| `<topic_wp_seq_pub>` (default `/waypoints`) | `mrpt_msgs/WaypointSequence` | Full waypoint sequence with per-waypoint tolerances and flags |
| `<topic_wp_seq_pub>_path` (default `/waypoints_path`) | `nav_msgs/Path` | Same path as `nav_msgs/Path` (empty if no path was found for a goal topic request, which cancels path followers) |
| `<topic_wp_seq_pub>_reference` (default `/waypoints_reference`) | `mrpt_nav_interfaces/ReferencePath` | Goal manager paths, with goal id, revision and whether they are partial (empty: navigation canceled) |
| `~/committed_path`, `~/tentative_path` | `nav_msgs/Path` | Goal manager path, split into its committed part and the rest (latched; shown in different colors in the tutorial RViz/Foxglove layouts) |
| `~/navigation_status` | `mrpt_nav_interfaces/NavigationStatus` | Planner readiness and state of the current goal, with measurements and a reason code (latched; see above) |
| `~/navigation_markers` | `visualization_msgs/MarkerArray` | The status as markers for 3D views (latched; see above) |
| `~/robot_shape` | `geometry_msgs/PolygonStamped` | Robot footprint used for planning (latched), so other nodes can check they use the same robot description |
| `<topic_costmaps_pub>_0`, `_1`, … | `nav_msgs/OccupancyGrid` | Inflated costmaps (one per obstacle source, latched), published after each one-shot plan and whenever the goal manager rebuilds them (converted only once subscribed; late subscribers get the latest one) |

### Actions

| Action | Type | Description |
|---|---|---|
| `~/navigate_goal` | `mrpt_nav_interfaces/NavigateGoal` | Navigate to a goal (as the goal topic), with feedback (state, distance to goal, remaining path length, ETA, time without progress) and a result with a `reason` once the follower reaches it or navigation fails. Can be canceled |

### Services

| Service | Type | Description |
|---|---|---|
| `<node_fqn>/make_plan_to` | `mrpt_nav_interfaces/MakePlanTo` | Plan from the current robot TF pose to a given goal `PoseStamped`; returns `valid_path_found` and `waypoints` |
| `<node_fqn>/make_plan_from_to` | `mrpt_nav_interfaces/MakePlanFromTo` | Plan between two explicitly given `Pose` values; no TF lookup needed |

Goal frames: goals on `<topic_goal_sub>` and `make_plan_to` targets
(`PoseStamped`) are transformed with TF into `frame_id_map`, at their stamp if
available, else with the latest transform. So goals can be given e.g. in the
robot frame, or clicked in an RViz/Foxglove view whose fixed frame is not the
map. A goal with an empty `frame_id` is taken as already in `frame_id_map`, and
one whose frame cannot be transformed is rejected (no plan). `make_plan_from_to`
poses carry no frame and are always in `frame_id_map`.

Both requests take an optional `ignore_obstacles_in` list of polygons (XY, map
frame): obstacle points inside them, from every source, are ignored for that
plan only (e.g. where obstacles are known to be traversable).

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

### Running as a composable node

The planner is also available as the component
`mrpt_tps_astar_planner::TPS_Astar_Planner_Node`. With the same launch file:

```bash
ros2 run rclcpp_components component_container_mt --ros-args -r __node:=nav_container
ros2 launch mrpt_tps_astar_planner tps_astar_planner.launch.py \
    use_composable:=true container_name:=/nav_container ptg_ini:=/path/to/ptgs.ini
```

Use a multi-threaded container (`component_container_mt`) so planning service
calls can run concurrently, as in the standalone node.
