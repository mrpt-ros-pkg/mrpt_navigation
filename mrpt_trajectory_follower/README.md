# mrpt_trajectory_follower

## Overview

This package provides a ROS 2 node (also available as a composable node) that
makes a wheeled robot (differential drive or Ackermann) follow a path, usually
one computed by [mrpt_tps_astar_planner](../mrpt_tps_astar_planner). It is a
wrapper of `mpp::TrajectoryFollower` from
[mrpt_path_planning](https://github.com/MRPT/mrpt_path_planning):

- Curvature-adaptive pure-pursuit tracking, with a speed profile that respects
  the robot kinematic limits (speed, acceleration, lateral acceleration,
  angular speed).
- Predictive safety: if a footprint and an obstacle source are available, the
  commanded motion is forecast, and the robot slows down or stops before any
  contact. Staying stopped longer than `block_timeout` reports `Blocked`.
- An optional last-resort collision guard (`collision_guard`), applied to
  every command using only the latest obstacles in the robot frame: the speed
  is limited so the robot can always brake before touching an obstacle, and
  missing or stale obstacle data stop the robot.
- Optional replanning on failure (off the path, blocked), by calling the
  planner `make_plan_to` service (`replan_on_failure`). Paths received as
  `ReferencePath` are not replanned here: their planner does it, watching
  this node status.
- A run-time speed limit (`speed_limit` parameter or `~/speed_limit` topic).

The robot footprint is taken from the PTG INI file (`ptg_ini`, the same file
used by the planner), or from `robot_radius` as a fallback. If it differs
from the footprint published by the planner, paths are rejected (status
`Failed`).

## Demos

    ros2 launch mrpt_tutorials demo_astar_trajectory_follower_gridmap.launch.py \
        robot:=ackermann speed_limit:=1.0

See the [repository README](../README.md) and
[mrpt_tutorials](../mrpt_tutorials) for this and the other demos
(`demo_astar_trajectory_follower_street.launch.py`,
`demo_astar_trajectory_follower_mvsim_3d.launch.py`).

## Node: mrpt_trajectory_follower_node

### Subscribed topics

- `<topic_path_sub>` (`nav_msgs/Path`): path to follow. An empty path cancels
  the current navigation.
- `<topic_reference_path_sub>` (`mrpt_nav_interfaces/ReferencePath`): path
  from a planner that keeps updating it while the robot moves (goal id,
  revision, and whether it is partial). Disabled if empty.
- `<topic_odom_sub>` (`nav_msgs/Odometry`): robot odometry.
- `<topic_obstacles_sub>` (`sensor_msgs/PointCloud2`): obstacles, e.g. from
  [mrpt_pointcloud_pipeline](../mrpt_pointcloud_pipeline). Disabled if empty.
- `<topic_robot_shape_sub>` (`geometry_msgs/PolygonStamped`): the planner
  footprint, to check it matches this node one. Disabled if empty.
- `~/speed_limit` (`std_msgs/Float64`): run-time speed limit [m/s] (<=0: none).
- `/tf`: robot pose in `frame_id_map`.

### Published topics

- `<topic_cmd_vel_pub>` (`geometry_msgs/Twist`, or `geometry_msgs/TwistStamped`
  in `frame_id_robot` if `cmd_vel_stamped` is true): velocity commands.
- `~/status` (`std_msgs/String`, latched): follower status: `Idle`, `Running`,
  `ReachedGoal`, `Blocked`, `OffPathExceeded`, `MissedGoal`, `WaitingForPath`,
  `Replanning`, `Canceled`, or `Failed`.
- `~/status_ex` (`mrpt_nav_interfaces/FollowerStatus`, latched): the same
  status, with the goal id and revision of the path it refers to.
- `~/predicted_trajectory` (`nav_msgs/Path`): the predicted short-term motion.
- `~/reference_path` (`nav_msgs/Path`, latched): the path being followed.

### Service clients

- `<planner_service>` (`mrpt_nav_interfaces/MakePlanTo`): only if
  `replan_on_failure` is true.

### Parameters

| Name | Default | Description |
| --- | --- | --- |
| `frame_id_map` | `"map"` | Map frame. |
| `frame_id_robot` | `"base_link"` | Robot frame. |
| `topic_path_sub` | `"/waypoints_path"` | Input path topic. |
| `topic_reference_path_sub` | `"/waypoints_reference"` | Input `ReferencePath` topic. |
| `topic_odom_sub` | `"/odom"` | Odometry topic. |
| `topic_obstacles_sub` | `""` | Obstacles topic. |
| `topic_cmd_vel_pub` | `"/cmd_vel"` | Output velocity command topic. |
| `cmd_vel_stamped` | `false` | Publish `TwistStamped` instead of `Twist`. |
| `ptg_ini` | `""` | PTG INI file, to take the robot footprint from. |
| `robot_radius` | `0.0` | Circular footprint [m], if no `ptg_ini` is given. |
| `follower_parameters` | `""` | YAML file with the follower parameters (see below). |
| `follower_parameters_overrides` | `""` | YAML file overriding some of them, e.g. per robot. |
| `speed_limit` | `0.0` | Run-time speed limit [m/s] (<=0: none). Can be changed with `ros2 param set`. |
| `obstacle_z_min`, `obstacle_z_max` | `-1e6`, `1e6` | Height band [m] (robot frame) of obstacle points, e.g. to use a raw 3D LiDAR. |
| `self_filter_radius` | `0.0` | Drop obstacle points within this radius [m] of the robot (<=0: disabled). |
| `collision_guard` | `true` | Enable the last-resort collision guard (needs `topic_obstacles_sub`). |
| `replan_on_failure` | `false` | Request a new plan when off the path or blocked. |
| `max_replan_attempts` | `3` | Max replans since the last externally given path. |
| `planner_service` | `"/mrpt_tps_astar_planner_node/make_plan_to"` | Planner service for replanning. |
| `topic_robot_shape_sub` | `"/mrpt_tps_astar_planner_node/robot_shape"` | Planner footprint topic. |
| `follower_debug_trace` | `false` | Print the follower internal per-cycle trace. |

The follower itself (kinematic limits, lookahead, goal tolerances, predictive
safety, collision guard, control period, etc.) is configured in the
`follower_parameters` YAML file. See the commented defaults in
[configs/params/follower-params.yaml](configs/params/follower-params.yaml), and
the per-robot overrides in `mrpt_tutorials/params/follower-overrides-*.yaml`.

## Template ROS 2 launch files

This package provides [launch/trajectory_follower.launch.py](launch/trajectory_follower.launch.py):

    ros2 launch mrpt_trajectory_follower trajectory_follower.launch.py \
        ptg_ini:=<path-to-ptgs.ini> topic_obstacles_sub:=/local_map_pointcloud

Its launch arguments have the same names and defaults as the node parameters
above (except `follower_debug_trace`, not exposed), and `follower_parameters`
defaults to the package `configs/params/follower-params.yaml`. Use
`use_composable:=true container_name:=<name>` to load it into an existing
component container, and `log_level` to change the log level.
