# mrpt_reactivenav2d

Pure reactive navigation for wheeled robots of arbitrary 2D shape, using the
TP-Space (Trajectory Parameter Space) algorithms in
[MRPT](https://github.com/MRPT/mrpt/).

## Demo

After installing [mrpt_navigation](https://index.ros.org/p/mrpt_navigation)
and [mvsim](https://index.ros.org/p/mvsim), run:

```bash
roslaunch mrpt_reactivenav2d reactive_nav_demo_with_mvsim.launch
```

It starts the mvsim simulator, `mrpt_local_obstacles` to build the obstacle
point cloud from the simulated lidar, this navigator, and RViz. Set navigation
goals with RViz "2D Nav Goal".

Video: [YouTube](https://www.youtube.com/watch?v=9iohyHqGc0g)

## Node: mrpt_reactivenav2d

### Subscribed topics

- `reactive_nav_goal` (`geometry_msgs/PoseStamped`): Navigation goal.
- `reactive_nav_waypoints` (`mrpt_msgs/WaypointSequence`): Sequence of
  waypoints to follow.
- `local_map_pointcloud` (`sensor_msgs/PointCloud2`): Nearby obstacles.
- `odom` (`nav_msgs/Odometry`): Robot odometry (velocity).
- `<topic_robot_shape>` (`geometry_msgs/Polygon`): Optional robot shape.

### Published topics

- `cmd_vel` (`geometry_msgs/Twist`): Velocity commands.

### Parameters

| Name | Default | Description |
| --- | --- | --- |
| `cfg_file_reactive` | (required) | INI file with the navigator, robot shape, and PTG parameters. See `tutorial/reactive2d_config.ini`. |
| `target_allowed_distance` | `0.40` | Distance [m] to consider a single goal reached. |
| `nav_period` | `0.1` | Navigation loop period [s]. |
| `frameid_reference` | `map` | Reference frame. |
| `frameid_robot` | `base_link` | Robot frame. |
| `topic_robot_shape` | (empty) | If set, topic to subscribe to for the robot shape. |
| `topic_wp_seq` | `reactive_nav_waypoints` | Waypoint sequence topic. |
| `topic_odometry` | `odom` | Odometry topic. |
| `topic_cmd_vel` | `cmd_vel` | Velocity command topic. |
| `topic_obstacles` | `local_map_pointcloud` | Obstacles topic. |
| `save_nav_log` | `false` | Save navigation logs for debugging. |
| `ptg_plugin_files` | (empty) | Optional shared libraries with additional PTG classes. |

## Theory and references

- J.L. Blanco, J. Gonzalez-Jimenez, J.A. Fernandez-Madrigal, "Extending
  Obstacle Avoidance Methods through Multiple Parameter-Space
  Transformations", Autonomous Robots, vol. 24, no. 1, pp. 29-48, 2008.
  ([DOI](https://doi.org/10.1007/s10514-007-9062-7),
  [Draft PDF](https://ingmec.ual.es/~jlblanco/papers/blanco2008eoa_DRAFT.pdf))
- M. Jaimez-Tarifa, J. Gonzalez-Jimenez, J.L. Blanco, "Efficient
  Reactive Navigation with Exact Collision Determination for 3D Robot
  Shapes", International Journal of Advanced Robotic Systems, vol. 12,
  no. 63, 2015. ([DOI](https://doi.org/10.5772/60563))
