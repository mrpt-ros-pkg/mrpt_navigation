# mrpt_tutorials

Launch files, parameters, maps, and datasets for the demos of the
[mrpt_navigation](https://github.com/mrpt-ros-pkg/mrpt_navigation) packages.
Most demos run in the [mvsim](https://github.com/MRPT/mvsim) simulator.

## Demos

Run any of them with `ros2 launch mrpt_tutorials <launch file>`:

| Launch file | Description |
| --- | --- |
| `demo_astar_trajectory_follower_gridmap.launch.py` | A* planning (`mrpt_tps_astar_planner`) + path following (`mrpt_trajectory_follower`) + localization (`mrpt_pf_localization`) on a 2D grid map. Arguments: `robot:=ackermann\|diffdrive`, `speed_limit`. |
| `demo_astar_trajectory_follower_street.launch.py` | Same as above, with a car-like robot in a street with parking bays. |
| `demo_astar_trajectory_follower_mvsim_3d.launch.py` | A* planning and path following with 3D LiDAR obstacles, in the mvsim `world:=greenhouse` or `world:=warehouse` worlds. |
| `demo_localization_pf_mvsim_2d_lidar.launch.py` | Particle filter localization (`mrpt_pf_localization`) with a 2D LiDAR. |
| `demo_pointcloud_pipeline_mvsim.launch.py` | Point cloud filtering (`mrpt_pointcloud_pipeline`) with a simulated Jackal robot. |
| `demo_composable_nodes.launch.py` | Map server, localization, and point cloud pipeline, optionally as composable nodes in one container (`use_composable`). |
| `demo_map_server_gridmap_from_yaml.launch.py` | `mrpt_map_server` publishing a grid map from a ROS `.yaml` map file. |
| `demo_map_server_from_mm.launch.py` | `mrpt_map_server` publishing the layers of an MRPT metric map (`.mm`) file (argument: `mm_file`). |
| `demo_astar_rnav_gridmap.launch.py` | Deprecated: like the first demo, using `mrpt_reactivenav2d` instead of the trajectory follower. |
| `demo_reactive_nav_mvsim.launch.py` | Deprecated: pure reactive navigation (`mrpt_reactivenav2d`). |

See the [repository README](../README.md) and the README of each package for
more details.

## Contents

- `launch/`: Demo launch files.
- `params/`: Parameter files for the planner, follower, localization, and
  point cloud pipeline.
- `maps/`: Grid maps, simple maps, and metric map files used by the demos.
- `mvsim/`: mvsim world files.
- `rviz2/`, `foxglove/`: Visualization layouts.
- `datasets/`: Small datasets used in unit tests.
