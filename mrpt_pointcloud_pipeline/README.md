# mrpt_pointcloud_pipeline

## Overview

This package provides a ROS 2 node (also available as a composable node,
`LocalObstaclesNode`) that builds a local obstacle point cloud around the robot
from recent 2D laser scans and/or point clouds. Typical uses are feeding the
obstacles of a planner or path follower, filtering a 3D LiDAR by a volume or
height band, and downsampling.

Every observation within a sliding time window is processed by a configurable
[mp2p_icp filter pipeline](https://docs.mola-slam.org/mp2p_icp/), defined in a
YAML file (see [params/point-cloud-pipeline.yaml](params/point-cloud-pipeline.yaml)):

- `generators`: how observations become point layers.
- `per_observation`: filters applied to each observation (e.g. removing the
  robot body).
- `final`: filters applied to the accumulated points (e.g. bounding box,
  decimation). One or more of its output layers are published as
  `sensor_msgs/PointCloud2`.

Two optional stages handle common sensor limitations:

- **Obstacle memory** (`memory_*` parameters): short-term memory of obstacles
  inside a region the sensors can't see (e.g. right in front of a high-mounted
  LiDAR). Points are stored in the reference frame, so obstacles that move into
  the blind zone are still reported, and are forgotten once observed free or
  after `memory_max_age`.
- **Obstacle confirmation** (`confirm_*` parameters): an output point is only
  reported if an update built from other observations had a point within
  `confirm_radius`, so one-off artifacts of a single scan are discarded.

## Demos

    ros2 launch mrpt_tutorials demo_pointcloud_pipeline_mvsim.launch.py

runs the node with a simulated Jackal robot in
[mvsim](https://github.com/MRPT/mvsim). The node is also used by most other
demos in [mrpt_tutorials](../mrpt_tutorials).

## Node: mrpt_pointcloud_pipeline_node

### Subscribed topics

- `<source_topics_2d_scans>` (`sensor_msgs/LaserScan`): 2D laser scans.
- `<source_topics_pointclouds>` (`sensor_msgs/PointCloud2`): point clouds.
- `/tf`, `/tf_static`: sensor poses on the robot (`frameid_robot`), and the
  robot pose in `frameid_reference` at each observation time.

### Published topics

- `<topic_local_map_pointcloud>` (`sensor_msgs/PointCloud2`): the local
  obstacle points, in `frameid_robot`. One topic per output layer.

### Parameters

| Name | Default | Description |
| --- | --- | --- |
| `source_topics_2d_scans` | `"scan, laser1"` | Comma-separated list of `LaserScan` topics. |
| `source_topics_pointclouds` | `""` | Comma-separated list of `PointCloud2` topics. |
| `pipeline_yaml_file` | (required) | YAML file with the mp2p_icp generators and filters. |
| `filter_output_layer_name` | `"output"` | Comma-separated list of pipeline output layers to publish. |
| `topic_local_map_pointcloud` | `"local_map_pointcloud"` | Comma-separated list of output topics, one per output layer. |
| `frameid_reference` | `"odom"` | Frame where observations are accumulated. |
| `frameid_robot` | `"base_link"` | Robot frame, also the frame of the published points. |
| `time_window` | `0.20` | Observations older than this [s] are discarded. |
| `one_observation_per_topic` | `false` | Use only the latest observation of each input topic. |
| `publish_period` | `0.05` | Timer period [s]. New observations are published right away; the timer handles those waiting for their robot pose and republishes the last output. Can't be larger than `time_window`. |
| `republish_unchanged` | `true` | Republish the last output on each timer period while there are no new observations. |
| `tf_max_wait` | `0.5` | How long [s] an observation waits for the robot pose at its time before being dropped. |
| `max_input_age` | `0.0` | If >0, nothing is published while the latest observation of some input topic (one that published before) is older than this [s], so a stalled sensor does not look like free space. |
| `memory_layers` | `""` | Comma-separated output layers with obstacle memory (empty: disabled). |
| `memory_region` | `""` | Remembered region `"x_min, y_min, x_max, y_max"` [m], robot frame. Required if `memory_layers` is set. |
| `memory_self_box` | `""` | Box `"x_min, y_min, x_max, y_max"` [m] (robot frame) whose points are never stored, e.g. the robot itself. |
| `memory_max_age` | `10.0` | Stored points are forgotten after this time [s]. |
| `memory_voxel_size` | `0.10` | Storage resolution [m]. |
| `memory_min_hits` | `3` | A voxel is only reported after being observed in this many updates. |
| `memory_store_margin` | `1.0` | Live points this close to the region [m] are stored too. |
| `confirm_radius` | `0.0` | If >0, enables obstacle confirmation with this radius [m]. |
| `confirm_max_age` | `0.5` | Past updates older than this [s] are not used for confirmation. Must be larger than `time_window`, unless `one_observation_per_topic` is true. |
| `show_gui` | `false` | Show an MRPT GUI window with the points. |

## Template ROS 2 launch files

This package provides [launch/pointcloud_pipeline.launch.py](launch/pointcloud_pipeline.launch.py):

    ros2 launch mrpt_pointcloud_pipeline pointcloud_pipeline.launch.py \
        scan_topic_name:=/scan

Its launch arguments map to the node parameters above: `scan_topic_name`
(`source_topics_2d_scans`), `points_topic_name` (`source_topics_pointclouds`),
`filter_output_topic_name` (`topic_local_map_pointcloud`, default
`/local_map_pointcloud`), `show_gui` (default `True`), and the rest with the
same names. Use `use_composable:=true container_name:=<name>` to load it into
an existing component container, and `log_level` to change the log level.
