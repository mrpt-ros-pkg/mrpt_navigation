# mrpt_tps_astar_planner

## Overview
This package provides a ROS 2 node that uses the PTG-based A* planner in mrpt_path_planning
to publish waypoint sequences moving a non-holonomic robot from A to B, taking into account
its real shape, orientation, and kinematic constraints.

## How to cite

<details>
    TBD!
</details>


## Configuration

Write me!

## Demos

Write me!

## Node: mrpt_tps_astar_planner_node

### Working rationale

Uses A* over a SE(2) lattice and PTGs to sample collision-free paths. The implementation is an anytime algorithm.


### ROS 2 parameters

* `topic_wp_seq_pub` (Default: `/waypoints`) Desired name of the topic in which to publish the calculated paths.

* `topic_goal_sub`: The name of the topic to subscribe for goal poses (`geometry_msgs/PoseStamped`).

* `show_gui`: Shows its own MRPT GUI with the planned paths.

* `topic_gridmap_sub`: One or more (comma separated) topic names to subscribe for occupancy grid maps.

* `topic_obstacle_points_sub`: One or more (comma separated) topic names to subscribe for obstacle points.


### Subscribed topics
* xxx

### Published topics
* `<topic_wp_seq_pub>`  (Default: `/waypoints`) (`mrpt_msgs::msg::WaypointSequence`): Calculated trajectory, in mrpt_msgs format with complete details for each waypoints.
* `<topic_wp_seq_pub>_path`  (Default: `/waypoints_path`) (`nav_msgs::msg::Path`): Calculated trajectory, as `nav_msgs::Path`. Mostly useful for visualization only.

### Services

Write me!

### Template ROS 2 launch files

This package provides [launch/tps_astar_planner.launch.py](launch/tps_astar_planner.launch.py):

    ros2 launch tps_astar_planner tps_astar_planner.launch.py

which can be used in user projects to launch the planner, by setting these [launch arguments](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Using-Substitutions.html):

* ``XXX``: XXX


