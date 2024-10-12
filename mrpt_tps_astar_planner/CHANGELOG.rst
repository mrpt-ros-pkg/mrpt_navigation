^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_tps_astar_planner_node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.1 (2024-10-12)
------------------
* Update demo for astar
* Publish costmaps to ROS for visualization too apart of the custom MRPT GUI
* Improve astar navigation demo
* astar planner: add refine() step and parameter to optionally disable it
* astar node: add two more launch args: problem_world_bbox_margin and problem_world_bbox_ignore_obstacles
* astar params: add more comments and tune for speed
* PTGs .ini: Add docs on how to enable backward motions
* Contributors: Jose Luis Blanco-Claraco

2.2.0 (2024-09-25)
------------------
* fix missing linters; tune tutorial params
* Update URL entries in package.xml to each package proper documentation
* ament linters: manually enable just cmake and xml linters
* Add roslog INFO traces to measure time spent initializing PTGs
* reformat clang-format with 100 column width
* Contributors: Jose Luis Blanco-Claraco

2.1.1 (2024-09-02)
------------------
* Remove temporary workaround in <depends> for buggy mrpt_libros_bridge package.xml
* Fix duplicated deps
* update dependencies
* Depend on new mrpt_lib packages (deprecate mrpt2)
* Contributors: Jose Luis Blanco-Claraco

* Remove temporary workaround in <depends> for buggy mrpt_libros_bridge package.xml
* Fix duplicated deps
* update dependencies
* Depend on new mrpt_lib packages (deprecate mrpt2)
* Contributors: Jose Luis Blanco-Claraco

2.1.0 (2024-08-08)
------------------
* Merge pull request `#147 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/147>`_ from r-aguilera/ros2
  mrpt_tps_astar_planner_node tf data availability
* added max duration to wait for tf data
* Merge pull request `#146 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/146>`_ from r-aguilera/ros2
  parameterized astar plan waypoints fields
* Merge pull request `#145 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/145>`_ from mrpt-ros-pkg/ros2_astar
  add WaypointSequence missing header
* Implement two services for making navigation plans; code clean up; delete "replan" topic
* FIX: transform point cloud observations
* add frame_id params
* reset planner obstacles between path plans
* Add param to create subscribers with map-like QoS
* Enable multiple obstacle sources (grids and points)
* delete old ros1 files
* Merge pull request `#144 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/144>`_ from mrpt-ros-pkg/wip/port-tps-astar
  Port TPS A* planner to ROS2
* Contributors: Jose Luis Blanco-Claraco, Raúl Aguilera, SRai22

