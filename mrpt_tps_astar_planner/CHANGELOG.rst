^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_tps_astar_planner_node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

