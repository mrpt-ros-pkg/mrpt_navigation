^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_msgs_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.0 (2024-09-25)
------------------
* fix missing linters; tune tutorial params
* Update URL entries in package.xml to each package proper documentation
* ament linters: manually enable just cmake and xml linters
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
* Merge branch 'ros2' into wip/port-tps-astar
* Contributors: Jose Luis Blanco-Claraco

2.0.1 (2024-05-28)
------------------

2.0.0 (2024-05-28)
------------------
* Prepare demo launch files
* Port mrpt localization for ros2 and whole refactor
* Add ament linter for testing builds
* Make marker_msgs an optional dependency.
  Closes `#138 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/138>`_
* Unify and clarify license headers in all files
* ROS2 port: mrpt_msgs_bridge
* Contributors: Jose Luis Blanco-Claraco, SRai22

1.0.3 (2022-06-25)
------------------
* Fix CMake script error due to last commit typo
* Contributors: Jose Luis Blanco Claraco

1.0.2 (2022-06-25)
------------------
* Fix wrong destination for mrpt_msgs_bridge headers
* Contributors: Jose Luis Blanco Claraco

1.0.1 (2022-06-24)
------------------
* New package mrpt_msgs_bridge
* Contributors: Jose Luis Blanco-Claraco
