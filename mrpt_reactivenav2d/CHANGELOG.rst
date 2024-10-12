^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_reactivenav2d
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.1 (2024-10-12)
------------------
* Update for MRPT 2.14.3 (new RNAV weight for target heading)
* Support speed_ratio property for Waypoints
* Improve astar navigation demo
* Contributors: Jose Luis Blanco-Claraco

2.2.0 (2024-09-25)
------------------
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
* Hide example output into <details>
* Merge branch 'ros2' into wip/port-tps-astar
* rnav node now detects live changes in the parameter 'pure_pursuit_mode'
* Merge branch 'ros2' into wip/port-tps-astar
* Contributors: Jose Luis Blanco-Claraco

2.0.1 (2024-05-28)
------------------

2.0.0 (2024-05-28)
------------------
* rnav: add new pure_pursuit mode
* Implement two action servers: NavigateGoal and NavigateWaypoints
* Comply with ROS2 REP 2003
* expose waypoint sequence launch argument
* use namespaces for launch files
* rnav node: publish selected PTG as marker
* FIX: waypoints should also observe frame_id
* Fix obsolete tf2_geometry_msgs.h header
* Unify and clarify license headers in all files
* Merge pull request `#134 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/134>`_ from SRai22/ros2
  Merge from SRai22 fork
* Port to ROS 2
* Contributors: Jose Luis Blanco-Claraco, Raúl Aguilera López, SRai22

1.0.3 (2022-06-25)
------------------

1.0.2 (2022-06-25)
------------------

1.0.1 (2022-06-24)
------------------
* Correct demos
* Fix all build errors
* Removed now obsolete tf_prefix
* Ported to tf2 and mrpt::ros1bridge
* Contributors: Jose Luis Blanco-Claraco

1.0.0 (2022-04-30)
------------------
* Update URLs to https
* Update build dep to mrpt2
* Contributors: Jose Luis Blanco Claraco

0.1.26 (2019-10-05)
-------------------

0.1.25 (2019-10-04)
-------------------
* fix nav to target clicking in RVIZ
* Contributors: Jose Luis Blanco Claraco

0.1.24 (2019-04-12)
-------------------
* Fix build against MRPT 1.9.9
* Contributors: Julian Lopez Velasquez

0.1.23 (2018-06-14)
-------------------

0.1.20 (2018-04-26)
-------------------
* fix build against mrpt 2.0
* partial fix build w mrpt 2.0
* optimized build (-O3)
* fix build errors against mrpt 1.5.5
* Merge branch 'master' of github.com:tuw-robotics/mrpt_navigation
* Merge branch 'master' into master
* CMake finds MRPT >=1.5 in ROS master branch
* Merge branch 'master' into compat-mrpt-1.5
* CMake finds MRPT >=1.9
* avoid Eigen warnings with GCC-7
* Removed unnecessry MRPT_VERSION checks
* Fixes for clang format
* Removed c++11 declarations in cmake
* Fixed error in demo launch file
* Adapted CMakeLists to new mrpt
* Ported to a new version of MRPT
* Issue `#75 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/75>`_: fix mrpt_reactivenav2d compilation errors
* Issue `#75 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/75>`_: fix mrpt_reactivenav2d compilation errors
* deprecated mrpt 2.0 module names
* Update launch file to work with MRPT 1.5.0
* fix build with MRPT 1.5.0
* Contributors: Borys Tymchenko, Jose Luis Blanco, Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco, Markus Bader, corot


0.1.22 (2018-05-22)
-------------------
* fix all catkin_lint errors
* Contributors: Jose Luis Blanco-Claraco

0.1.21 (2018-04-27)
-------------------
* Upgrade version 0.1.20 (`#99 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/99>`_)
* fix build against mrpt 2.0
* partial fix build w mrpt 2.0
* optimized build (-O3)
* fix build errors against mrpt 1.5.5
* Merge branch 'master' of github.com:tuw-robotics/mrpt_navigation
* Merge branch 'master' into master
* CMake finds MRPT >=1.5 in ROS master branch
* Merge branch 'master' into compat-mrpt-1.5
* CMake finds MRPT >=1.9
* avoid Eigen warnings with GCC-7
* Removed unnecessry MRPT_VERSION checks
* Fixes for clang format
* Removed c++11 declarations in cmake
* Fixed error in demo launch file
* Adapted CMakeLists to new mrpt
* Ported to a new version of MRPT
* Issue `#75 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/75>`_: fix mrpt_reactivenav2d compilation errors
* Issue `#75 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/75>`_: fix mrpt_reactivenav2d compilation errors
* deprecated mrpt 2.0 module names
* Update launch file to work with MRPT 1.5.0
* fix build with MRPT 1.5.0
* Contributors: Borys Tymchenko, Hunter Laux, Jose Luis Blanco, Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco, Markus Bader, corot

0.1.18 (2017-01-22)
-------------------

0.1.17 (2017-01-22)
-------------------
* fix build against latest mrpt 1.5.0
* make catkin_lint clean
* Remove all errors generated by catkin_lint and cleanup unused templates from CMakeLists.txt files
* Update to MRPT 1.5.0
* Contributors: Jorge Santos, Jose Luis Blanco

0.1.16 (2016-12-13)
-------------------
* Fix `#52 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/52>`_
* Contributors: Jose-Luis Blanco-Claraco

0.1.15 (2016-11-06)
-------------------
* Fix mrpt-reactivenav2d compilation errors
* Fix compilation for MRPT < 1.5.0
* Add include guard  for MRPT >= 1.5.0, fill timestamp entries
* Add include guard for CVehicleVelCmd_DiffDriven in mrpt >= 1.5.0
* Fix mrpt-reactivenav2d compilation errors
* Contributors: Nikos Koukis, bergercookie

0.1.14 (2016-09-12)
-------------------

0.1.13 (2016-09-03)
-------------------

0.1.12 (2016-09-03)
-------------------

0.1.11 (2016-08-21)
-------------------

0.1.10 (2016-08-05)
-------------------
* fix build error against mrpt < 1.5.0
* Contributors: Jose-Luis Blanco-Claraco

0.1.9 (2016-08-05)
------------------
* fix build against mrpt 1.5.0
* Contributors: Jose-Luis Blanco-Claraco

0.1.8 (2016-06-29)
------------------
* Reactive nav default config file: coarser collision grid for faster initialization
* fix build and sample config file for reactivenav with mrpt>=1.5.0
* Contributors: Jose-Luis Blanco-Claraco

0.1.7 (2016-06-20)
------------------

0.1.6 (2016-03-20)
------------------
* more build fixes
* reactivenav: more complete template config file
* Contributors: Jose Luis Blanco

0.1.5 (2015-04-29)
------------------

0.1.4 (2014-12-27)
------------------
* First working version of the reactive navigator
* Contributors: Jose Luis Blanco

0.1.3 (2014-12-18 23:21)
------------------------

0.1.2 (2014-12-18 11:49)
------------------------

0.1.1 (2014-12-17)
------------------
