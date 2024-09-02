^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_pf_localization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.1 (2024-09-02)
------------------
* Remove temporary workaround in <depends> for buggy mrpt_libros_bridge package.xml
* update dependencies
* Depend on new mrpt_lib packages (deprecate mrpt2)
* Contributors: Jose Luis Blanco-Claraco

* Remove temporary workaround in <depends> for buggy mrpt_libros_bridge package.xml
* update dependencies
* Depend on new mrpt_lib packages (deprecate mrpt2)
* Contributors: Jose Luis Blanco-Claraco

2.1.0 (2024-08-08)
------------------
* Fix GNSS name typo
* Merge branch 'ros2' into wip/port-tps-astar
* Merge branch 'ros2' into wip/port-tps-astar
* Contributors: Jose Luis Blanco-Claraco

2.0.1 (2024-05-28)
------------------

2.0.0 (2024-05-28)
------------------
* unit test executable now accepts many env var arguments for use in batch tests
* More relocalization parameters
* pf-test: fix static not POD warnings, and support env var-based config file too
* Fix for latest mp2p_icp api
* Implement relocalization based on ICP
* fix relocalization with reference pointmaps
* code clean up; check convergence in unit test
* Do not update the PF if there are no usable observations
* use pf/m² to initialize; estimate twist
* Expose gnss topic in the launch file
* GNSS-based initialization
* Comply with ROS2 REP-2003
* use namespaces for launch files
* allow overriding map likelihood options
* Show more debug info on metric map likelihood options
* New param for core-only debug level
* enable running the test from an MM file
* Receive gridmap from ROS topic
* Clearer warn messages
* Reorganize launch and demo files
* Prepare demo launch files
* Port mrpt localization to ROS 2 and whole refactor
* Renamed: mrpt_localization: mrpt_pf_localization
* Contributors: Jose Luis Blanco-Claraco

1.0.3 (2022-06-25)
------------------

1.0.2 (2022-06-25)
------------------

1.0.1 (2022-06-24)
------------------
* fix all build errors; removed now obsolete tf_prefix
* Ported to tf2 and mrpt::ros1bridge
* modernize cmake
* Contributors: Jose Luis Blanco-Claraco

1.0.0 (2022-04-30)
------------------
* Update URLs to https
* Update build dep to mrpt2
* Merge pull request `#118 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/118>`_ from mx-robotics/ag-remove_laser_tf_cache
  continuous laser pose update from tf tree fag added
* added ROS parameter continuous sensor pose update
* Fix incorrect publication of estimated pose (Closes: `#117 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/117>`_)
* Contributors: Jose Luis Blanco Claraco, Markus Bader

0.1.26 (2019-10-05)
-------------------

0.1.25 (2019-10-04)
-------------------
* fix build against mrpt2
* Contributors: Jose Luis Blanco-Claraco

0.1.24 (2019-04-12)
-------------------
* Fix build against MRPT 1.9.9
* Contributors: Inounx, Jose Luis Blanco-Claraco, Julian Lopez Velasquez, Markus Bader

0.1.23 (2018-06-14)
-------------------

0.1.22 (2018-05-22)
-------------------
* fix all catkin_lint errors
* remove exec +x flag to cfg files
* Contributors: Jose Luis Blanco-Claraco

0.1.21 (2018-04-27)
-------------------
* Upgrade version 0.1.20 (`#99 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/99>`_)
* fix build against mrpt 2.0
* partial fix build w mrpt 2.0
* fix build in mrpt 2.0
* optimized build (-O3)
* Fix travis (`#94 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/94>`_)
  * add dep stereo_msgs
  * add dep stereo_msgs
  * fix minor warnigngs and errors
* fix use c++14
* Merge branch 'master' of github.com:tuw-robotics/mrpt_navigation
* Merge branch 'master' into master
* CMake finds MRPT >=1.5 in ROS master branch
* Merge branch 'master' into compat-mrpt-1.5
* CMake finds MRPT >=1.9
* avoid Eigen warnings with GCC-7
* Removed unnecessry MRPT_VERSION checks
* Fixes for clang format
* Removed c++11 declarations in cmake
* Adapted CMakeLists to new mrpt
* Ported to a new version of MRPT
* Merge pull request `#74 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/74>`_ from bergercookie/devel
  Use C++11 in mrpt_bridge, mrpt_pf_localization
* Merge pull request `#77 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/77>`_ from corot/master
  Allow using maps from topic
* Allow using maps from topic
* Use C++11 in mrpt_bridge, mrpt_pf_localization
* Merge pull request `#72 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/72>`_ from bergercookie/devel
  Add more conversion methods for CNetworkOfPoses classes
* Correct include guard
* Fix debug messages: (`#65 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/65>`_)
  * replace printf with log_info
  * read and use debug param
  * dumping to console only once
  Also publish pose even if not updating the filter (sorry for mixing commits)
* Fix debug messages:
  * replace printf with log_info
  * read and use debug param
  * dumping to console only once
  Also publish pose even if not updating the filter (sorry for mixing commits)
* put right the covariance matrix initialization (`#63 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/63>`_)
* Merge pull request `#61 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/61>`_ from corot/master
  Prevent extrapolation into the past when publishing the tf
* Fix a bug in the way I handle filter state
* Prevent extrapolation into the past when publishing the tf and handle update/not update more nicelly. Also, ROS-format variables
* Contributors: Ashish Raste, Borys Tymchenko, Hunter Laux, Jorge Santos, Jorge Santos Simón, Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco, Markus Bader, Nikos Koukis, corot

0.1.20 (2018-04-26)
-------------------
* fix build against mrpt 2.0
* partial fix build w mrpt 2.0
* fix build in mrpt 2.0
* optimized build (-O3)
* Fix travis (`#94 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/94>`_)
  * add dep stereo_msgs
  * add dep stereo_msgs
  * fix minor warnigngs and errors
* fix use c++14
* Merge branch 'master' of github.com:tuw-robotics/mrpt_navigation
* Merge branch 'master' into master
* CMake finds MRPT >=1.5 in ROS master branch
* Merge branch 'master' into compat-mrpt-1.5
* CMake finds MRPT >=1.9
* avoid Eigen warnings with GCC-7
* Removed unnecessry MRPT_VERSION checks
* Fixes for clang format
* Removed c++11 declarations in cmake
* Adapted CMakeLists to new mrpt
* Ported to a new version of MRPT
* Merge pull request `#74 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/74>`_ from bergercookie/devel
  Use C++11 in mrpt_bridge, mrpt_pf_localization
* Merge pull request `#77 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/77>`_ from corot/master
  Allow using maps from topic
* Allow using maps from topic
* Use C++11 in mrpt_bridge, mrpt_pf_localization
* Merge pull request `#72 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/72>`_ from bergercookie/devel
  Add more conversion methods for CNetworkOfPoses classes
* Correct include guard
* Fix debug messages: (`#65 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/65>`_)
  * replace printf with log_info
  * read and use debug param
  * dumping to console only once
  Also publish pose even if not updating the filter (sorry for mixing commits)
* Fix debug messages:
  * replace printf with log_info
  * read and use debug param
  * dumping to console only once
  Also publish pose even if not updating the filter (sorry for mixing commits)
* put right the covariance matrix initialization (`#63 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/63>`_)
* Merge pull request `#61 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/61>`_ from corot/master
  Prevent extrapolation into the past when publishing the tf
* Fix a bug in the way I handle filter state
* Prevent extrapolation into the past when publishing the tf and handle update/not update more nicelly. Also, ROS-format variables
* Contributors: Ashish Raste, Borys Tymchenko, Jorge Santos, Jorge Santos Simón, Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco, Markus Bader, Nikos Koukis, corot


0.1.18 (2017-01-22)
-------------------

0.1.17 (2017-01-22)
-------------------
* Do not consider epsilon velocities (<1e-3) as robot moving
* make catkin_lint clean
* Remove all errors generated by catkin_lint and cleanup unused templates from CMakeLists.txt files
* Issue `#53 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/53>`_: add a parameter to disable updating when robot not moving
* Contributors: Jorge Santos, Jose Luis Blanco

0.1.16 (2016-12-13)
-------------------
* Fix for issue `#50 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/50>`_
* Tabs to spaces
* Fix for issue `#48 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/48>`_
* Remove unneeded include
* Allow robot poses from external algorithms to be integrated into mrpt particles filter
* fix typo
* Contributors: Jorge Santos, Jorge Santos Simón, Jose-Luis Blanco-Claraco

0.1.15 (2016-11-06)
-------------------
* Fix build against MRPT 1.5.0
* Use ros::Time::now() to time stamp first 10 poses
  If not, they contain wall time, what when working on simulation prevents robot_localization fusion to work.
  Other than that, the change is innocuous
* PR `#33 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/33>`_ prevented pose initialization with the robot stopped; fix it!
* Stop mrpt_pf_localization updating when robot is not moving (odom twist is zero)
* Contributors: Jorge Santos, Jorge Santos Simón, Jose-Luis Blanco-Claraco

0.1.14 (2016-09-12)
-------------------

0.1.13 (2016-09-03)
-------------------

0.1.12 (2016-09-03)
-------------------
* Put the ROS log setting withing if MRPT_VERSION>=0x150 so it doesn't break the compilation agains .deb mrpt libs
* Restamp pose on first iteration with ROS time because filter time is still not initialized and can create problems when integrating on robot_localization
* Set ROS log level also on MRPT internal log system. Prevents spamming of [FIXED_SAMPLING] and [ADAPTIVE SAMPLE SIZE] messages
* Modify so we can use in conjuntion with robot_localization package: provide a PoseWithCovarianceStamped, allow disabling tf publishing and make transform_tolerance a parameter
* Contributors: Jorge Santos

0.1.11 (2016-08-21)
-------------------

0.1.10 (2016-08-05)
-------------------

0.1.9 (2016-08-05)
------------------

0.1.8 (2016-06-29)
------------------

0.1.7 (2016-06-20)
------------------
* Fix laser scan stamp problem. TODO: something is still broken since nothing pops up for mrpt_pose
* fix almost everything to add a pose publisher
* Contributors: Megacephalo

0.1.6 (2016-03-20)
------------------
* New support for range-only (RO) localization
* fix build against mrpt <1.3.0
* Contributors: Jose Luis Blanco, Jose Luis Blanco-Claraco, Raphael Zack

0.1.5 (2015-04-29)
------------------
* fix to strange pf-localization bug
* Cleaner build against mrpt 1.3.0
* Fix build against mrpt 1.3.0
* Contributors: Jose Luis Blanco

0.1.4 (2014-12-27)
------------------
* dont publish if numSubscribers()==0
* fixes for mrpt 1.3.0
* Removed 'mrpt' dep from catkin_package().
  I *think* this is giving problems to dependant pkgs and is not needed...
* pose_cov_ops removed from mrpt_navigation metapkg
* localization: New param to configure sensor sources in a flexible way
* Contributors: Jose Luis Blanco

0.1.3 (2014-12-18)
------------------
* Fix many missing install files
* Contributors: Jose Luis Blanco

0.1.2 (2014-12-18)
------------------

0.1.1 (2014-12-17)
------------------
* First public binary release.

0.1.0 (2014-12-17)
------------------
* consistent version numbers
* fix build error without WX
* Fixes broken dependencies
* config and demos tested
* localization working like amcl
