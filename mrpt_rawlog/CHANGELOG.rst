^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_rawlog
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.0 (2024-09-25)
------------------
* Update URL entries in package.xml to each package proper documentation
* ament linters: manually enable just cmake and xml linters
* reformat clang-format with 100 column width
* Contributors: Jose Luis Blanco-Claraco

2.1.1 (2024-09-02)
------------------
* Remove temporary workaround in <depends> for buggy mrpt_libros_bridge package.xml
* update dependencies
* Depend on new mrpt_lib packages (deprecate mrpt2)
* rosbag2rawlog app: support generating CObservationOdometry from /tf odom->base_link msgs
* mrpt_rawlog: delete old ROS1 leftover files
* Contributors: Jose Luis Blanco-Claraco

* Remove temporary workaround in <depends> for buggy mrpt_libros_bridge package.xml
* update dependencies
* Depend on new mrpt_lib packages (deprecate mrpt2)
* rosbag2rawlog app: support generating CObservationOdometry from /tf odom->base_link msgs
* mrpt_rawlog: delete old ROS1 leftover files
* Contributors: Jose Luis Blanco-Claraco

2.1.0 (2024-08-08)
------------------
* BUGFIX: Data stream ignored for sensors where fixed_sensor_pose was defined.
* Fix GNSS name typo
* Merge branch 'ros2' into wip/port-tps-astar
* Merge branch 'ros2' into wip/port-tps-astar
* Contributors: Jose Luis Blanco-Claraco

2.0.1 (2024-05-28)
------------------

2.0.0 (2024-05-28)
------------------
* rosbag2rawlog: BUGFIX: sensor poses were inverted
* rosbag2rawlog: drop msgs if /tf data did not arrive yet instead of aborting
* rosbag2rawlog: now also can convert from mrpt_msgs/GenericObservation messages
* Import GNSS observations
* Fix build with older versions of cv_bridge
* rosbag2rawlog: Implement automatic detection of sensorPose from /tf, and added option to override poses from config yaml
* rosbag2rawlog: Add support for XYZIRT pointcloud observations
* Add support for CObservationRotatingScan in rosbag2rawlog
* Fix rosbag2rawlog install path should be 'bin'
* rosbag2rawlog: add conversion for LaserScan msgs
* Fix missing build dep
* Add ament linter for testing builds
* mrpt_rawlog: remove all record nodes. The package will only provide a play rawlog node, and offline tool to convert rosbag2 to rawlog
* Fix obsolete tf2_geometry_msgs.h header
* Unify and clarify license headers in all files
* Port to ROS 2
* Contributors: Jose Luis Blanco-Claraco

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
* Merge pull request `#126 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/126>`_ from lukschwalb/master
  Fix incorrect class check
* Fix incorrect class name
* Merge pull request `#119 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/119>`_ from mx-robotics/develop
  rawlog_record: fnc call fileNameStripInvalidChars removed
* Contributors: Jose Luis Blanco Claraco, Markus Bader, Schwalb, Luk

0.1.26 (2019-10-05)
-------------------
* mrpt_rawlog: abort if input file does not exist. Fix crash if built against mrpt1.
* remove qtcreator temporary files
* Contributors: Jose Luis Blanco Claraco

0.1.25 (2019-10-04)
-------------------
* fix build in gcc 4.x
* fix build against mrpt1 (Closes `#113 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/113>`_)
* fix build against current mrpt2
* Contributors: Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco

0.1.24 (2019-04-12)
-------------------
* Fix build against mrpt-1.5
* Fix build against MRPT 1.9.9
* fix regression introduced in last refactor of mrpt_rawlog
* rawlog_record: Explain error if cannot write .rawlog
* Contributors: Jose Luis Blanco Claraco, Julian Lopez Velasquez

0.1.23 (2018-06-14)
-------------------

0.1.20 (2018-04-26)
-------------------
* rawlog_play: clean up; support obs-only rawlog
* fix missing mrpt/config hdr
* fix build against mrpt 2.0
* partial fix build w mrpt 2.0
* optimized build (-O3)
* Fix travis (`#94 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/94>`_)
  * add dep stereo_msgs
  * add dep stereo_msgs
  * fix minor warnigngs and errors
* Use docker to run modern ROS distros in travis (`#93 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/93>`_)
* marker messages are also logged as beacons if needed
* mrpt_rawlog recorder update
* static tf in rawlog record fixed
* marker msgs added
* timestamp problem solved
* Merge branch 'master' of github.com:tuw-robotics/mrpt_navigation
* bearing readings added
* Merge branch 'master' into master
* CMake finds MRPT >=1.5 in ROS master branch
* Merge branch 'master' into compat-mrpt-1.5
* CMake finds MRPT >=1.9
* avoid Eigen warnings with GCC-7
* Removed unnecessry MRPT_VERSION checks
* Fixes for clang format
* Adapted CMakeLists to new mrpt
* Ported to a new version of MRPT
* Fix paths in demo_play_ekf(_2d).launch files.
* [mrpt_rawlog] Fix missing rosbag dependency in package.xml. (`#66 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/66>`_)
* [mrpt_rawlog] Fix missing rosbag dependency in package.xml.
* Contributors: Borys Tymchenko, Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco, Logrus, Markus Bader, Vladislav Tananaev


0.1.22 (2018-05-22)
-------------------
* fix all catkin_lint errors
* Contributors: Jose Luis Blanco-Claraco

0.1.21 (2018-04-27)
-------------------
* Upgrade version 0.1.20 (`#99 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/99>`_)
* rawlog_play: clean up; support obs-only rawlog
* fix missing mrpt/config hdr
* fix build against mrpt 2.0
* partial fix build w mrpt 2.0
* optimized build (-O3)
* Fix travis (`#94 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/94>`_)
  * add dep stereo_msgs
  * add dep stereo_msgs
  * fix minor warnigngs and errors
* Use docker to run modern ROS distros in travis (`#93 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/93>`_)
* marker messages are also logged as beacons if needed
* mrpt_rawlog recorder update
* static tf in rawlog record fixed
* marker msgs added
* timestamp problem solved
* Merge branch 'master' of github.com:tuw-robotics/mrpt_navigation
* bearing readings added
* Merge branch 'master' into master
* CMake finds MRPT >=1.5 in ROS master branch
* Merge branch 'master' into compat-mrpt-1.5
* CMake finds MRPT >=1.9
* avoid Eigen warnings with GCC-7
* Removed unnecessry MRPT_VERSION checks
* Fixes for clang format
* Adapted CMakeLists to new mrpt
* Ported to a new version of MRPT
* Fix paths in demo_play_ekf(_2d).launch files.
* [mrpt_rawlog] Fix missing rosbag dependency in package.xml. (`#66 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/66>`_)
* [mrpt_rawlog] Fix missing rosbag dependency in package.xml.
* Contributors: Borys Tymchenko, Hunter Laux, Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco, Logrus, Markus Bader, Vladislav Tananaev

0.1.18 (2017-01-22)
-------------------

0.1.17 (2017-01-22)
-------------------
* Remove all errors generated by catkin_lint and cleanup unused templates from CMakeLists.txt files
* Contributors: Jorge Santos

0.1.16 (2016-12-13)
-------------------

0.1.15 (2016-11-06)
-------------------
* Fix build against MRPT 1.5.0
* Contributors: Jose-Luis Blanco-Claraco

0.1.14 (2016-09-12)
-------------------

0.1.13 (2016-09-03)
-------------------

0.1.12 (2016-09-03)
-------------------

0.1.11 (2016-08-21)
-------------------
* fix missing #include
* Add wheeled robot example and 2d ekf.
* Add landmark to bridge.
* Contributors: Jose Luis Blanco, Logrus

0.1.10 (2016-08-05)
-------------------

0.1.9 (2016-08-05)
------------------
* fix install of .so targets
* Contributors: Jose-Luis Blanco-Claraco

0.1.8 (2016-06-29)
------------------
* Fix CMake dependencies (it failed to build in some platforms randomly)
* Contributors: Jose-Luis Blanco-Claraco

0.1.7 (2016-06-20)
------------------

0.1.6 (2016-03-20)
------------------
* added a launch file that plays a range-only rawlog
* Added in beacon publisher capabilities
* fix build with latest mrpt version
* update stamp with ros time now
  - since no clock recorded, tf/msgs published in the past, complains from everywhere
  - todo : extrapolate time between first/last msg stamp and pub clock
* default laser frame if msg_laser\_ has none
* Contributors: Jeremie Deray, Jose Luis Blanco, Raphael Zack

0.1.5 (2015-04-29)
------------------
* Cleaner build against mrpt 1.3.0
* Fix build against mrpt 1.3.0
* Contributors: Jose Luis Blanco

0.1.4 (2014-12-27)
------------------
* Removed 'mrpt' dep from catkin_package().
  I *think* this is giving problems to dependant pkgs and is not needed...
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
* More debug output
* consistent version numbers
* Fix demo_play with a sample .rawlog (was missing)
* Fixes broken dependencies
* Removed obsolete rawlog_play & fix build of other nodes.
* Update all wiki URLs
* Fix build with mrpt 1.2.x
