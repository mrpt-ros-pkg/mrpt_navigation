^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.20 (2018-04-26)
-------------------
* fix warnings
* fix build against mrpt 2.0
* partial fix build w mrpt 2.0
* optimized build (-O3)
* Fix travis (`#94 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/94>`_)
  * add dep stereo_msgs
  * add dep stereo_msgs
  * fix minor warnigngs and errors
* Use docker to run modern ROS distros in travis (`#93 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/93>`_)
* fix build errors against mrpt master. Closes `#92 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/92>`_
* Fix install .h files in mrpt-bridge
  Closes `#91 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/91>`_
* Fix CMake error (Closes `#90 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/90>`_)
* marker messages are also logged as beacons if needed
* marker msgs added
* Merge branch 'master' of github.com:tuw-robotics/mrpt_navigation
* Merge pull request `#83 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/83>`_ from raghavendersahdev/master
  Extending ros_bridge to playback MRPT datasets
* Merge branch 'master' into master
* Work done upto August 25 : added CObservationRange
* CMake finds MRPT >=1.5 in ROS master branch
* Merge branch 'master' into compat-mrpt-1.5
* CMake finds MRPT >=1.9
* Work done upto August 21: added stereo image support to mrpt bridge
* small change moved beacon.h to its correct place
* Work done from August 17 to August 18 : added Tester function test the mrpt-ros bridge and publishing ros message after conversion
* changed test_Bridge file, testing the bridge in a different way
* Work done from August 17 to August 18 : added a tester function of the bridge, error while compiling currently
* avoid Eigen warnings with GCC-7
* Removed unnecessry MRPT_VERSION checks
* Fixes for clang format
* Removed c++11 declarations in cmake
* Adapted CMakeLists to new mrpt
* Work done from August 13 to August 14 : added image mrpt-ros bridge and comments
* Work done upto August 13: added mrpt_bridge conversions for range, GPS, IMU under construction; pending: mrpt_bridge for images and comments
* Merge pull request `#74 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/74>`_ from bergercookie/devel
  Use C++11 in mrpt_bridge, mrpt_localization
* Use C++11 in mrpt_bridge, mrpt_localization
* Merge pull request `#72 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/72>`_ from bergercookie/devel
  Add more conversion methods for CNetworkOfPoses classes
* Comply to clang warnings
* Fix compilation issue with forgotten git-related text
* Correct typo
* Work towards mr-slam
* Implement converion method for CNetworkOfPoses2DInf_NA specialization class
* Add more msgs for use in mrpt_graphslam_2d
* fix build with MRPT 1.5.0
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
* documentation patch in mrpt_bridge beacon header (`#62 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/62>`_)
* Fix `#60 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/60>`_
* Contributors: Ashish Raste, Borys Tymchenko, Jorge Santos, Jorge Santos Simón, Jose Luis Blanco, Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco, Jose-Luis Blanco-Claraco, Markus Bader, Nikos Koukis, Raghavender Sahdev

0.1.19 (2017-08-25)
-------------------
* Added mrpt_ros bridge for the following messages
* mrpt::obs::CObservationImage <--> sensor_msgs/Image
* mrpt::obs::CObservationIMU <--> sensor_msgs/IMU
* mrpt::obs::CObservationGPS <--> sensor_msgs/NavSatFix
* mrpt::obs::CObservationRange <--> sensor_msgs/Range
* mrpt::obs::CObservationStereoImages <--> 3xsensor_Image for left, right and disparity
* also added the file test_Bridge to test the above conversions and publishing on respective ROS topics
* Contributors: Raghavender Sahdev

0.1.21 (2018-04-27)
-------------------
* Upgrade version 0.1.20 (`#99 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/99>`_)
* fix warnings
* fix build against mrpt 2.0
* partial fix build w mrpt 2.0
* optimized build (-O3)
* Fix travis (`#94 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/94>`_)
  * add dep stereo_msgs
  * add dep stereo_msgs
  * fix minor warnigngs and errors
* Use docker to run modern ROS distros in travis (`#93 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/93>`_)
* fix build errors against mrpt master. Closes `#92 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/92>`_
* Fix install .h files in mrpt-bridge
  Closes `#91 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/91>`_
* Fix CMake error (Closes `#90 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/90>`_)
* marker messages are also logged as beacons if needed
* marker msgs added
* Merge branch 'master' of github.com:tuw-robotics/mrpt_navigation
* Merge pull request `#83 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/83>`_ from raghavendersahdev/master
  Extending ros_bridge to playback MRPT datasets
* Merge branch 'master' into master
* Work done upto August 25 : added CObservationRange
* CMake finds MRPT >=1.5 in ROS master branch
* Merge branch 'master' into compat-mrpt-1.5
* CMake finds MRPT >=1.9
* Work done upto August 21: added stereo image support to mrpt bridge
* small change moved beacon.h to its correct place
* Work done from August 17 to August 18 : added Tester function test the mrpt-ros bridge and publishing ros message after conversion
* changed test_Bridge file, testing the bridge in a different way
* Work done from August 17 to August 18 : added a tester function of the bridge, error while compiling currently
* avoid Eigen warnings with GCC-7
* Removed unnecessry MRPT_VERSION checks
* Fixes for clang format
* Removed c++11 declarations in cmake
* Adapted CMakeLists to new mrpt
* Work done from August 13 to August 14 : added image mrpt-ros bridge and comments
* Work done upto August 13: added mrpt_bridge conversions for range, GPS, IMU under construction; pending: mrpt_bridge for images and comments
* Merge pull request `#74 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/74>`_ from bergercookie/devel
  Use C++11 in mrpt_bridge, mrpt_localization
* Use C++11 in mrpt_bridge, mrpt_localization
* Merge pull request `#72 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/72>`_ from bergercookie/devel
  Add more conversion methods for CNetworkOfPoses classes
* Comply to clang warnings
* Fix compilation issue with forgotten git-related text
* Correct typo
* Work towards mr-slam
* Implement converion method for CNetworkOfPoses2DInf_NA specialization class
* Add more msgs for use in mrpt_graphslam_2d
* fix build with MRPT 1.5.0
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
* documentation patch in mrpt_bridge beacon header (`#62 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/62>`_)
* Fix `#60 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/60>`_
* Contributors: Ashish Raste, Borys Tymchenko, Hunter Laux, Jorge Santos, Jorge Santos Simón, Jose Luis Blanco, Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco, Jose-Luis Blanco-Claraco, Markus Bader, Nikos Koukis, Raghavender Sahdev

0.1.18 (2017-01-22)
-------------------
* Remove types_simple.h header to fix compilation
* Contributors: Jose Luis Blanco, Nikos Koukis

0.1.17 (2017-01-22)
-------------------
* remove debug ::pause()
* make catkin_lint clean
* mrpt_bridge: Move includes in implementation file
* mrpt_bridge: Add conversion definition in NetworkOfPoses
* Fix indentation, formatting in pose conversion files
  Abide general code style according to Google C++ code guidelines and
  [MRPT coding style](https://github.com/MRPT/mrpt/blob/master/doc/MRPT_Coding_Style.md)
* mrpt_bridge: Fix bug in MRPT->ROS pose conversion
  When transorfming from MRPT 3x3 form to PoseWithCovariance 6x6 form the
  yaw components were overwritten, thus resulting yaw components would
  always be 0.
* Correct minor typo
* mrpt_bridge: Add conversion methods for NetworkOfPoses
  Commit adds conversion methods between mrpt_msgs::NetworkOfPoses and
  mrpt::graphs::CNetworkOfPoses2DInf class instances
* mrpt_bridge: Add conversion methods for Information-related Pose classes
* Remove all errors generated by catkin_lint and cleanup unused templates from CMakeLists.txt files
* Contributors: Jorge Santos, Jose Luis Blanco, Nikos Koukis

0.1.16 (2016-12-13)
-------------------

0.1.15 (2016-11-06)
-------------------
* mrpt_bridge: Fix bug in laserScans conversion
* Fix new scanRange API in MRPT 1.5.0
* Contributors: Jose-Luis Blanco-Claraco, Nikos Koukis

0.1.14 (2016-09-12)
-------------------
* fix build against mrpt < 1.5.0
* Contributors: Jose-Luis Blanco-Claraco

0.1.13 (2016-09-03)
-------------------

0.1.12 (2016-09-03)
-------------------
* Put the ROS log setting withing if MRPT_VERSION>=0x150 so it doesn't break the compilation agains .deb mrpt libs
* Add a check for an empty string.
* Strip trailing \n in logs if present.
* Add a callback function to stream MRPT logs to ROS.
* Contributors: Jorge Santos, Logrus

0.1.11 (2016-08-21)
-------------------
* fix unit test error due to uninitialized tf::Pose
* fix deprecated PCL header
* Add landmark to bridge.
* Contributors: Jose-Luis Blanco-Claraco, Logrus

0.1.10 (2016-08-05)
-------------------

0.1.9 (2016-08-05)
------------------

0.1.8 (2016-06-29)
------------------

0.1.7 (2016-06-20)
------------------

0.1.6 (2016-03-20)
------------------
* New ObservationRangeBeacon message.
* More descriptive error msgs
* Contributors: Jose Luis Blanco, Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco, Logrus, Raphael Zack

0.1.5 (2015-04-29)
------------------
* mrpt_bridge: BUGFIX in convert() for 360deg scans
* Cleaner build against mrpt 1.3.0
* Fix build against mrpt 1.3.0
* Contributors: Jose Luis Blanco

0.1.4 (2014-12-27)
------------------
* Solved some old 'TODO' comments
* Removed 'mrpt' dep from catkin_package().
  I *think* this is giving problems to dependant pkgs and is not needed...
* Start new pkg mrpt_local_obstacles.
  Fixes in package.xml's
* Better doxygen docs
* localization: New param to configure sensor sources in a flexible way
* Contributors: Jose Luis Blanco

0.1.3 (2014-12-18)
------------------

0.1.2 (2014-12-18)
------------------
* Fix missing build dependency (nav_msgs)

0.1.1 (2014-12-17)
------------------
* First public binary release.


0.1.0 (2014-12-17)
------------------
* consistent version numbers
* Fixes broken dependencies
* Removed obsolete rawlog_play & fix build of other nodes.
* Fix build with mrpt 1.2.x
* localization uses tf odom
* localization working like amcl

