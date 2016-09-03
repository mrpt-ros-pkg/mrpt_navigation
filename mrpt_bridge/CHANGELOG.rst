^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

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

