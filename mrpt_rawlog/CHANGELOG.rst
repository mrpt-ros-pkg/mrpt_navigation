^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_rawlog
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

