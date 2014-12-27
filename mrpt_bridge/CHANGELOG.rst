^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

