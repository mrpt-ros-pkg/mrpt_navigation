^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_localization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

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

