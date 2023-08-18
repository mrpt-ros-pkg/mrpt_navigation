| Distro | Build dev | Build release | Stable sync |
| --- | --- | --- | --- |
| ROS 2 Humble (u22.04) | [![Build Status](https://build.ros2.org/job/Hdev__mrpt_navigation__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__mrpt_navigation__ubuntu_jammy_amd64/) |  TBD | [![Version](https://img.shields.io/ros/v/iron/mrpt_navigation)](https://index.ros.org/search/?term=mrpt_navigation) |
| ROS 2 Iron (u22.04) | [![Build Status](https://build.ros2.org/job/Idev__mrpt_navigation__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Idev__mrpt_navigation__ubuntu_jammy_amd64/) |  TBD | [![Version](https://img.shields.io/ros/v/iron/mrpt_navigation)](https://index.ros.org/search/?term=mrpt_navigation) |
| ROS 2 Rolling (u22.04) | [![Build Status](https://build.ros2.org/job/Rdev__mrpt_navigation__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mrpt_navigation__ubuntu_jammy_amd64/) |  TBD | [![Version](https://img.shields.io/ros/v/rolling/mrpt_navigation)](https://index.ros.org/search/?term=mrpt_navigation) |

<img align="center" src="https://mrpt.github.io/imgs/mrpt_reactivenav_ros_demo_s40.gif">

mrpt_navigation
===============

This repository provides packages that wrap functionality in the Mobile Robot Programming Toolkit ([MRPT](https://github.com/MRPT/mrpt/)) related to localization and navigation. SLAM and sensor access are wrapped into [other ROS repositories](https://github.com/mrpt-ros-pkg/).


Documentation for each package
----------------------------------

* [mrpt_local_obstacles](mrpt_local_obstacles): A node that maintains a local obstacle map.
* [mrpt_localization](mrpt_localization): A node for particle filter-based localization of a robot from any kind of metric map (gridmap, points, range-only sensors, ...).
* [mrpt_map](mrpt_map): A node that loads a ROS standard gridmap or an MRPT map and publishes it to a topic.
* ...


General documentation
----------------------------------
* ROS wiki: http://wiki.ros.org/mrpt_navigation
* Compiling instructions: http://wiki.ros.org/mrpt_navigation/Tutorials/Installing
* Usage examples and tutorials: http://wiki.ros.org/mrpt_navigation/Tutorials
* Branches:
  * `ros2`: The most recent, active branch for modern ROS 2 distributions.
  * `ros1`: Intended for ROS 1. No further development will happen there.

Demo videos
------------

* Localization with particle filters: http://youtu.be/b5glQhT2Zac
