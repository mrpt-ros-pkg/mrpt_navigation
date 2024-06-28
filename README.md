[![CI Build colcon](https://github.com/mrpt-ros-pkg/mrpt_navigation/actions/workflows/build-ros.yml/badge.svg)](https://github.com/mrpt-ros-pkg/mrpt_navigation/actions/workflows/build-ros.yml)

| Distro | Build dev | Stable sync |
| --- | --- | --- |
| ROS 2 Humble (u22.04) | [![Build Status](https://build.ros2.org/job/Hdev__mrpt_navigation__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__mrpt_navigation__ubuntu_jammy_amd64/) | [![Version](https://img.shields.io/ros/v/iron/mrpt_navigation)](https://index.ros.org/search/?term=mrpt_navigation) |
| ROS 2 Iron (u22.04) | [![Build Status](https://build.ros2.org/job/Idev__mrpt_navigation__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Idev__mrpt_navigation__ubuntu_jammy_amd64/) |  [![Version](https://img.shields.io/ros/v/iron/mrpt_navigation)](https://index.ros.org/search/?term=mrpt_navigation) |
| ROS 2 Jazzy (u24.04) | [![Build Status](https://build.ros2.org/job/Jdev__mrpt_navigation__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__mrpt_navigation__ubuntu_noble_amd64/) | [![Version](https://img.shields.io/ros/v/jazzy/mrpt_navigation)](https://index.ros.org/search/?term=mrpt_navigation) |
| ROS 2 Rolling (u24.04) | [![Build Status](https://build.ros2.org/job/Rdev__mrpt_navigation__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mrpt_navigation__ubuntu_noble_amd64/) | [![Version](https://img.shields.io/ros/v/rolling/mrpt_navigation)](https://index.ros.org/search/?term=mrpt_navigation) |

<img align="center" src="https://mrpt.github.io/imgs/mrpt_reactivenav_ros_demo_s40.gif">

mrpt_navigation
===============

This repository provides packages that wrap functionality in the Mobile Robot Programming Toolkit ([MRPT](https://github.com/MRPT/mrpt/)) related to localization and navigation. MRPT SLAM and sensor access are wrapped into [other ROS repositories](https://github.com/mrpt-ros-pkg/).

The latest **SLAM framework**, whose maps are compatible with this repository for localization, is [MOLA](https://github.com/MOLAorg/).


Documentation for each package
----------------------------------
All packages follow [REP-2003](https://ros.org/reps/rep-2003.html) regarding ROS 2 topic QoS.

Related to localization:
* [mrpt_map_server](mrpt_map_server): A node that loads a ROS standard gridmap or an MRPT or MP2P_ICP map and publishes it to a (set of) topic(s).
* [mrpt_pf_localization](mrpt_pf_localization): A node for particle filter-based localization of a robot from any kind of metric map (gridmap, points, range-only sensors, ...).

Related to sensor pipelines:
* [mrpt_pointcloud_pipeline](mrpt_pointcloud_pipeline): A node that maintains a local obstacle map from recent sensor readings, including optional point cloud pipeline filtering or processing. For example,
  - For 3D LIDARs, to filter by a volume or area, downsample the number of points, etc.
  - For 2D laser scanners, to keep a memory of obstacles that get out of the sensor field of view.

Related to autonomous navigation:
* [mrpt_reactivenav2d](mrpt_reactivenav2d): A pure reactive navigator for polygonal robots on 2D worlds.
* [mrpt_tps_astar_planner](mrpt_tps_astar_planner): A path planner based on PTG trajectories using A* in a SE(2) lattice.

Others:
* [mrpt_rawlog](mrpt_rawlog): Nodes and CLI tools to convert between MRPT rawlog format and ROS rosbag2.
* [mrpt_tutorials](mrpt_tutorials): Launch and configuration files for the various examples provided for the other packages.
* [mrpt_msgs_bridge](mrpt_msgs_bridge): C++ library to convert between custom [mrpt_msgs](https://github.com/mrpt-ros-pkg/mrpt_msgs) messages and native MRPT classes
* [mrpt_nav_interfaces](mrpt_nav_interfaces): Definition of msgs, srvs, and actions used by the other packages.


General documentation
----------------------------------
* ROS wiki: http://wiki.ros.org/mrpt_navigation
* Compiling instructions: http://wiki.ros.org/mrpt_navigation/Tutorials/Installing
* Usage examples and tutorials: http://wiki.ros.org/mrpt_navigation/Tutorials
* Branches:
  * `ros2`: The most recent, active branch for modern ROS 2 distributions.
  * `ros1`: Intended for ROS 1. No further development will happen there.

Individual package build status
---------------------------------

| Package | ROS 2 Humble <br/> BinBuild |  ROS 2 Iron <br/> BinBuild | ROS 2 Jazzy <br/> BinBuild |  ROS 2 Rolling <br/> BinBuild |
| --- | --- | --- | --- | --- |
| mrpt_map_server | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mrpt_map_server__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mrpt_map_server__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ibin_uJ64__mrpt_map_server__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Ibin_uJ64__mrpt_map_server__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__mrpt_map_server__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mrpt_map_server__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_uN64__mrpt_map_server__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mrpt_map_server__ubuntu_noble_amd64__binary/) |
| mrpt_msgs_bridge | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mrpt_msgs_bridge__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mrpt_msgs_bridge__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ibin_uJ64__mrpt_msgs_bridge__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Ibin_uJ64__mrpt_msgs_bridge__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__mrpt_msgs_bridge__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mrpt_msgs_bridge__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_uN64__mrpt_msgs_bridge__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mrpt_msgs_bridge__ubuntu_noble_amd64__binary/) |
| mrpt_nav_interfaces | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mrpt_nav_interfaces__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mrpt_nav_interfaces__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ibin_uJ64__mrpt_nav_interfaces__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Ibin_uJ64__mrpt_nav_interfaces__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__mrpt_nav_interfaces__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mrpt_nav_interfaces__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_uN64__mrpt_nav_interfaces__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mrpt_nav_interfaces__ubuntu_noble_amd64__binary/) |
| mrpt_navigation | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mrpt_navigation__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mrpt_navigation__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ibin_uJ64__mrpt_navigation__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Ibin_uJ64__mrpt_navigation__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__mrpt_navigation__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mrpt_navigation__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_uN64__mrpt_navigation__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mrpt_navigation__ubuntu_noble_amd64__binary/) |
| mrpt_pf_localization | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mrpt_pf_localization__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mrpt_pf_localization__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ibin_uJ64__mrpt_pf_localization__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Ibin_uJ64__mrpt_pf_localization__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__mrpt_pf_localization__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mrpt_pf_localization__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_uN64__mrpt_pf_localization__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mrpt_pf_localization__ubuntu_noble_amd64__binary/) |
| mrpt_pointcloud_pipeline | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mrpt_pointcloud_pipeline__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mrpt_pointcloud_pipeline__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ibin_uJ64__mrpt_pointcloud_pipeline__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Ibin_uJ64__mrpt_pointcloud_pipeline__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__mrpt_pointcloud_pipeline__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mrpt_pointcloud_pipeline__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_uN64__mrpt_pointcloud_pipeline__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mrpt_pointcloud_pipeline__ubuntu_noble_amd64__binary/) |
| mrpt_rawlog | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mrpt_rawlog__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mrpt_rawlog__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ibin_uJ64__mrpt_rawlog__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Ibin_uJ64__mrpt_rawlog__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__mrpt_rawlog__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mrpt_rawlog__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_uN64__mrpt_rawlog__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mrpt_rawlog__ubuntu_noble_amd64__binary/) |
| mrpt_reactivenav2d | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mrpt_reactivenav2d__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mrpt_reactivenav2d__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ibin_uJ64__mrpt_reactivenav2d__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Ibin_uJ64__mrpt_reactivenav2d__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__mrpt_reactivenav2d__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mrpt_reactivenav2d__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_uN64__mrpt_reactivenav2d__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mrpt_reactivenav2d__ubuntu_noble_amd64__binary/) |
| mrpt_tps_astar_planner | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mrpt_tps_astar_planner__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mrpt_tps_astar_planner__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ibin_uJ64__mrpt_tps_astar_planner__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Ibin_uJ64__mrpt_tps_astar_planner__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__mrpt_tps_astar_planner__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mrpt_tps_astar_planner__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_uN64__mrpt_tps_astar_planner__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mrpt_tps_astar_planner__ubuntu_noble_amd64__binary/) |
| mrpt_tutorials | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mrpt_tutorials__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mrpt_tutorials__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ibin_uJ64__mrpt_tutorials__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Ibin_uJ64__mrpt_tutorials__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__mrpt_tutorials__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mrpt_tutorials__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_uN64__mrpt_tutorials__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mrpt_tutorials__ubuntu_noble_amd64__binary/) |


Contributing
----------------------------------
* Code formatting: We use clang-format to ensure formatting consistency in the
  code base. Set up your IDE to automatically use clang-format-11,
  use `git clang-format-11`, or invoke it manually from the root directory as:
  
      find . -iname *.hpp -o -iname *.cpp -o -iname *.h | xargs clang-format-11 -i

**Contributors**

<a href="https://github.com/mrpt-ros-pkg/mrpt_navigation/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=mrpt-ros-pkg/mrpt_navigation" />
</a>
