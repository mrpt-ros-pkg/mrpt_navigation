# mrpt_pf_localization

* [mrpt_pf_localization](#mrpt_pf_localization)
   * [Overview](#overview)
   * [Related papers](#related-papers)
   * [Demos](#demos)
      * [2D LIDAR localization with a gridmap](#2d-lidar-localization-with-a-gridmap)
      * [Range-only (RO) localization with a set of fixed, known radio beacons](#range-only-ro-localization-with-a-set-of-fixed-known-radio-beacons)
   * [Node: mrpt_pf_localization](#node-mrpt_pf_localization)
      * [Working rationale](#working-rationale)
      * [ROS 2 parameters](#ros-2-parameters)
      * [Subscribed topics](#subscribed-topics)
      * [Published topics](#published-topics)
      * [Template ROS 2 launch files](#template-ros-2-launch-files)

<!-- Created by https://github.com/ekalinin/github-markdown-toc -->
<!-- Regenerate with: gh-md-toc README.md -->

## Overview
This package provides a ROS 2 node for self-localization using 2D or 3D (SE(2) or SE(3))
particle filter-based algorithms and a number of different metric maps as reference maps
to which to "compare" sensor observations.

In a sense, this package is an equivalent to the classic ROS 1 ``amcl``, but with superpowers :-)

Features:

* A number of [different PF algorithms](https://www.mrpt.org/tutorials/programming/statistics-and-bayes-filtering/particle_filter_algorithms/).

* Different map types: Occupancy grid maps (as images, ROS yaml files, or in MRPT binary format), point clouds, beacon map (for range-only sensors). At present, these combinations are exposed in this node:
  * Map: **occupancy grid**, Sensor: anyone capable of generating a point cloud. Several occupancy grids, each at a different height, to be used for laser scans at the corresponding robot height.
  * Map: **beacons** at predefined 3D positions, Sensor: range-only. For Range-Only (RO) Localization.
  * Map: **point cloud**, Sensor: 2D or 3D Lidars (TO-DO as of Aug 2023).
  * **GNSS** (GPS) readings, in parallel to any of the above (TO-DO as of Aug 2023).

* Multiple simultaneous sensors: The combinations above can be used together and their probabilistic information 
automatically **fused together**.

<img src="https://mrpt.github.io/imgs/ros-pf-localization-pioneer.jpg" style="width: 500px; align: center;" />

## Related papers
* Optimal particle filtering algorithm:

    J.L. Blanco, J. Gonzalez-Jimenez, J.A. Fernandez-Madrigal, "Optimal Filtering for Non-Parametric Observation Models: Applications to Localization and SLAM", The International Journal of Robotics Research (IJRR), vol. 29, no. 14, 2010. ([PDF](https://ingmec.ual.es/~jlblanco/papers/blanco2010ofn_IJRR.pdf))

* Range-Only localization:

    J. Gonzalez-Jimenez, J.L. Blanco, C. Galindo, A. Ortiz-de-Galisteo, J.A. Fernandez-Madrigal, F.A. Moreno, J. Martinez, "Mobile Robot Localization based on Ultra-Wide-Band Ranging: A Particle Filter Approach", Robotics and Autonomous Systems, vol. 57, no. 5, pp. 496--507, 2009. ([PDF](https://ingmec.ual.es/~jlblanco/papers/gonzalez2008mrl.pdf))


## Demos

### 2D LIDAR localization with a gridmap
Run:

    ros2 launch mrpt_pf_localization demo.launch

to start:

* a rosbag including tf and laser scan data,
* the mrpt localization with a mrpt map
* RViz for visualization

### Range-only (RO) localization with a set of fixed, known radio beacons
Run:

    ros2 launch mrpt_localization demo_ro.launch

to start:

* a dataset (rawlog format) including RO and odometry observations,
* the mrpt localization with known beacon locations, and
* RViz for visualization


## Node: mrpt_pf_localization

### Working rationale

The C++ ROS 2 node comprises an internal, independent ``PFLocalizationCore`` C++ class, which implements
the main functionality. It features an internal finite state machine (FSM) with these states:

* ``UNINITIALIZED``: The filter has been neither initialized nor parameters/map loaded. 
  State after initialization. "Loops" in this state do nothing.
* ``TO_BE_INITIALIZED``: Once the parameters have been loaded, and a map has been loaded 
  (or if subscribed to a map topic, the topic data has been received), the ``PFLocalizationCore`` 
  is put into this state by the node. Upon next "loop", the particles and data structures will be initialized.
* ``RUNNING``: Normal state. At each "loop", odometry (if present) is used together with sensors to
  localize the robot.


### ROS 2 parameters

There is a core set of parameters to configure the particle filter algorithm itself,
which is self-documented in the provided template parameters yaml file:
[params/default.config.yaml](params/default.config.yaml).
Please, read that file and its comments for details.

If the ``initial_pose`` parameter is provided, and there is an occupancy gridmap, particles will be distributed
along free space cells only. Otherwise, they will be distributed in the box that circumscribes the confidence interval
of mean ±1 sigma of the uncertainty.


### Subscribed topics
* xxx

### Published topics
* xxx

### Template ROS 2 launch files

This package provides [launch/localization.launch.py](launch/localization.launch.py):

    ros2 launch mrpt_pf_localization localization.launch.py

which can be used in user projects to launch the MRPT PF localization node, by setting this [launch arguments](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Using-Substitutions.html):

* ``mrpt_map_config_file`` and ``mrpt_simplemap_file``: If used together, they select an MRPT map to be loaded as
  reference map for the localization algorithm. These two files work as described in ["format 2"](https://github.com/mrpt-ros-pkg/mrpt_navigation/tree/ros2/mrpt_map#working-rationale) in the docs for ``mrpt_map``.
* ``pf_params_file`` (Default: [params/default.config.yaml](params/default.config.yaml)): If defined, overrides the default
particle filter algorithm. Note that probabilistic likelihood models for the metric maps are in an independent file (``mrpt_map_config_file``).


