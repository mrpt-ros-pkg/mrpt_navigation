# mrpt_map

* [mrpt_map](#mrpt_map)
   * [Overview](#overview)
   * [Node: mrpt_map_server](#node-mrpt_map_server)
      * [Working rationale](#working-rationale)
      * [ROS Parameters](#ros-parameters)
      * [Subscribed topics](#subscribed-topics)
      * [Published topics](#published-topics)
      * [Template ROS 2 launch files](#template-ros-2-launch-files)

<!-- Created by https://github.com/ekalinin/github-markdown-toc -->
<!-- Regenerate with: gh-md-toc README.md -->

## Overview
This package provides a ROS 2 node that publishes a static **map** for other nodes to use it.
Unlike classic ROS 1 ``map_server``, this node can publish a range of different metric maps, not only occupancy grids.

## Node: mrpt_map_server

### Working rationale
The C++ ROS 2 node loads all parameters at start up, loads the map
as requested by parameters, and publishes the metric map in the corresponding topics. It is also possible to change the map via ROS services.

There are **three formats** in which maps can be read:

1. As a [ROS standard YAML file](https://wiki.ros.org/map_server). Here, a ``*.yaml`` file specifies the metadata of a 2D occupancy gridmap, which is stored as an accompanying image file.

2. As a [serialized](https://docs.mrpt.org/reference/latest/group_mrpt_serialization_grp.html) MRPT metric map file.
A ``*.metricmap`` file contains any of the existing 
[MRPT metric maps](https://docs.mrpt.org/reference/latest/group_mrpt_maps_grp.html)
(point clouds, grid maps, etc.), which may come from custom 
applications or other SLAM packages.

3. As an [MRPT "simplemap"](https://docs.mrpt.org/reference/latest/class_mrpt_maps_CSimpleMap.html) ``*.simplemap`` which contains
raw sensory data, together with a metric map specification
in an [multi-metric map](https://docs.mrpt.org/reference/latest/tutorial-mrpt-maps-model.html)
``*.ini`` file. The former typically
comes from an MRPT SLAM package. The later, then can be used
to build different metric maps (octomaps, gridmaps, pointclouds, etc.) from the same original SLAM result.

Refer to example launch files at the end of this file for examples
of usage of each of these methods.


### ROS Parameters
* ``map_yaml_file`` (Default=undefined): Define this parameter to load a [ROS standard YAML file](https://wiki.ros.org/map_server) gridmap (option 1 above).
* ``ini_file`` and ``map_file`` (Default=undefined): Define these two parameters to use option 2 above.
* ``mrpt_metricmap_file`` (Default=undefined): Define this parameter to use option 3 above (TODO).
* ``frame_id`` (Default=``map``): TF frame.

* ``pub_map_ros`` (Default=``map``) and ``pub_metadata`` (Default=``map_metadata``): The topic names to which to publish the occupancy gridmap and its metadata. In case of the read map being
an MRPT multimetric map, the first occupancy grid will be published here.

### Subscribed topics
None.

### Published topics
* ``map`` (``nav_msgs::msg::OccupancyGrid``)
* ``pub_metadata`` (``nav_msgs::msg::MapMetaData``)

### Template ROS 2 launch files
* xxx

