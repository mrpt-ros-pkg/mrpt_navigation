# mrpt_map_server

# Table of Contents
* [Overview](#Overview)
* [Node: mrpt_map_server](#Node:-mrpt_map_server)
	* [Working rationale](#Working-rationale)
	* [ROS Parameters](#ROS-Parameters)
	* [Subscribed topics](#Subscribed-topics)
	* [Published topics](#Published-topics)
	* [Services](#services)
	* [Template ROS 2 launch files](#Template-ROS-2-launch-files)
* [Demos](#Demos)

## Overview
This package provides a ROS 2 node that publishes a static **map** for other nodes to use it.
Unlike classic ROS 1 ``map_server``, this node can publish a range of different metric maps, not only occupancy grids.

## Node: mrpt_map_server

### Working rationale
The C++ ROS 2 node loads all parameters at start up, loads the map
as requested by parameters, and publishes the metric map in the corresponding topics.
Messages are sent as transient local, so new subscribers can receive them even 
if they start afterwards.

There are **three formats** in which maps can be read:

1. The **preferred format** is as an [mp2p_icp](https://github.com/MOLAorg/mp2p_icp)'s metric map file (`*.mm`), normally generated
   via [sm2mm](https://github.com/MOLAorg/mp2p_icp/tree/master/apps/sm2mm) from a [MRPT "simplemap"](https://docs.mrpt.org/reference/latest/class_mrpt_maps_CSimpleMap.html) (``*.simplemap``)
   that comes from a SLAM session, e.g. using [mola_lidar_odometry](https://docs.mola-slam.org/latest/).

3. As a [ROS standard YAML file](https://wiki.ros.org/map_server). Here, a ``*.yaml`` file specifies the metadata of a 2D occupancy gridmap,
   which is stored as an accompanying image file. The map will be actually encapsulated into a `metric_map_t` map with layer name `map`.

4. As a [serialized](https://docs.mrpt.org/reference/latest/group_mrpt_serialization_grp.html) MRPT metric map file.
A ``*.metricmap`` file contains any of the existing 
[MRPT metric maps](https://docs.mrpt.org/reference/latest/group_mrpt_maps_grp.html)
(point clouds, grid maps, etc.), which may come from custom applications or other SLAM packages.
The map will be actually encapsulated into a `metric_map_t` map with layer name `map`.

So, whatever is the map source, this node will internally build a [`metric_map_t`](https://docs.mola-slam.org/latest/mp2p_icp_basics.html)
with one or more map layers, so it gets published in a uniform way to subscribers.

Refer to example launch files at the end of this file for examples
of usage of each of these methods.


### ROS Parameters

#### Related to determining where to read the map from
* (Option 1 above) ``mm_file`` (Default=undefined). Determine the `metric_map_t` file to load, coming from [sm2mm](https://github.com/MOLAorg/mp2p_icp/tree/master/apps/sm2mm) or any other custom user application using `mp2p_icp`.
* (Option 2 above) ``map_yaml_file`` (Default=undefined): Define this parameter to load a [ROS standard YAML file](https://wiki.ros.org/map_server) gridmap.
* (Option 3 above) ``mrpt_metricmap_file`` (Default=undefined).

#### Related to ROS published topics:
* ``frame_id`` (Default=``map``): TF frame.
* `pub_mm_topic` (Default=`mrpt_map`). Despite the map source, it will be eventually stored as a `mp2p_icp`'s `metric_map_t` (`*.mm`) structure, then each layer will be published using its **layer name** as a **topic name** and with the appropriate type
(e.g. PointCloud2, OccupancyGrid,...). Also, the whole metric map is published as a generic serialized object to the topic defined by the 
parameter `pub_mm_topic`.

### Subscribed topics
None.

### Published topics
* ``${pub_mm_topic}/metric_map`` (Default: ``mrpt_map/metric_map``) (``mrpt_msgs::msg::GenericObject``) (topic name can be changed with parameter `pub_mm_topic`).
* ``${pub_mm_topic}/<LAYER_NAME>`` (Default: ``mrpt_map/<LAYER_NAME>``) (``mrpt_msgs::msg::GenericObject``) 
* ``${pub_mm_topic}/<LAYER_NAME>_points`` (``sensor_msgs::msg::PointCloud2``), one per map layer.
* ``${pub_mm_topic}/<LAYER_NAME>_gridmap`` (``nav_msgs::msg::OccupancyGrid``)
* ``${pub_mm_topic}/<LAYER_NAME>_gridmap_metadata`` (``nav_msgs::msg::MapMetaData``)
* (... one per map layer ...)

If using options 2 or 3 above, there will be just one layer named `map`.

### Services
* ``GetLayers``: Returns the list of map layer names:

```bash
# Example usage:
ros2 service call /map_server_node/get_layers  mrpt_nav_interfaces/srv/GetLayers
requester: making request: mrpt_nav_interfaces.srv.GetLayers_Request()

response:
mrpt_nav_interfaces.srv.GetLayers_Response(layers=['map'])
```

* ``GetGridmapLayer``: Can be used to request a given map layer of type gridmap.

```bash
# Example usage:
ros2 service call /map_server_node/get_grid_layer mrpt_nav_interfaces/srv/GetGridmapLayer "layer_name:\
 'map'"
requester: making request: mrpt_nav_interfaces.srv.GetGridmapLayer_Request(layer_name='map')

response:
mrpt_nav_interfaces.srv.GetGridmapLayer_Response(valid=True, grid=nav_msgs.msg.OccupancyGrid(...
```

* ``GetPointmapLayer``: Can be used to request a given map layer of type point cloud.

### Template ROS 2 launch files

This package provides [launch/mrpt_map_server.launch.py](launch/mrpt_map_server.launch.py):

    ros2 launch mrpt_map_server mrpt_map_server.launch.py

which can be used in user projects to launch the MRPT map server node, by setting these [launch arguments](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Using-Substitutions.html):


## Demos

Launch a map server from a ROS yaml gridmap ([launch file](../mrpt_tutorials/launch/demo_map_server_gridmap_from_yaml.launch.py)):

```bash
ros2 launch mrpt_tutorials demo_map_server_gridmap_from_yaml.launch.py
```

Launch a map server from a custom `.mm` map ([launch file](../mrpt_tutorials/launch/demo_map_server_from_mm.launch.py)), 
which in the launch file is read from the environment variable `MM_FILE`, so it can be used like:
 
```bash
MM_FILE=/path/to/my/map.mm ros2 launch mrpt_tutorials demo_map_server_from_mm.launch.py
```
