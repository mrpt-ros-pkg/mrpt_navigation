# mrpt_nav_interfaces

Definitions of the messages, services, and actions used by the
`mrpt_navigation` packages.

## Actions

| Action | Description | Servers |
| --- | --- | --- |
| [NavigateGoal](action/NavigateGoal.action) | Navigate to a target pose. | `mrpt_tps_astar_planner`, `mrpt_reactivenav2d` |
| [NavigateWaypoints](action/NavigateWaypoints.action) | Navigate through a sequence of waypoints (`mrpt_msgs/WaypointSequence`), optionally waiting (instead of detouring) when blocked. | `mrpt_tps_astar_planner`, `mrpt_reactivenav2d` |

Both report `NavigationFeedback` while running, and end with a
`NavigationFinalStatus`.

## Services

| Service | Description | Servers |
| --- | --- | --- |
| [MakePlanTo](srv/MakePlanTo.srv) | Plan a path from the current robot pose (from `/tf`) to a target pose. | `mrpt_tps_astar_planner` |
| [MakePlanFromTo](srv/MakePlanFromTo.srv) | Plan a path between two given poses. | `mrpt_tps_astar_planner` |
| [GetLayers](srv/GetLayers.srv) | List the layers of the loaded metric map. | `mrpt_map_server` |
| [GetGridmapLayer](srv/GetGridmapLayer.srv) | Get a map layer as a `nav_msgs/OccupancyGrid`. | `mrpt_map_server` |
| [GetPointmapLayer](srv/GetPointmapLayer.srv) | Get a map layer as a `sensor_msgs/PointCloud2`. | `mrpt_map_server` |

The planning services accept optional polygons where obstacles are ignored.

## Messages

| Message | Description |
| --- | --- |
| [NavigationStatus](msg/NavigationStatus.msg) | Live status of a navigator: state, reason of the latest event, initialization progress, current goal and its progress, waypoint counters, and follower status. |
| [NavigationFeedback](msg/NavigationFeedback.msg) | Feedback of the navigation actions. |
| [NavigationFinalStatus](msg/NavigationFinalStatus.msg) | Result of the navigation actions (success, error, canceled), with a reason and message. |
| [ReferencePath](msg/ReferencePath.msg) | A path from a planner that keeps updating it (goal id, revision, partial flag), for `mrpt_trajectory_follower`. |
| [FollowerStatus](msg/FollowerStatus.msg) | Status of a path follower, with the goal id and revision of the path it refers to. |
| [GeoreferencingMetadata](msg/GeoreferencingMetadata.msg) | Georeferencing of a map (ENU, UTM, and geodetic coordinates), published by `mrpt_map_server`. |
