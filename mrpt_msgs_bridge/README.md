# mrpt_msgs_bridge

## Overview

C++ library to convert between messages of
[mrpt_msgs](https://github.com/mrpt-ros-pkg/mrpt_msgs) (and, optionally,
`marker_msgs`) and the corresponding MRPT classes. For the conversion of
standard ROS 2 messages, see `mrpt::ros2bridge` in
[mrpt_ros_bridge](https://github.com/MRPT/mrpt_ros_bridge).

All functions are in the `mrpt_msgs_bridge` namespace:

| Header | ROS 2 message | MRPT class | Functions |
| --- | --- | --- | --- |
| `mrpt_msgs_bridge/beacon.hpp` | `mrpt_msgs/ObservationRangeBeacon` | `mrpt::obs::CObservationBeaconRanges` | `fromROS()`, `toROS()` |
| `mrpt_msgs_bridge/landmark.hpp` | `mrpt_msgs/ObservationRangeBearing` | `mrpt::obs::CObservationBearingRange` | `fromROS()`, `toROS()` |
| `mrpt_msgs_bridge/network_of_poses.hpp` | `mrpt_msgs/NetworkOfPoses` | `mrpt::graphs::CNetworkOfPoses{2D,3D,2DInf,3DInf}` | `fromROS()`, `toROS()` |
| `mrpt_msgs_bridge/marker_msgs.hpp` | `marker_msgs/MarkerDetection` | `CObservationBearingRange`, `CObservationBeaconRanges` | `fromROS()` |

The `marker_msgs` converters are only built if that package is found at build
time.

## Usage

In `package.xml`:

```xml
<depend>mrpt_msgs_bridge</depend>
```

In `CMakeLists.txt`:

```cmake
find_package(mrpt_msgs_bridge REQUIRED)
target_link_libraries(my_target ${mrpt_msgs_bridge_TARGETS})
```

In C++:

```cpp
#include <mrpt_msgs_bridge/beacon.hpp>

mrpt::obs::CObservationBeaconRanges obs;
mrpt_msgs_bridge::fromROS(msg, sensorPoseOnRobot, obs);
```
