#ifndef MRPT_BRIDGE_LASER_SCAN_H
#define MRPT_BRIDGE_LASER_SCAN_H

#include <stdint.h>
#include <string>

namespace std
{
template <class T>
class allocator;
}

namespace geometry_msgs
{
template <class ContainerAllocator>
struct Pose_;
typedef Pose_<std::allocator<void>> Pose;
}

namespace sensor_msgs
{
template <class ContainerAllocator>
struct LaserScan_;
typedef LaserScan_<std::allocator<void>> LaserScan;
}

namespace mrpt
{
namespace poses
{
class CPose3D;
}
}
#include <mrpt/version.h>
namespace mrpt
{
namespace obs
{
class CObservation2DRangeScan;
}
}

namespace mrpt_bridge
{
/** @name LaserScan: ROS <-> MRPT
 *  @{ */

/** ROS->MRPT: Takes a sensor_msgs::LaserScan and the relative pose of the laser
 * wrt base_link and builds a CObservation2DRangeScan
  * \return true on sucessful conversion, false on any error.
  * \sa mrpt2ros
  */
bool convert(
	const sensor_msgs::LaserScan& _msg, const mrpt::poses::CPose3D& _pose,
	mrpt::obs::CObservation2DRangeScan& _obj);

/** MRPT->ROS: Takes a CObservation2DRangeScan and outputs range data in
 * sensor_msgs::LaserScan
  * \return true on sucessful conversion, false on any error.
  * \sa ros2mrpt
  */
bool convert(
	const mrpt::obs::CObservation2DRangeScan& _obj,
	sensor_msgs::LaserScan& _msg);

/** MRPT->ROS: Takes a CObservation2DRangeScan and outputs range data in
 * sensor_msgs::LaserScan + the relative pose of the laser wrt base_link
  * \return true on sucessful conversion, false on any error.
  * \sa ros2mrpt
  */
bool convert(
	const mrpt::obs::CObservation2DRangeScan& _obj,
	sensor_msgs::LaserScan& _msg, geometry_msgs::Pose& _pose);

/** @} */

}  // namespace mrpt_bridge

#endif  // MRPT_BRIDGE_LASER_SCAN_H
