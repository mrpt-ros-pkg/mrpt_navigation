#ifndef MRPT_BRIDGE_POINT_CLOUD2_H
#define MRPT_BRIDGE_POINT_CLOUD2_H

namespace std
{
template <class T>
class allocator;
}

namespace std_msgs
{
template <class ContainerAllocator>
struct Header_;
typedef Header_<std::allocator<void>> Header;
}

namespace sensor_msgs
{
template <class ContainerAllocator>
struct PointCloud2_;
typedef PointCloud2_<std::allocator<void>> PointCloud2;
}

#include <mrpt/version.h>
namespace mrpt
{
namespace maps
{
class CSimplePointsMap;
class CColouredPointsMap;
}
}

namespace mrpt_bridge
{
/** @name sensor_msgs::PointCloud2: ROS <-> MRPT
 *  @{ */

/** Convert sensor_msgs/PointCloud2 -> mrpt::slam::CSimplePointsMap
 *  CSimplePointsMap only contains (x,y,z) data, so
 * sensor_msgs::PointCloud2::channels are ignored.
 * \return true on sucessful conversion, false on any error.
 * \sa mrpt2ros
 */
bool copy(
	const sensor_msgs::PointCloud2& msg, mrpt::maps::CSimplePointsMap& obj);

/** Convert mrpt::slam::CSimplePointsMap -> sensor_msgs/PointCloud2
 *  The user must supply the "msg_header" field to be copied into the output
 * message object, since that part does not appear in MRPT classes.
 *
 *  Since CSimplePointsMap only contains (x,y,z) data,
 * sensor_msgs::PointCloud2::channels will be empty.
 * \return true on sucessful conversion, false on any error.
 * \sa ros2mrpt
 */
bool copy(
	const mrpt::maps::CSimplePointsMap& obj, const std_msgs::Header& msg_header,
	sensor_msgs::PointCloud2& msg);

/** @} */

}  // namespace mrpt_bridge

#endif  // MRPT_BRIDGE_POINT_CLOUD2_H
