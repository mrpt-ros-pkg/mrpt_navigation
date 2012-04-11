#ifndef MRPT_POINT_CLOUD_H
#define MRPT_POINT_CLOUD_H

#include <sensor_msgs/PointCloud2.h>
#include <mrpt/slam/CSimplePointsMap.h>
#include <mrpt/slam/CColouredPointsMap.h>

namespace mrpt_bridge
{

/** This class contains methods to convert between ROS msgs and MRPT objects for point-cloud datatypes.
 * \note Methods are not static, so you will need to instantiate an object of this converter class. This is done on purpose so options and switches can be added to the class in the future.
 */
namespace PointCloud2
{

/** Convert sensor_msgs/PointCloud2 -> mrpt::slam::CSimplePointsMap
 *  CSimplePointsMap only contains (x,y,z) data, so sensor_msgs::PointCloud2::channels are ignored.
 * \return true on sucessful conversion, false on any error.
 * \sa mrpt2ros
 */
bool ros2mrpt(const sensor_msgs::PointCloud2 &msg, mrpt::slam::CSimplePointsMap &obj);

/** Convert mrpt::slam::CSimplePointsMap -> sensor_msgs/PointCloud2
 *  The user must supply the "msg_header" field to be copied into the output message object, since that part does not appear in MRPT classes.
 *
 *  Since CSimplePointsMap only contains (x,y,z) data, sensor_msgs::PointCloud2::channels will be empty.
 * \return true on sucessful conversion, false on any error.
 * \sa ros2mrpt
 */
bool mrpt2ros(const mrpt::slam::CSimplePointsMap &obj, const std_msgs::Header &msg_header,
              sensor_msgs::PointCloud2 &msg);
}
;
// end of PointCloud
}//namespace mrpt_bridge

#endif
