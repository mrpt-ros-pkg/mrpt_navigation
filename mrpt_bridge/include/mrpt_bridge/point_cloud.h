#ifndef MRPT_POINT_CLOUD_H
#define MRPT_POINT_CLOUD_H

#include <sensor_msgs/PointCloud.h>
#include <mrpt/slam/CSimplePointsMap.h>
#include <mrpt/slam/CColouredPointsMap.h>

namespace mrpt_bridge {

/** Methods to convert between ROS msgs and MRPT objects for point-cloud datatypes.
  */
namespace point_cloud
{
	/** Convert sensor_msgs/PointCloud -> mrpt::slam::CSimplePointsMap
	  *  CSimplePointsMap only contains (x,y,z) data, so sensor_msgs::PointCloud::channels are ignored.
	  * \return true on sucessful conversion, false on any error.
	  * \sa mrpt2ros
	  */
	bool ros2mrpt(
		const sensor_msgs::PointCloud &msg,
		mrpt::slam::CSimplePointsMap  &obj
		);

	/** Convert mrpt::slam::CSimplePointsMap -> sensor_msgs/PointCloud
	  *  The user must supply the "msg_header" field to be copied into the output message object, since that part does not appear in MRPT classes.
	  *
	  *  Since CSimplePointsMap only contains (x,y,z) data, sensor_msgs::PointCloud::channels will be empty.
	  * \return true on sucessful conversion, false on any error.
	  * \sa ros2mrpt
	  */
	bool mrpt2ros(
		const mrpt::slam::CSimplePointsMap  &obj,
		const std_msgs::Header &msg_header,
		sensor_msgs::PointCloud &msg
		);
}; // end of PointCloud
} //namespace mrpt_bridge

#endif
