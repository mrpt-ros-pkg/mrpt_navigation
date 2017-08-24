
#include "mrpt_bridge/point_cloud.h"
#include <ros/console.h>

namespace mrpt_bridge
{
/** Convert sensor_msgs/PointCloud -> mrpt::slam::CSimplePointsMap
  *
  * \return true on sucessful conversion, false on any error.
  */
bool point_cloud::ros2mrpt(
	const sensor_msgs::PointCloud& msg, CSimplePointsMap& obj)
{
	const size_t N = msg.points.size();

	obj.clear();
	obj.reserve(N);
	for (size_t i = 0; i < N; i++)
		obj.insertPoint(msg.points[i].x, msg.points[i].y, msg.points[i].z);

	return true;
}

/** Convert mrpt::maps::CSimplePointsMap -> sensor_msgs/PointCloud
  *  The user must supply the "msg_header" field to be copied into the output
 * message object, since that part does not appear in MRPT classes.
  *
  *  Since CSimplePointsMap only contains (x,y,z) data,
 * sensor_msgs::PointCloud::channels will be empty.
  * \return true on sucessful conversion, false on any error.
  */
bool point_cloud::mrpt2ros(
	const CSimplePointsMap& obj, const std_msgs::Header& msg_header,
	sensor_msgs::PointCloud& msg)
{
	// 1) sensor_msgs::PointCloud:: header
	msg.header = msg_header;

	// 2) sensor_msgs::PointCloud:: points
	const size_t N = obj.size();
	msg.points.resize(N);
	for (size_t i = 0; i < N; i++)
	{
		geometry_msgs::Point32& pt = msg.points[i];
		obj.getPoint(i, pt.x, pt.y, pt.z);
	}

	// 3) sensor_msgs::PointCloud:: channels
	msg.channels.clear();

	return true;
}

}  // namespace mrpt_bridge
