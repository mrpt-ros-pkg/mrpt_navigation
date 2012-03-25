
#include "mrpt_bridge/laser_scan.h"
#include <ros/console.h>

namespace mrpt_bridge {

bool laser_scan::ros2mrpt(
	const sensor_msgs::LaserScan &msg,
	mrpt::slam::CObservation2DRangeScan  &obj
	)
{
	MRPT_TODO("Implement me!")

	return true;
}

bool laser_scan::mrpt2ros(
	const mrpt::slam::CObservation2DRangeScan  &obj,
	const std_msgs::Header &msg_header,
	sensor_msgs::LaserScan &msg
	)
{
	MRPT_TODO("Implement me!")

	return true;
}

} // end namespace
