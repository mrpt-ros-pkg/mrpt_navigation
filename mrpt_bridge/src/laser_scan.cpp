
#include "mrpt_bridge/laser_scan.h"
#include <ros/console.h>

namespace mrpt_bridge {

bool LaserScan::ros2mrpt(
	const sensor_msgs::LaserScan &msg,
	mrpt::slam::CObservation2DRangeScan  &obj
	)
{

	return true;
}


} // end namespace
