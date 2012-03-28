
#include "mrpt_bridge/laser_scan.h"
#include <ros/console.h>

namespace mrpt_bridge {

bool laser_scan::ros2mrpt(
	const sensor_msgs::LaserScan &msg,
	mrpt::slam::CObservation2DRangeScan  &obj
	)
{
	MRPT_TODO("Implement me!")
	throw std::runtime_error("Not implemented yet!");

	return true;
}

bool laser_scan::mrpt2ros(
	const mrpt::slam::CObservation2DRangeScan  &obj,
	const std_msgs::Header &msg_header,
	sensor_msgs::LaserScan &msg
	)
{
	const size_t nRays = obj.scan.size();
	if (!nRays) return false;

	ASSERT_EQUAL_(obj.scan.size(),obj.validRange.size() )

    msg.angle_min = -0.5f*obj.aperture;
    msg.angle_max =  0.5f*obj.aperture;
    msg.angle_increment = obj.aperture/(obj.scan.size()-1);

    msg.time_increment = 1./30.; // Anything better?
    msg.scan_time = msg.time_increment; // idem?

    msg.range_min = 0.02;
    msg.range_max = obj.maxRange;

    msg.ranges.resize(nRays);
    for (size_t i=0;i<nRays;i++)
		msg.ranges[i] = obj.scan[i];

	// Set header data:
	msg.header.stamp = msg_header.stamp;
	msg.header.frame_id = msg_header.frame_id;

	return true;
}

} // end namespace
