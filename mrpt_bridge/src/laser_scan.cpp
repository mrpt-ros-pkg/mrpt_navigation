
#include "mrpt_bridge/time.h"
#include "mrpt_bridge/laser_scan.h"
#include <ros/console.h>

namespace mrpt_bridge {

bool laser_scan::ros2mrpt(
	const sensor_msgs::LaserScan &msg,
    const mrpt::poses::CPose3D &pose,
	mrpt::slam::CObservation2DRangeScan  &obj
	)
{
    mrpt_bridge::time::ros2mrpt(msg.header.stamp, obj.timestamp);
    obj.rightToLeft = true;
    obj.sensorLabel = "FLASER";
    obj.aperture = msg.angle_max - msg.angle_min;
    obj.maxRange = msg.range_max;
    obj.sensorPose =  pose;
    obj.validRange.resize(msg.ranges.size());
    obj.scan.resize(msg.ranges.size());
    for(std::size_t i = 0; i < msg.ranges.size(); i++) {
        obj.scan[i] = msg.ranges[i];
        if((obj.scan[i] < (msg.range_max*0.95)) && (obj.scan[i] > msg.range_min)) {
            obj.validRange[i] = 1;
        } else {
            obj.validRange[i] = 0;
        }
    }
    
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
