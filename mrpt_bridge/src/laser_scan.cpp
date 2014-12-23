
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <mrpt/slam/CObservation2DRangeScan.h>
#include "mrpt_bridge/time.h"
#include "mrpt_bridge/pose.h"
#include "mrpt_bridge/laser_scan.h"

namespace mrpt_bridge {

bool convert(const sensor_msgs::LaserScan &_msg, const mrpt::poses::CPose3D &_pose, mrpt::slam::CObservation2DRangeScan  &_obj
                         )
{
    mrpt_bridge::convert(_msg.header.stamp, _obj.timestamp);
    _obj.rightToLeft = true;
    _obj.sensorLabel = _msg.header.frame_id;
    _obj.aperture = _msg.angle_max - _msg.angle_min;
    _obj.maxRange = _msg.range_max;
    _obj.sensorPose =  _pose;
    _obj.validRange.resize(_msg.ranges.size());
    _obj.scan.resize(_msg.ranges.size());
    for(std::size_t i = 0; i < _msg.ranges.size(); i++) {
        _obj.scan[i] = _msg.ranges[i];
        if((_obj.scan[i] < (_msg.range_max*0.95)) && (_obj.scan[i] > _msg.range_min)) {
            _obj.validRange[i] = 1;
        } else {
            _obj.validRange[i] = 0;
        }
    }

    return true;
}

bool convert(const mrpt::slam::CObservation2DRangeScan  &_obj, sensor_msgs::LaserScan &_msg) {
    const size_t nRays = _obj.scan.size();
    if (!nRays) return false;

    ASSERT_EQUAL_(_obj.scan.size(),_obj.validRange.size() )

    _msg.angle_min = -0.5f * _obj.aperture;
    _msg.angle_max =  0.5f * _obj.aperture;
    _msg.angle_increment = _obj.aperture/(_obj.scan.size()-1);

    // setting the following values to zero solves a rviz visualization problem
    _msg.time_increment = 0.0; // 1./30.; // Anything better?
    _msg.scan_time = 0.0; // _msg.time_increment; // idem?

    _msg.range_min = 0.02;
    _msg.range_max = _obj.maxRange;

    _msg.ranges.resize(nRays);
    for (size_t i=0; i<nRays; i++)
        _msg.ranges[i] = _obj.scan[i];

    // Set header data:
    mrpt_bridge::convert(_obj.timestamp, _msg.header.stamp);
    _msg.header.frame_id = _obj.sensorLabel;

    return true;
}

bool convert(const mrpt::slam::CObservation2DRangeScan  &_obj, sensor_msgs::LaserScan &_msg, geometry_msgs::Pose &_pose) {
	convert(_obj, _msg);
	mrpt::poses::CPose3D pose;
	_obj.getSensorPose(pose);
	convert(pose,_pose);
	return true;
}
} // end namespace
