
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include "mrpt_bridge/time.h"
#include "mrpt_bridge/pose.h"
#include "mrpt_bridge/laser_scan.h"

#include <mrpt/version.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
using namespace mrpt::obs;

namespace mrpt_bridge
{
bool convert(
	const sensor_msgs::LaserScan& _msg, const mrpt::poses::CPose3D& _pose,
	CObservation2DRangeScan& _obj)
{
	mrpt_bridge::convert(_msg.header.stamp, _obj.timestamp);
	_obj.rightToLeft = true;
	_obj.sensorLabel = _msg.header.frame_id;
	_obj.aperture = _msg.angle_max - _msg.angle_min;
	_obj.maxRange = _msg.range_max;
	_obj.sensorPose = _pose;

	ASSERT_(_msg.ranges.size() > 1);

	const size_t N = _msg.ranges.size();
	const double ang_step = _obj.aperture / (N - 1);
	const double fov05 = 0.5 * _obj.aperture;
	const double inv_ang_step = (N - 1) / _obj.aperture;

	_obj.resizeScan(N);
	for (std::size_t i_mrpt = 0; i_mrpt < N; i_mrpt++)
	{
		// ROS indices go from _msg.angle_min to _msg.angle_max, while
		// in MRPT they go from -FOV/2 to +FOV/2.
		int i_ros =
			inv_ang_step * (-fov05 - _msg.angle_min + ang_step * i_mrpt);
		if (i_ros < 0)
			i_ros += N;
		else if (i_ros >= (int)N)
			i_ros -= N;  // wrap around 2PI...

		// set the scan
		const float r = _msg.ranges[i_ros];
		_obj.setScanRange(i_mrpt, r);

		// set the validity of the scan
		const bool r_valid =
			((_obj.scan[i_mrpt] < (_msg.range_max * 0.95)) &&
			 (_obj.scan[i_mrpt] > _msg.range_min));
		_obj.setScanRangeValidity(i_mrpt, r_valid);
	}

	return true;
}

bool convert(const CObservation2DRangeScan& _obj, sensor_msgs::LaserScan& _msg)
{
	const size_t nRays = _obj.scan.size();
	if (!nRays) return false;

	ASSERT_EQUAL_(_obj.scan.size(), _obj.validRange.size());

	_msg.angle_min = -0.5f * _obj.aperture;
	_msg.angle_max = 0.5f * _obj.aperture;
	_msg.angle_increment = _obj.aperture / (_obj.scan.size() - 1);

	// setting the following values to zero solves a rviz visualization problem
	_msg.time_increment = 0.0;  // 1./30.; // Anything better?
	_msg.scan_time = 0.0;  // _msg.time_increment; // idem?

	_msg.range_min = 0.02;
	_msg.range_max = _obj.maxRange;

	_msg.ranges.resize(nRays);
	for (size_t i = 0; i < nRays; i++) _msg.ranges[i] = _obj.scan[i];

	// Set header data:
	mrpt_bridge::convert(_obj.timestamp, _msg.header.stamp);
	_msg.header.frame_id = _obj.sensorLabel;

	return true;
}

bool convert(
	const CObservation2DRangeScan& _obj, sensor_msgs::LaserScan& _msg,
	geometry_msgs::Pose& _pose)
{
	convert(_obj, _msg);
	mrpt::poses::CPose3D pose;
	_obj.getSensorPose(pose);
	convert(pose, _pose);
	return true;
}
}  // end namespace
