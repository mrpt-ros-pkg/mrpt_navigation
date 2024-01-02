/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/time.h>

#include <mrpt_msgs_bridge/beacon.hpp>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/transform_datatypes.h"

using namespace mrpt::obs;

bool mrpt_msgs_bridge::fromROS(
	const mrpt_msgs::msg::ObservationRangeBeacon _msg,
	const mrpt::poses::CPose3D& _pose, CObservationBeaconRanges& _obj)
{
	_obj.timestamp = mrpt::ros2bridge::fromROS(_msg.header.stamp);
	mrpt::poses::CPose3D cpose_obj;

	_obj.stdError = _msg.sensor_std_range;
	_obj.sensorLabel = _msg.header.frame_id;
	_obj.maxSensorDistance = _msg.max_sensor_distance;
	_obj.minSensorDistance = _msg.min_sensor_distance;

	if (_pose.empty())
	{
		cpose_obj = mrpt::ros2bridge::fromROS(_msg.sensor_pose_on_robot);
		_obj.setSensorPose(cpose_obj);
	}
	else
	{
		_obj.setSensorPose(_pose);
	}

	ASSERT_(_msg.sensed_data.size() >= 1);
	const size_t N = _msg.sensed_data.size();

	_obj.sensedData.resize(N);

	for (std::size_t i_mrpt = 0; i_mrpt < N; i_mrpt++)
	{
		_obj.sensedData[i_mrpt].sensedDistance = _msg.sensed_data[i_mrpt].range;
		_obj.sensedData[i_mrpt].beaconID = _msg.sensed_data[i_mrpt].id;
	}
	return true;
}

bool mrpt_msgs_bridge::toROS(
	const CObservationBeaconRanges& _obj,
	mrpt_msgs::msg::ObservationRangeBeacon& _msg)
{
	mrpt::poses::CPose3D cpose_obj;

	_msg.header.stamp = mrpt::ros2bridge::toROS(_obj.timestamp);
	_obj.getSensorPose(cpose_obj);
	_msg.sensor_pose_on_robot = mrpt::ros2bridge::toROS_Pose(cpose_obj);

	_msg.sensor_std_range = _obj.stdError;
	_msg.header.frame_id = _obj.sensorLabel;
	_msg.max_sensor_distance = _obj.maxSensorDistance;
	_msg.min_sensor_distance = _obj.minSensorDistance;

	ASSERT_(_obj.sensedData.size() >= 1);
	const size_t N = _obj.sensedData.size();

	_msg.sensed_data.resize(N);

	for (std::size_t i_msg = 0; i_msg < N; i_msg++)
	{
		_msg.sensed_data[i_msg].range = _obj.sensedData[i_msg].sensedDistance;
		_msg.sensed_data[i_msg].id = _obj.sensedData[i_msg].beaconID;
	}
	return true;
}

bool mrpt_msgs_bridge::toROS(
	const CObservationBeaconRanges& _obj,
	mrpt_msgs::msg::ObservationRangeBeacon& _msg,
	geometry_msgs::msg::Pose& _pose)
{
	toROS(_obj, _msg);
	mrpt::poses::CPose3D pose;
	_obj.getSensorPose(pose);
	_pose = mrpt::ros2bridge::toROS_Pose(pose);
	return true;
}