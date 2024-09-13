/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

/*
 * File: landmark.cpp
 * Author: Vladislav Tananaev
 *
 */

#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/time.h>

#include <mrpt_msgs_bridge/landmark.hpp>

bool mrpt_msgs_bridge::fromROS(
	const mrpt_msgs::msg::ObservationRangeBearing& _msg, const mrpt::poses::CPose3D& _pose,
	mrpt::obs::CObservationBearingRange& _obj)

{
	_obj.timestamp = mrpt::ros2bridge::fromROS(_msg.header.stamp);

	mrpt::poses::CPose3D cpose_obj;

	//_obj.stdError = _msg.sensor_std_range;
	//_obj.sensorLabel = _msg.header.frame_id;
	_obj.maxSensorDistance = _msg.max_sensor_distance;
	_obj.minSensorDistance = _msg.min_sensor_distance;
	_obj.sensor_std_yaw = _msg.sensor_std_yaw;
	_obj.sensor_std_pitch = _msg.sensor_std_pitch;
	_obj.sensor_std_range = _msg.sensor_std_range;

	if (_pose.empty())
	{
		_obj.setSensorPose(mrpt::ros2bridge::fromROS(_msg.sensor_pose_on_robot));
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
		_obj.sensedData[i_mrpt].range = _msg.sensed_data[i_mrpt].range;
		_obj.sensedData[i_mrpt].landmarkID = _msg.sensed_data[i_mrpt].id;
		_obj.sensedData[i_mrpt].yaw = _msg.sensed_data[i_mrpt].yaw;
		_obj.sensedData[i_mrpt].pitch = _msg.sensed_data[i_mrpt].pitch;
	}
	return true;
}

bool mrpt_msgs_bridge::toROS(
	const mrpt::obs::CObservationBearingRange& _obj, mrpt_msgs::msg::ObservationRangeBearing& _msg)
{
	_msg.header.stamp = mrpt::ros2bridge::toROS(_obj.timestamp);

	_msg.sensor_pose_on_robot = mrpt::ros2bridge::toROS_Pose(_obj.sensorPose());

	_msg.max_sensor_distance = _obj.maxSensorDistance;
	_msg.min_sensor_distance = _obj.minSensorDistance;
	_msg.sensor_std_yaw = _obj.sensor_std_yaw;
	_msg.sensor_std_pitch = _obj.sensor_std_pitch;
	_msg.sensor_std_range = _obj.sensor_std_range;

	ASSERT_(_obj.sensedData.size() >= 1);
	const size_t N = _obj.sensedData.size();

	_msg.sensed_data.resize(N);

	for (std::size_t i_msg = 0; i_msg < N; i_msg++)
	{
		_msg.sensed_data[i_msg].range = _obj.sensedData[i_msg].range;
		_msg.sensed_data[i_msg].id = _obj.sensedData[i_msg].landmarkID;
		_msg.sensed_data[i_msg].yaw = _obj.sensedData[i_msg].yaw;
		_msg.sensed_data[i_msg].pitch = _obj.sensedData[i_msg].pitch;
	}
	return true;
}

bool mrpt_msgs_bridge::toROS(
	const mrpt::obs::CObservationBearingRange& _obj, mrpt_msgs::msg::ObservationRangeBearing& _msg,
	geometry_msgs::msg::Pose& sensorPose)
{
	toROS(_obj, _msg);
	sensorPose = mrpt::ros2bridge::toROS_Pose(_obj.sensorPose());
	return true;
}