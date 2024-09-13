/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

/**
 * @file   marker_msgs.cpp
 * @author Markus Bader <markus.bader@tuwien.ac.at>
 * @date   September, 2017
 * @brief  Funtions to convert marker_msgs to mrpt msgs
 **/

#include <mrpt/ros2bridge/time.h>

#include <mrpt_msgs_bridge/marker_msgs.hpp>

bool mrpt_msgs_bridge::fromROS(
	const marker_msgs::msg::MarkerDetection& src, const mrpt::poses::CPose3D& sensorPoseOnRobot,
	mrpt::obs::CObservationBearingRange& des)
{
	des.timestamp = mrpt::ros2bridge::fromROS(src.header.stamp);
	des.setSensorPose(sensorPoseOnRobot);
	des.minSensorDistance = src.distance_min;
	des.maxSensorDistance = src.distance_max;

	des.sensedData.resize(src.markers.size());
	for (size_t i = 0; i < src.markers.size(); i++)
	{
		const marker_msgs::msg::Marker& marker = src.markers[i];
		// mrpt::obs::CObservationBearingRange::TMeasurement
		auto& m = des.sensedData[i];
		m.range = sqrt(
			marker.pose.position.x * marker.pose.position.x +
			marker.pose.position.y * marker.pose.position.y);
		m.yaw = atan2(marker.pose.position.y, marker.pose.position.x);
		m.pitch = 0.0;
		if (marker.ids.size() > 0)
		{
			m.landmarkID = marker.ids[0];
		}
		else
		{
			m.landmarkID = -1;
		}
	}
	return true;
}

bool mrpt_msgs_bridge::fromROS(
	const marker_msgs::msg::MarkerDetection& src, const mrpt::poses::CPose3D& sensorPoseOnRobot,
	mrpt::obs::CObservationBeaconRanges& des)
{
	des.timestamp = mrpt::ros2bridge::fromROS(src.header.stamp);

	des.setSensorPose(sensorPoseOnRobot);
	des.minSensorDistance = src.distance_min;
	des.maxSensorDistance = src.distance_max;

	des.sensedData.resize(src.markers.size());
	for (size_t i = 0; i < src.markers.size(); i++)
	{
		const marker_msgs::msg::Marker& marker = src.markers[i];
		// mrpt::obs::CObservationBeaconRanges::TMeasurement
		auto& m = des.sensedData[i];
		m.sensedDistance = sqrt(
			marker.pose.position.x * marker.pose.position.x +
			marker.pose.position.y * marker.pose.position.y);
		m.sensorLocationOnRobot.m_coords = sensorPoseOnRobot.m_coords;
		if (marker.ids.size() > 0)
		{
			m.beaconID = marker.ids[0];
		}
		else
		{
			m.beaconID = -1;
		}
	}
	return true;
}