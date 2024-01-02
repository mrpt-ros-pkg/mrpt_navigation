/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

/**
 * @file   marker_msgs.h
 * @author Markus Bader <markus.bader@tuwien.ac.at>
 * @date   September, 2017
 * @brief  Funtions to convert marker_msgs to mrpt msgs
 **/

#pragma once

#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/poses/CPose3D.h>

#include <geometry_msgs/msg/pose.hpp>
#include <marker_msgs/msg/marker_detection.hpp>

namespace mrpt_msgs_bridge
{
// NOTE: These converters are here instead of mrpt::ros2bridge since
//  the builds dep. MarkerDetection is not available in Debian/Ubuntu
//  official repos, so we need to build it here within ROS.

bool fromROS(
	const marker_msgs::msg::MarkerDetection& _msg,
	const mrpt::poses::CPose3D& sensorPoseOnRobot,
	mrpt::obs::CObservationBearingRange& _obj);

bool fromROS(
	const marker_msgs::msg::MarkerDetection& _msg,
	const mrpt::poses::CPose3D& sensorPoseOnRobot,
	mrpt::obs::CObservationBeaconRanges& _obj);

}  // namespace mrpt_msgs_bridge