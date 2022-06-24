/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/**
 * @file   marker_msgs.h
 * @author Markus Bader <markus.bader@tuwien.ac.at>
 * @date   September, 2017
 * @brief  Funtions to convert marker_msgs to mrpt msgs
 **/

#pragma once

#include <geometry_msgs/Pose.h>
#include <marker_msgs/MarkerDetection.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/poses/CPose3D.h>

namespace mrpt_msgs_bridge
{
// NOTE: These converters are here instead of mrpt::ros1bridge since
//  the builds dep. MarkerDetection is not available in Debian/Ubuntu
//  official repos, so we need to build it here within ROS.

bool fromROS(
	const marker_msgs::MarkerDetection& _msg,
	const mrpt::poses::CPose3D& sensorPoseOnRobot,
	mrpt::obs::CObservationBearingRange& _obj);

bool fromROS(
	const marker_msgs::MarkerDetection& _msg,
	const mrpt::poses::CPose3D& sensorPoseOnRobot,
	mrpt::obs::CObservationBeaconRanges& _obj);

}  // namespace mrpt_msgs_bridge
