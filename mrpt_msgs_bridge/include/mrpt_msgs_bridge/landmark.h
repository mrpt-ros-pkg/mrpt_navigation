/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

/*
 * File: landmark.h
 * Author: Vladislav Tananaev
 *
 */

#pragma once

#include <cstdint>
#include <string>
#include <geometry_msgs/Pose.h>
#include <mrpt_msgs/ObservationRangeBearing.h>
#include <mrpt/obs/CObservationBearingRange.h>

namespace mrpt_msgs_bridge
{
/** @name LaserScan: ROS <-> MRPT
 *  @{ */

/** ROS->MRPT: Takes a mrpt_msgs::CObservationBearingRange and the relative pose
 * of the sensor wrt base_link and builds a ObservationRangeBearing
 * \return true on sucessful conversion, false on any error.
 * \sa mrpt2ros
 */
bool fromROS(
	const mrpt_msgs::ObservationRangeBearing& _msg,
	const mrpt::poses::CPose3D& _pose, mrpt::obs::CObservationBearingRange& _obj
);

/** MRPT->ROS: Takes a CObservationBearingRange and outputs range data in
 * mrpt_msgs::ObservationRangeBearing
 * \return true on sucessful conversion, false on any error.
 * \sa ros2mrpt
 */
bool toROS(
	const mrpt::obs::CObservationBearingRange& _obj,
	mrpt_msgs::ObservationRangeBearing& _msg);

/** MRPT->ROS: Takes a CObservationBearingRange and outputs range data in
 * mrpt_msgs::ObservationRangeBearing + the relative pose of the range sensor
 * wrt base_link
 * \return true on sucessful conversion, false on any error.
 * \sa ros2mrpt
 */
bool toROS(
	const mrpt::obs::CObservationBearingRange& _obj,
	mrpt_msgs::ObservationRangeBearing& _msg, geometry_msgs::Pose& sensorPose);

/** @} */

}  // namespace mrpt_bridge
