/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
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
#include <geometry_msgs/msg/pose.hpp>
#include <mrpt_msgs/msg/observation_range_bearing.hpp>
#include <mrpt/obs/CObservationBearingRange.h>

namespace mrpt_msgs_bridge
{
/** @name LaserScan: ROS2 <-> MRPT
 *  @{ */

/** ROS2->MRPT: Takes a mrpt_msgs::CObservationBearingRange and the relative pose
 * of the sensor wrt base_link and builds a ObservationRangeBearing
 * \return true on sucessful conversion, false on any error.
 * \sa mrpt2ros
 */
bool fromROS(
	const mrpt_msgs::msg::ObservationRangeBearing& _msg,
	const mrpt::poses::CPose3D& _pose, mrpt::obs::CObservationBearingRange& _obj
);

/** MRPT->ROS2: Takes a CObservationBearingRange and outputs range data in
 * mrpt_msgs::ObservationRangeBearing
 * \return true on sucessful conversion, false on any error.
 * \sa ros2mrpt
 */
bool toROS(
	const mrpt::obs::CObservationBearingRange& _obj,
	mrpt_msgs::msg::ObservationRangeBearing& _msg);

/** MRPT->ROS2: Takes a CObservationBearingRange and outputs range data in
 * mrpt_msgs::ObservationRangeBearing + the relative pose of the range sensor
 * wrt base_link
 * \return true on sucessful conversion, false on any error.
 * \sa ros2mrpt
 */
bool toROS(
	const mrpt::obs::CObservationBearingRange& _obj,
	mrpt_msgs::msg::ObservationRangeBearing& _msg, geometry_msgs::msg::Pose& sensorPose);

/** @} */

}  // namespace mrpt_bridge