/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

/*
 * File: landmark.h
 * Author: Vladislav Tananaev
 *
 */

#pragma once

#include <mrpt/obs/CObservationBearingRange.h>

#include <cstdint>
#include <geometry_msgs/msg/pose.hpp>
#include <mrpt_msgs/msg/observation_range_bearing.hpp>
#include <string>

namespace mrpt_msgs_bridge
{
/** @name LaserScan: ROS2 <-> MRPT
 *  @{ */

/** ROS2->MRPT: Takes a mrpt_msgs::CObservationBearingRange and the relative
 * pose of the sensor wrt base_link and builds a ObservationRangeBearing \return
 * true on sucessful conversion, false on any error. \sa mrpt2ros
 */
bool fromROS(
	const mrpt_msgs::msg::ObservationRangeBearing& _msg,
	const mrpt::poses::CPose3D& _pose,
	mrpt::obs::CObservationBearingRange& _obj);

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
	mrpt_msgs::msg::ObservationRangeBearing& _msg,
	geometry_msgs::msg::Pose& sensorPose);

/** @} */

}  // namespace mrpt_msgs_bridge