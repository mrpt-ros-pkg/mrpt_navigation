/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

/*
 * File: landmark.hpp
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
/** @name ObservationRangeBearing: ROS 2 <-> MRPT
 *  @{ */

/** ROS 2->MRPT: Takes a mrpt_msgs::msg::ObservationRangeBearing and the
 * relative pose of the sensor w.r.t. base_link, and builds a
 * CObservationBearingRange.
 * \return true on successful conversion, false on any error.
 * \sa toROS
 */
bool fromROS(
	const mrpt_msgs::msg::ObservationRangeBearing& _msg, const mrpt::poses::CPose3D& _pose,
	mrpt::obs::CObservationBearingRange& _obj);

/** MRPT->ROS 2: Takes a CObservationBearingRange and outputs range data in
 * mrpt_msgs::msg::ObservationRangeBearing.
 * \return true on successful conversion, false on any error.
 * \sa fromROS
 */
bool toROS(
	const mrpt::obs::CObservationBearingRange& _obj, mrpt_msgs::msg::ObservationRangeBearing& _msg);

/** MRPT->ROS 2: Takes a CObservationBearingRange and outputs range data in
 * mrpt_msgs::msg::ObservationRangeBearing, plus the relative pose of the
 * sensor w.r.t. base_link.
 * \return true on successful conversion, false on any error.
 * \sa fromROS
 */
bool toROS(
	const mrpt::obs::CObservationBearingRange& _obj, mrpt_msgs::msg::ObservationRangeBearing& _msg,
	geometry_msgs::msg::Pose& sensorPose);

/** @} */

}  // namespace mrpt_msgs_bridge