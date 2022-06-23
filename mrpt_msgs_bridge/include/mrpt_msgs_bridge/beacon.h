/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <geometry_msgs/Pose.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt_msgs/ObservationRangeBeacon.h>

#include <cstdint>
#include <string>

namespace mrpt_msgs_bridge
{
/** @name ObservationRangeBeacon: ROS <-> MRPT
 *  @{ */

/** ROS->MRPT: Takes a mrpt_msgs::ObservationRangeBeacon and the relative pose
 * of the range sensor wrt base_link and builds a CObservationBeaconRanges
 * \return true on sucessful conversion, false on any error.
 * \sa mrpt2ros
 */
bool fromROS(
	const mrpt_msgs::ObservationRangeBeacon& _msg,
	const mrpt::poses::CPose3D& _pose,
	mrpt::obs::CObservationBeaconRanges& _obj);

/** MRPT->ROS: Takes a CObservationBeaconRanges and outputs range data in
 * mrpt_msgs::ObservationRangeBeacon \return true on sucessful conversion, false
 * on any error. \sa ros2mrpt
 */
bool toROS(
	const mrpt::obs::CObservationBeaconRanges& _obj,
	mrpt_msgs::ObservationRangeBeacon& _msg);

/** MRPT->ROS: Takes a CObservationBeaconRanges and outputs range data in
 * mrpt_msgs::ObservationRangeBeacon + the relative pose of the range sensor wrt
 * base_link \return true on sucessful conversion, false on any error. \sa
 * ros2mrpt
 */
bool toROS(
	const mrpt::obs::CObservationBeaconRanges& _obj,
	mrpt_msgs::ObservationRangeBeacon& _msg, geometry_msgs::Pose& _pose);

/** @} */

}  // namespace mrpt_msgs_bridge