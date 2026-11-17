/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#pragma once
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/poses/CPose3D.h>

#include <cstdint>
#include <geometry_msgs/msg/pose.hpp>
#include <mrpt_msgs/msg/observation_range_beacon.hpp>
#include <string>

namespace mrpt_msgs_bridge
{
/** @name ObservationRangeBeacon: ROS 2 <-> MRPT
 *  @{ */

/** ROS 2->MRPT: Takes a mrpt_msgs::msg::ObservationRangeBeacon and the
 * relative pose of the range sensor w.r.t. base_link, and builds a
 * CObservationBeaconRanges.
 * \return true on successful conversion, false on any error.
 * \sa toROS
 */
bool fromROS(
	const mrpt_msgs::msg::ObservationRangeBeacon& _msg, const mrpt::poses::CPose3D& _pose,
	mrpt::obs::CObservationBeaconRanges& _obj);

/** MRPT->ROS 2: Takes a CObservationBeaconRanges and outputs range data in
 * mrpt_msgs::msg::ObservationRangeBeacon.
 * \return true on successful conversion, false on any error.
 * \sa fromROS
 */
bool toROS(
	const mrpt::obs::CObservationBeaconRanges& _obj, mrpt_msgs::msg::ObservationRangeBeacon& _msg);

/** MRPT->ROS 2: Takes a CObservationBeaconRanges and outputs range data in
 * mrpt_msgs::msg::ObservationRangeBeacon, plus the relative pose of the range
 * sensor w.r.t. base_link.
 * \return true on successful conversion, false on any error.
 * \sa fromROS
 */
bool toROS(
	const mrpt::obs::CObservationBeaconRanges& _obj, mrpt_msgs::msg::ObservationRangeBeacon& _msg,
	geometry_msgs::msg::Pose& _pose);

/** @} */

}  // namespace mrpt_msgs_bridge