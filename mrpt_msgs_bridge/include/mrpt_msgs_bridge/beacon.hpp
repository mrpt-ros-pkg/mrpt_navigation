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
/** @name ObservationRangeBeacon: ROS2 <-> MRPT
 *  @{ */

/** ROS->MRPT: Takes a mrpt_msgs::ObservationRangeBeacon and the relative pose
 * of the range sensor wrt base_link and builds a CObservationBeaconRanges
 * \return true on sucessful conversion, false on any error.
 * \sa mrpt2ros
 */
bool fromROS(
	const mrpt_msgs::msg::ObservationRangeBeacon _msg, const mrpt::poses::CPose3D& _pose,
	mrpt::obs::CObservationBeaconRanges& _obj);

/** MRPT->ROS2: Takes a CObservationBeaconRanges and outputs range data in
 * mrpt_msgs::ObservationRangeBeacon \return true on sucessful conversion, false
 * on any error. \sa ros2mrpt
 */
bool toROS(
	const mrpt::obs::CObservationBeaconRanges& _obj, mrpt_msgs::msg::ObservationRangeBeacon& _msg);

/** MRPT->ROS2: Takes a CObservationBeaconRanges and outputs range data in
 * mrpt_msgs::ObservationRangeBeacon + the relative pose of the range sensor wrt
 * base_link \return true on sucessful conversion, false on any error. \sa
 * ros2mrpt
 */
bool toROS(
	const mrpt::obs::CObservationBeaconRanges& _obj, mrpt_msgs::msg::ObservationRangeBeacon& _msg,
	geometry_msgs::msg::Pose& _pose);

/** @} */

}  // namespace mrpt_msgs_bridge