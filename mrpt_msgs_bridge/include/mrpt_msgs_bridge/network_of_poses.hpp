/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

/**\brief File includes methods for converting CNetworkOfPoses*DInf <=>
 * NetworkOfPoses message types
 */

#pragma once

#include <mrpt/graphs/CNetworkOfPoses.h>

#include <mrpt_msgs/msg/network_of_poses.hpp>

namespace mrpt_msgs_bridge
{
/**\name MRPT \rightarrow ROS2 conversions */
/**\brief Convert [MRPT] CNetworkOfPoses*D \rightarrow [ROS2] NetworkOfPoses.
 * \param[in] mrpt_graph MRPT graph representation
 * \param[out] ros_graph ROS graph representation
 */
/**\{*/

// TODO - convert these methods into a common polymorphic method
void toROS(
	const mrpt::graphs::CNetworkOfPoses2D& mrpt_graph, mrpt_msgs::msg::NetworkOfPoses& ros_graph);

void toROS(
	const mrpt::graphs::CNetworkOfPoses3D& mrpt_graph, mrpt_msgs::msg::NetworkOfPoses& ros_graph);

void toROS(
	const mrpt::graphs::CNetworkOfPoses2DInf& mrpt_graph,
	mrpt_msgs::msg::NetworkOfPoses& ros_graph);

void toROS(
	const mrpt::graphs::CNetworkOfPoses3DInf& mrpt_graph,
	mrpt_msgs::msg::NetworkOfPoses& ros_graph);

void toROS(
	const mrpt::graphs::CNetworkOfPoses2DInf_NA& mrpt_graph,
	mrpt_msgs::msg::NetworkOfPoses& ros_graph);

void toROS(
	const mrpt::graphs::CNetworkOfPoses3DInf_NA& mrpt_graph,
	mrpt_msgs::msg::NetworkOfPoses& ros_graph);

/**\} */

/////////////////////////////////////////////////////////////////////////

/**\name ROS2 \rightarrow MRPT conversions */
/**\brief Convert [ROS2] NetworkOfPoses \rightarrow [MRPT] CNetworkOfPoses*DInf.
 * \param[in] mrpt_graph ROS2 graph representation
 * \param[out] ros_graph MRPT graph representation
 */
/**\{ */
void fromROS(
	const mrpt::graphs::CNetworkOfPoses2D& mrpt_graph, mrpt_msgs::msg::NetworkOfPoses& ros_graph);

void fromROS(
	const mrpt::graphs::CNetworkOfPoses3D& mrpt_graph, mrpt_msgs::msg::NetworkOfPoses& ros_graph);

void fromROS(
	const mrpt_msgs::msg::NetworkOfPoses& ros_graph,
	mrpt::graphs::CNetworkOfPoses2DInf& mrpt_graph);

void fromROS(
	const mrpt_msgs::msg::NetworkOfPoses& ros_graph,
	mrpt::graphs::CNetworkOfPoses3DInf& mrpt_graph);

void fromROS(
	const mrpt_msgs::msg::NetworkOfPoses& ros_graph,
	mrpt::graphs::CNetworkOfPoses2DInf_NA& mrpt_graph);

void fromROS(
	const mrpt_msgs::msg::NetworkOfPoses& ros_graph,
	mrpt::graphs::CNetworkOfPoses3DInf_NA& mrpt_graph);

/**\} */

}  // namespace mrpt_msgs_bridge