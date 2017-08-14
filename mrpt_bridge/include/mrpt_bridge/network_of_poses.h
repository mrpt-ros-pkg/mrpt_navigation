/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT) |
	 |                          http://www.mrpt.org/ |
	 | |
	 | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file |
	 | See: http://www.mrpt.org/Authors - All rights reserved. |
	 | Released under BSD License. See details in http://www.mrpt.org/License |
	 +---------------------------------------------------------------------------+
   */

/**\brief File includes methods for converting CNetworkOfPoses*DInf <=>
 * NetworkOfPoses message types
 */

#ifndef NETWORK_OF_POSES_H
#define NETWORK_OF_POSES_H

#include <mrpt_msgs/NetworkOfPoses.h>
#include <mrpt/graphs/CNetworkOfPoses.h>

namespace mrpt_bridge
{
/**\name ROS \rightarrow MRPT conversions */
/**\brief Convert [MRPT] CNetworkOfPoses*D \rightarrow [ROS] NetworkOfPoses.
 * \param[in] mrpt_graph MRPT graph representation
 * \param[out] ros_graph ROS graph representation
 */
/**\{*/

// TODO - convert these methods into a common polymorphic method
void convert(
	const mrpt::graphs::CNetworkOfPoses2D& mrpt_graph,
	mrpt_msgs::NetworkOfPoses& ros_graph);

void convert(
	const mrpt::graphs::CNetworkOfPoses3D& mrpt_graph,
	mrpt_msgs::NetworkOfPoses& ros_graph);

void convert(
	const mrpt::graphs::CNetworkOfPoses2DInf& mrpt_graph,
	mrpt_msgs::NetworkOfPoses& ros_graph);

void convert(
	const mrpt::graphs::CNetworkOfPoses3DInf& mrpt_graph,
	mrpt_msgs::NetworkOfPoses& ros_graph);

void convert(
	const mrpt::graphs::CNetworkOfPoses2DInf_NA& mrpt_graph,
	mrpt_msgs::NetworkOfPoses& ros_graph);

void convert(
	const mrpt::graphs::CNetworkOfPoses3DInf_NA& mrpt_graph,
	mrpt_msgs::NetworkOfPoses& ros_graph);

/**\} */

/////////////////////////////////////////////////////////////////////////

/**\name ROS \rightarrow MRPT conversions */
/**\brief Convert [ROS] NetworkOfPoses \rightarrow [MRPT] CNetworkOfPoses*DInf.
 * \param[in] mrpt_graph ROS graph representation
 * \param[out] ros_graph MRPT graph representation
 */
/**\{ */
void convert(
	const mrpt::graphs::CNetworkOfPoses2D& mrpt_graph,
	mrpt_msgs::NetworkOfPoses& ros_graph);

void convert(
	const mrpt::graphs::CNetworkOfPoses3D& mrpt_graph,
	mrpt_msgs::NetworkOfPoses& ros_graph);

void convert(
	const mrpt_msgs::NetworkOfPoses& ros_graph,
	mrpt::graphs::CNetworkOfPoses2DInf& mrpt_graph);

void convert(
	const mrpt_msgs::NetworkOfPoses& ros_graph,
	mrpt::graphs::CNetworkOfPoses3DInf& mrpt_graph);

void convert(
	const mrpt_msgs::NetworkOfPoses& ros_graph,
	mrpt::graphs::CNetworkOfPoses2DInf_NA& mrpt_graph);

void convert(
	const mrpt_msgs::NetworkOfPoses& ros_graph,
	mrpt::graphs::CNetworkOfPoses3DInf_NA& mrpt_graph);

/**\} */

}  // end of namespace - mrpt_bridge

#endif /* end of include guard: NETWORK_OF_POSES_H */
