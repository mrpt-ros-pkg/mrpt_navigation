/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT) |
	 |                          http://www.mrpt.org/ |
	 | |
	 | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file |
	 | See: http://www.mrpt.org/Authors - All rights reserved. |
	 | Released under BSD License. See details in http://www.mrpt.org/License |
	 +---------------------------------------------------------------------------+
   */

#include "mrpt_bridge/network_of_poses.h"
#include "mrpt_bridge/pose.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <mrpt_msgs/NodeIDWithPose.h>

#include <iostream>  // for debugging reasons

#include <mrpt/version.h>
#if MRPT_VERSION<0x199
using namespace mrpt::utils;
#else
#include <mrpt/graphs/TNodeID.h>
using mrpt::graphs::TNodeID;
#endif


namespace mrpt_bridge
{
///////////////////////////////////////////////////////////////////////////////////////////
// MRPT => ROS
///////////////////////////////////////////////////////////////////////////////////////////

void convert(
	const mrpt::graphs::CNetworkOfPoses2DInf& mrpt_graph,
	mrpt_msgs::NetworkOfPoses& ros_graph)
{
	MRPT_START;
	using namespace geometry_msgs;
	using namespace mrpt::math;
	using namespace mrpt::graphs;
	using namespace mrpt::poses;
	using namespace std;

	typedef typename mrpt::graphs::CNetworkOfPoses2DInf::global_poses_t::
		const_iterator poses_cit_t;

	const CNetworkOfPoses2DInf::BASE::edges_map_t& constraints =
		mrpt_graph.BASE::edges;

	// fill root node
	ros_graph.root = mrpt_graph.root;

	// fill nodeIDs, positions
	for (poses_cit_t poses_cit = mrpt_graph.nodes.begin();
		 poses_cit != mrpt_graph.nodes.end(); ++poses_cit)
	{
		mrpt_msgs::NodeIDWithPose ros_node;

		// nodeID
		ros_node.nodeID = poses_cit->first;
		// pose
		convert(poses_cit->second, ros_node.pose);

		// zero the optional fields
		ros_node.str_ID.data = "";
		ros_node.nodeID_loc = 0;

		ros_graph.nodes.vec.push_back(ros_node);
	}

	// fill the constraints
	for (CNetworkOfPoses2DInf::const_iterator constr_it = constraints.begin();
		 constr_it != constraints.end(); ++constr_it)
	{
		mrpt_msgs::GraphConstraint ros_constr;

		// constraint ends
		ros_constr.nodeID_from = constr_it->first.first;
		ros_constr.nodeID_to = constr_it->first.second;

		// constraint mean and covariance
		if (mrpt_graph.edges_store_inverse_poses)
		{
			CPosePDFGaussianInf constr_inv;
			constr_it->second.inverse(constr_inv);
			convert(constr_inv, ros_constr.constraint);
		}
		else
		{
			convert(constr_it->second, ros_constr.constraint);
		}

		ros_graph.constraints.push_back(ros_constr);
	}

	MRPT_END;
}

void convert(
	const mrpt::graphs::CNetworkOfPoses3DInf& mrpt_graph,
	mrpt_msgs::NetworkOfPoses& ros_graph)
{
	THROW_EXCEPTION("Conversion not implemented yet");
	MRPT_TODO(
		"Implement CNetworkOfPoses3DInf => mrpt_msgs::NetworkOfPoses "
		"conversion.");
}

void convert(
	const mrpt::graphs::CNetworkOfPoses2DInf_NA& mrpt_graph,
	mrpt_msgs::NetworkOfPoses& ros_graph)
{
	MRPT_START;

	using namespace geometry_msgs;
	using namespace mrpt::math;
	using namespace mrpt::graphs;
	using namespace mrpt::poses;
	using namespace std;

	typedef typename mrpt::graphs::CNetworkOfPoses2DInf_NA::global_poses_t::
		const_iterator poses_cit_t;

	//// debugging print.
	// for (poses_cit_t it = mrpt_graph.nodes.begin();
	// it != mrpt_graph.nodes.end();
	//++it) {
	// cout << it->first << " | " << it->second << endl;
	//}

	const CNetworkOfPoses2DInf_NA::BASE::edges_map_t& constraints =
		mrpt_graph.BASE::edges;

	// fill root node
	ros_graph.root = mrpt_graph.root;

	// fill nodeIDs, positions
	for (poses_cit_t poses_cit = mrpt_graph.nodes.begin();
		 poses_cit != mrpt_graph.nodes.end(); ++poses_cit)
	{
		mrpt_msgs::NodeIDWithPose ros_node;

		// nodeID
		ros_node.nodeID = poses_cit->first;
		// pose
		convert(poses_cit->second, ros_node.pose);

		// optional fields for the MR-SLAM case
		ros_node.str_ID.data = poses_cit->second.agent_ID_str;
		ros_node.nodeID_loc = poses_cit->second.nodeID_loc;

		ros_graph.nodes.vec.push_back(ros_node);
	}

	// fill the constraints -- same as in the conversion from
	// CNetworkOfPoses2DInf
	for (CNetworkOfPoses2DInf::const_iterator constr_it = constraints.begin();
		 constr_it != constraints.end(); ++constr_it)
	{
		mrpt_msgs::GraphConstraint ros_constr;

		// constraint ends
		ros_constr.nodeID_from = constr_it->first.first;
		ros_constr.nodeID_to = constr_it->first.second;

		// constraint mean and covariance
		if (mrpt_graph.edges_store_inverse_poses)
		{
			CPosePDFGaussianInf constr_inv;
			constr_it->second.inverse(constr_inv);
			convert(constr_inv, ros_constr.constraint);
		}
		else
		{
			convert(constr_it->second, ros_constr.constraint);
		}

		ros_graph.constraints.push_back(ros_constr);
	}

	MRPT_END;
}

void convert(
	const mrpt::graphs::CNetworkOfPoses3DInf_NA& mrpt_graph,
	mrpt_msgs::NetworkOfPoses& ros_graph)
{
	THROW_EXCEPTION("Conversion not implemented yet");
}

///////////////////////////////////////////////////////////////////////////////////////////
// ROS => MRPT
///////////////////////////////////////////////////////////////////////////////////////////

void convert(
	const mrpt_msgs::NetworkOfPoses& ros_graph,
	mrpt::graphs::CNetworkOfPoses2DInf& mrpt_graph)
{
	MRPT_START;
	using namespace mrpt::poses;
	using namespace mrpt_msgs;
	using namespace std;

	typedef NetworkOfPoses::_nodes_type::_vec_type::const_iterator nodes_cit_t;
	typedef NetworkOfPoses::_constraints_type::const_iterator constraints_cit_t;

	// fill root node
	mrpt_graph.root = ros_graph.root;

	// fill nodeIDs, positions
	for (nodes_cit_t nodes_cit = ros_graph.nodes.vec.begin();
		 nodes_cit != ros_graph.nodes.vec.end(); ++nodes_cit)
	{
		// get the pose
		CPose2D mrpt_pose;
		convert(nodes_cit->pose, mrpt_pose);

		mrpt_graph.nodes.insert(
			make_pair(static_cast<TNodeID>(nodes_cit->nodeID), mrpt_pose));
	}

	// fill the constraints
	for (constraints_cit_t constr_cit = ros_graph.constraints.begin();
		 constr_cit != ros_graph.constraints.end(); ++constr_cit)
	{
		// constraint ends
		auto constr_ends(
			make_pair(
				static_cast<TNodeID>(constr_cit->nodeID_from),
				static_cast<TNodeID>(constr_cit->nodeID_to)));

		// constraint value
		mrpt::poses::CPosePDFGaussianInf mrpt_constr;
		convert(constr_cit->constraint, mrpt_constr);

		mrpt_graph.edges.insert(make_pair(constr_ends, mrpt_constr));
	}

	mrpt_graph.edges_store_inverse_poses = false;

	MRPT_END;
}

void convert(
	const mrpt_msgs::NetworkOfPoses& ros_graph,
	mrpt::graphs::CNetworkOfPoses3DInf& mrpt_graph)
{
	THROW_EXCEPTION("Conversion not implemented yet");
	MRPT_TODO(
		"Implement mrpt_msgs::NetworkOfPoses => CNetworkOfPoses3DInf "
		"conversion.");
}

void convert(
	const mrpt_msgs::NetworkOfPoses& ros_graph,
	mrpt::graphs::CNetworkOfPoses2DInf_NA& mrpt_graph)
{
	MRPT_START;
	using namespace mrpt::poses;
	using namespace mrpt_msgs;
	using namespace std;

	typedef NetworkOfPoses::_nodes_type::_vec_type::const_iterator nodes_cit_t;
	typedef NetworkOfPoses::_constraints_type::const_iterator constraints_cit_t;

	typedef mrpt::graphs::CNetworkOfPoses2DInf_NA::global_pose_t
		mrpt_graph_pose_t;

	// fill root node
	mrpt_graph.root = ros_graph.root;

	// fill nodeIDs, positions
	for (nodes_cit_t nodes_cit = ros_graph.nodes.vec.begin();
		 nodes_cit != ros_graph.nodes.vec.end(); ++nodes_cit)
	{
		mrpt_graph_pose_t mrpt_node;

		// set the nodeID/pose
		convert(nodes_cit->pose, mrpt_node);

		// set the MR-SLAM fields
		mrpt_node.agent_ID_str = nodes_cit->str_ID.data;
		mrpt_node.nodeID_loc = nodes_cit->nodeID_loc;

		mrpt_graph.nodes.insert(
			make_pair(static_cast<TNodeID>(nodes_cit->nodeID), mrpt_node));
	}

	// fill the constraints
	for (constraints_cit_t constr_cit = ros_graph.constraints.begin();
		 constr_cit != ros_graph.constraints.end(); ++constr_cit)
	{
		// constraint ends
		auto constr_ends(
			make_pair(
				static_cast<TNodeID>(constr_cit->nodeID_from),
				static_cast<TNodeID>(constr_cit->nodeID_to)));

		// constraint value
		mrpt::poses::CPosePDFGaussianInf mrpt_constr;
		convert(constr_cit->constraint, mrpt_constr);

		mrpt_graph.edges.insert(make_pair(constr_ends, mrpt_constr));
	}

	mrpt_graph.edges_store_inverse_poses = false;

	MRPT_END;
}

void convert(
	mrpt_msgs::NetworkOfPoses& ros_graph,
	const mrpt::graphs::CNetworkOfPoses3DInf_NA& mrpt_graph)
{
	THROW_EXCEPTION("Conversion not implemented yet");
}

}  // end of namespace
