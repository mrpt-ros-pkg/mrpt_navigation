/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <mrpt/graphs/TNodeID.h>
#include <mrpt/ros1bridge/pose.h>
#include <mrpt_msgs/NodeIDWithPose.h>
#include <mrpt_msgs_bridge/network_of_poses.h>

#include <iostream>	 // for debugging reasons

using mrpt::graphs::TNodeID;

///////////////////////////////////////////////////////////////////////////////////////////
// MRPT => ROS
///////////////////////////////////////////////////////////////////////////////////////////

void mrpt_msgs_bridge::toROS(
	const mrpt::graphs::CNetworkOfPoses2DInf& mrpt_graph,
	mrpt_msgs::NetworkOfPoses& ros_graph)
{
	MRPT_START
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
		ros_node.node_id = poses_cit->first;
		// pose
		ros_node.pose = mrpt::ros1bridge::toROS_Pose(poses_cit->second);

		// zero the optional fields
		ros_node.str_id.data = "";
		ros_node.node_id_loc = 0;

		ros_graph.nodes.vec.push_back(ros_node);
	}

	// fill the constraints
	for (const auto& edge : constraints)
	{
		mrpt_msgs::GraphConstraint ros_constr;

		// constraint ends
		ros_constr.node_id_from = edge.first.first;
		ros_constr.node_id_to = edge.first.second;

		// constraint mean and covariance
		if (mrpt_graph.edges_store_inverse_poses)
		{
			CPosePDFGaussianInf constr_inv;
			edge.second.inverse(constr_inv);
			ros_constr.constraint = mrpt::ros1bridge::toROS(constr_inv);
		}
		else
		{
			ros_constr.constraint = mrpt::ros1bridge::toROS(edge.second);
		}

		ros_graph.constraints.push_back(ros_constr);
	}

	MRPT_END
}

void mrpt_msgs_bridge::toROS(
	[[maybe_unused]] const mrpt::graphs::CNetworkOfPoses3DInf& mrpt_graph,
	[[maybe_unused]] mrpt_msgs::NetworkOfPoses& ros_graph)
{
	THROW_EXCEPTION("Conversion not implemented yet");
	// TODO: Implement CNetworkOfPoses3DInf => mrpt_msgs::NetworkOfPoses
	// conversion
}

void mrpt_msgs_bridge::toROS(
	const mrpt::graphs::CNetworkOfPoses2DInf_NA& mrpt_graph,
	mrpt_msgs::NetworkOfPoses& ros_graph)
{
	MRPT_START

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
		ros_node.node_id = poses_cit->first;
		// pose
		ros_node.pose = mrpt::ros1bridge::toROS_Pose(poses_cit->second);

		// optional fields for the MR-SLAM case
		ros_node.str_id.data = poses_cit->second.agent_ID_str;
		ros_node.node_id_loc = poses_cit->second.nodeID_loc;

		ros_graph.nodes.vec.push_back(ros_node);
	}

	// fill the constraints -- same as in the conversion from
	// CNetworkOfPoses2DInf
	for (const auto& edge : constraints)
	{
		mrpt_msgs::GraphConstraint ros_constr;

		// constraint ends
		ros_constr.node_id_from = edge.first.first;
		ros_constr.node_id_to = edge.first.second;

		// constraint mean and covariance
		if (mrpt_graph.edges_store_inverse_poses)
		{
			CPosePDFGaussianInf constr_inv;
			edge.second.inverse(constr_inv);
			ros_constr.constraint = mrpt::ros1bridge::toROS(constr_inv);
		}
		else
		{
			ros_constr.constraint = mrpt::ros1bridge::toROS(edge.second);
		}

		ros_graph.constraints.push_back(ros_constr);
	}

	MRPT_END
}

void mrpt_msgs_bridge::toROS(
	[[maybe_unused]] const mrpt::graphs::CNetworkOfPoses3DInf_NA& mrpt_graph,
	[[maybe_unused]] mrpt_msgs::NetworkOfPoses& ros_graph)
{
	THROW_EXCEPTION("Conversion not implemented yet");
}

///////////////////////////////////////////////////////////////////////////////////////////
// ROS => MRPT
///////////////////////////////////////////////////////////////////////////////////////////

void mrpt_msgs_bridge::fromROS(
	const mrpt_msgs::NetworkOfPoses& ros_graph,
	mrpt::graphs::CNetworkOfPoses2DInf& mrpt_graph)
{
	MRPT_START
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
		CPose2D mrpt_pose = CPose2D(mrpt::ros1bridge::fromROS(nodes_cit->pose));

		mrpt_graph.nodes.insert(
			make_pair(static_cast<TNodeID>(nodes_cit->node_id), mrpt_pose));
	}

	// fill the constraints
	for (constraints_cit_t constr_cit = ros_graph.constraints.begin();
		 constr_cit != ros_graph.constraints.end(); ++constr_cit)
	{
		// constraint ends
		auto constr_ends(make_pair(
			static_cast<TNodeID>(constr_cit->node_id_from),
			static_cast<TNodeID>(constr_cit->node_id_to)));

		// constraint value
		const auto mrpt_constr = mrpt::poses::CPosePDFGaussianInf(
			mrpt::ros1bridge::fromROS(constr_cit->constraint));

		mrpt_graph.edges.insert(make_pair(constr_ends, mrpt_constr));
	}

	mrpt_graph.edges_store_inverse_poses = false;

	MRPT_END
}

void mrpt_msgs_bridge::fromROS(
	[[maybe_unused]] const mrpt_msgs::NetworkOfPoses& ros_graph,
	[[maybe_unused]] mrpt::graphs::CNetworkOfPoses3DInf& mrpt_graph)
{
	THROW_EXCEPTION("Conversion not implemented yet");
	// TODO: Implement mrpt_msgs::NetworkOfPoses => CNetworkOfPoses3DInf
	// conversion.
}

void mrpt_msgs_bridge::fromROS(
	const mrpt_msgs::NetworkOfPoses& ros_graph,
	mrpt::graphs::CNetworkOfPoses2DInf_NA& mrpt_graph)
{
	MRPT_START
	using namespace mrpt::poses;
	using namespace mrpt_msgs;
	using namespace std;

	using nodes_cit_t = NetworkOfPoses::_nodes_type::_vec_type::const_iterator;
	using constraints_cit_t = NetworkOfPoses::_constraints_type::const_iterator;

	using mrpt_graph_pose_t =
		mrpt::graphs::CNetworkOfPoses2DInf_NA::global_pose_t;

	// fill root node
	mrpt_graph.root = ros_graph.root;

	// fill nodeIDs, positions
	for (nodes_cit_t nodes_cit = ros_graph.nodes.vec.begin();
		 nodes_cit != ros_graph.nodes.vec.end(); ++nodes_cit)
	{
		// set the nodeID/pose
		mrpt_graph_pose_t mrpt_node =
			mrpt::ros1bridge::fromROS(nodes_cit->pose);

		// set the MR-SLAM fields
		mrpt_node.agent_ID_str = nodes_cit->str_id.data;
		mrpt_node.nodeID_loc = nodes_cit->node_id_loc;

		mrpt_graph.nodes.insert(
			make_pair(static_cast<TNodeID>(nodes_cit->node_id), mrpt_node));
	}

	// fill the constraints
	for (constraints_cit_t constr_cit = ros_graph.constraints.begin();
		 constr_cit != ros_graph.constraints.end(); ++constr_cit)
	{
		// constraint ends
		auto constr_ends(make_pair(
			static_cast<TNodeID>(constr_cit->node_id_from),
			static_cast<TNodeID>(constr_cit->node_id_to)));

		// constraint value
		const auto mrpt_constr = mrpt::poses::CPosePDFGaussianInf(
			mrpt::ros1bridge::fromROS(constr_cit->constraint));

		mrpt_graph.edges.insert(make_pair(constr_ends, mrpt_constr));
	}

	mrpt_graph.edges_store_inverse_poses = false;

	MRPT_END
}

void convert(
	[[maybe_unused]] mrpt_msgs::NetworkOfPoses& ros_graph,
	[[maybe_unused]] const mrpt::graphs::CNetworkOfPoses3DInf_NA& mrpt_graph)
{
	THROW_EXCEPTION("Conversion not implemented yet");
}
