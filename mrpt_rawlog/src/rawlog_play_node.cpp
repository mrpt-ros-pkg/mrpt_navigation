/***********************************************************************************
 * Revised BSD License *
 * Copyright (c) 2014, Markus Bader <markus.bader@tuwien.ac.at> *
 * All rights reserved. *
 *                                                                                 *
 * Redistribution and use in source and binary forms, with or without *
 * modification, are permitted provided that the following conditions are met: *
 *     * Redistributions of source code must retain the above copyright *
 *       notice, this list of conditions and the following disclaimer. *
 *     * Redistributions in binary form must reproduce the above copyright *
 *       notice, this list of conditions and the following disclaimer in the *
 *       documentation and/or other materials provided with the distribution. *
 *     * Neither the name of the Vienna University of Technology nor the *
 *       names of its contributors may be used to endorse or promote products *
 *       derived from this software without specific prior written permission. *
 *                                                                                 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND *
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 **
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE *
 * DISCLAIMED. IN NO EVENT SHALL Markus Bader BE LIABLE FOR ANY *
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES *
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 **
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND *
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 **
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **                       *
 ***********************************************************************************/

#include "rawlog_play_node.h"

#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/ros1bridge/laser_scan.h>
#include <mrpt/ros1bridge/pose.h>
#include <mrpt/ros1bridge/time.h>
#include <mrpt/system/filesystem.h>
#include <mrpt_msgs_bridge/beacon.h>
#include <mrpt_msgs_bridge/landmark.h>

#include <boost/interprocess/sync/scoped_lock.hpp>
using namespace mrpt::obs;

#include <mrpt/serialization/CArchive.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rawlog_play");
	ros::NodeHandle n;
	RawlogPlayNode my_node(n);
	my_node.init();
	my_node.loop();
	return 0;
}

RawlogPlayNode::~RawlogPlayNode() {}
RawlogPlayNode::RawlogPlayNode(ros::NodeHandle& n)
	: RawlogPlay(new RawlogPlayNode::ParametersNode()), n_(n), loop_count_(0)
{
}

RawlogPlayNode::ParametersNode* RawlogPlayNode::param()
{
	return static_cast<RawlogPlayNode::ParametersNode*>(param_);
}

void RawlogPlayNode::init()
{
	if (!mrpt::system::fileExists(param_->rawlog_file))
	{
		ROS_ERROR("raw_file: %s does not exit", param_->rawlog_file.c_str());
		ros::shutdown();
		return;
	}
	rawlog_stream_.open(param_->rawlog_file);
	pub_laser_ = n_.advertise<sensor_msgs::LaserScan>("scan", 10);
	pub_beacon_ = n_.advertise<mrpt_msgs::ObservationRangeBeacon>("beacon", 10);
	pub_landmark_ =
		n_.advertise<mrpt_msgs::ObservationRangeBearing>("landmark", 10);
	odom_frame_ = param()->odom_frame;
	base_frame_ = param()->base_frame;
	robotPose = mrpt::poses::CPose3DPDFGaussian();
}

void RawlogPlayNode::publishSingleObservation(
	const mrpt::obs::CObservation::Ptr& o)
{
	mrpt::poses::CPose3D pose_sensor;
	o->getSensorPose(pose_sensor);

	geometry_msgs::Pose msgSensorPose;	// not actually used after all.

	// Aux lambda to publish TF transforms "base_link" ==> "sensor_frame"
	// ------
	auto lambdaSendTfSensorPose = [&](const std_msgs::Header& header) {
		const tf2::Transform tfSensorPose =
			mrpt::ros1bridge::toROS_tfTransform(pose_sensor);

		geometry_msgs::TransformStamped tfGeom =
			tf2::toMsg(tf2::Stamped<tf2::Transform>(
				tfSensorPose, header.stamp + ros::Duration(0.05), base_frame_));
		tfGeom.child_frame_id = header.frame_id;

		tf_broadcaster_.sendTransform(tfGeom);
	};
	// ------

	if (auto laser = std::dynamic_pointer_cast<CObservation2DRangeScan>(o);
		laser)
	{  // laser observation detected
		mrpt::ros1bridge::toROS(*laser, msg_laser_, msgSensorPose);

		if (msg_laser_.header.frame_id.empty())
			msg_laser_.header.frame_id = "laser_link";
		msg_laser_.header.stamp = ros::Time::now();

		lambdaSendTfSensorPose(msg_laser_.header);

		pub_laser_.publish(msg_laser_);
	}
	else if (auto beacon =
				 std::dynamic_pointer_cast<CObservationBeaconRanges>(o);
			 beacon)
	{
		mrpt_msgs_bridge::toROS(*beacon, msg_beacon_, msgSensorPose);
		if (msg_beacon_.header.frame_id.empty())
			msg_beacon_.header.frame_id = "beacon_link";
		msg_beacon_.header.stamp = ros::Time::now();

		lambdaSendTfSensorPose(msg_beacon_.header);

		pub_beacon_.publish(msg_beacon_);
	}
	else if (auto landmark =
				 std::dynamic_pointer_cast<CObservationBearingRange>(o);
			 landmark)
	{
		mrpt_msgs_bridge::toROS(*landmark, msg_landmark_, msgSensorPose);
		if (msg_landmark_.header.frame_id.empty())
			msg_landmark_.header.frame_id = "landmark_link";
		msg_landmark_.header.stamp = ros::Time::now();

		lambdaSendTfSensorPose(msg_landmark_.header);
		pub_landmark_.publish(msg_landmark_);
	}
	else
	{
		ROS_WARN(
			"Observation mapping to ROS not implemented: %s",
			o->GetRuntimeClass()->className);
	}
}  // end publishSingleObservation()

bool RawlogPlayNode::nextEntry()
{
	CActionCollection::Ptr action;
	CSensoryFrame::Ptr observations;
	CObservation::Ptr obs;

	auto rs = mrpt::serialization::archiveFrom(rawlog_stream_);

	if (!CRawlog::getActionObservationPairOrObservation(
			rs, action, observations, obs, entry_))
	{
		ROS_INFO("end of stream!");
		return true;
	}
	tf2::Transform transform;

	// Process single obs, if present:
	if (obs) publishSingleObservation(obs);
	// and process all obs into a CSensoryFrame, if present:
	for (const auto& o : *observations) publishSingleObservation(o);

	mrpt::poses::CPose3DPDFGaussian out_pose_increment;
	action->getFirstMovementEstimation(out_pose_increment);
	robotPose -= out_pose_increment;

	msg_odom_.header.frame_id = odom_frame_;
	msg_odom_.child_frame_id = base_frame_;
	if (!msg_laser_.header.frame_id.empty())
	{
		msg_odom_.header.stamp = msg_laser_.header.stamp;
		msg_odom_.header.seq = msg_laser_.header.seq;
	}
	else if (!msg_beacon_.header.frame_id.empty())
	{
		msg_odom_.header.stamp = msg_beacon_.header.stamp;
		msg_odom_.header.seq = msg_beacon_.header.seq;
	}
	else if (!msg_landmark_.header.frame_id.empty())
	{
		msg_odom_.header.stamp = msg_landmark_.header.stamp;
		msg_odom_.header.seq = msg_landmark_.header.seq;
	}

	msg_odom_.pose = mrpt::ros1bridge::toROS_Pose(robotPose);
	transform = mrpt::ros1bridge::toROS_tfTransform(robotPose.mean);

	msg_odom_.header.stamp = ros::Time::now();

	MRPT_TODO("Publish /odom topic too?");

	{
		geometry_msgs::TransformStamped tfGeom =
			tf2::toMsg(tf2::Stamped<tf2::Transform>(
				transform.inverse(),
				msg_odom_.header.stamp + ros::Duration(0.05), base_frame_));
		tfGeom.child_frame_id = msg_odom_.header.frame_id;

		tf_broadcaster_.sendTransform(tfGeom);
	}

	return false;
}

void RawlogPlayNode::loop()
{
	bool end = false;
	for (ros::Rate rate(param()->rate); ros::ok() && !end; loop_count_++)
	{
		param()->update(loop_count_);
		end = nextEntry();
		ros::spinOnce();
		rate.sleep();
	}
}
