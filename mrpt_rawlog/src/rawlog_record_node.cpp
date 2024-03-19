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

#include "rawlog_record_node.h"

#include <mrpt/ros1bridge/laser_scan.h>
#include <mrpt/ros1bridge/pose.h>
#include <mrpt/ros1bridge/time.h>
#include <mrpt_msgs_bridge/marker_msgs.h>

#include <boost/interprocess/sync/scoped_lock.hpp>

using namespace mrpt::maps;
using namespace mrpt::obs;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rawlog_record");
	ros::NodeHandle n;
	RawlogRecordNode my_node(n);
	my_node.init();
	my_node.loop();
	return 0;
}

RawlogRecordNode::~RawlogRecordNode() {}
RawlogRecordNode::RawlogRecordNode(ros::NodeHandle& n) : n_(n) {}

void RawlogRecordNode::init()
{
	updateRawLogName(mrpt::Clock::now());
	ROS_INFO("rawlog file: %s", base_param_.raw_log_name.c_str());
	if (base_param_.record_range_scan)
	{
		subLaser_ =
			n_.subscribe("scan", 1, &RawlogRecordNode::callbackLaser, this);
	}
	if (base_param_.record_bearing_range)
	{
		subMarker_ =
			n_.subscribe("marker", 1, &RawlogRecordNode::callbackMarker, this);
	}
	subOdometry_ =
		n_.subscribe("odom", 1, &RawlogRecordNode::callbackOdometry, this);
}

void RawlogRecordNode::loop() { ros::spin(); }

bool RawlogRecordNode::waitForTransform(
	mrpt::poses::CPose3D& des, const std::string& target_frame,
	const std::string& source_frame, const ros::Time& time,
	const ros::Duration& timeout, const ros::Duration& polling_sleep_duration)
{
	geometry_msgs::TransformStamped transform;
	try
	{
		transform = tf_buffer_.lookupTransform(
			target_frame, source_frame, time, timeout);
	}
	catch (const tf2::TransformException& e)
	{
		ROS_WARN(
			"Failed to get transform target_frame (%s) to source_frame (%s): "
			"%s",
			target_frame.c_str(), source_frame.c_str(), e.what());
		return false;
	}
	tf2::Transform tx;
	tf2::fromMsg(transform.transform, tx);
	des = mrpt::ros1bridge::fromROS(tx);
	return true;
}

void RawlogRecordNode::convert(
	const nav_msgs::Odometry& src, mrpt::obs::CObservationOdometry& des)
{
	des.timestamp = mrpt::ros1bridge::fromROS(src.header.stamp);
	des.odometry =
		mrpt::poses::CPose2D(mrpt::ros1bridge::fromROS(src.pose.pose));

	std::string odom_frame_id = param_.odom_frame_id;
	des.sensorLabel = odom_frame_id;
	des.hasEncodersInfo = false;
	des.hasVelocities = false;
}

void RawlogRecordNode::callbackOdometry(const nav_msgs::Odometry& _msg)
{
	// ROS_INFO("callbackOdometry");
	if (!last_odometry_)
	{
		last_odometry_ = CObservationOdometry::Create();
	}
	convert(_msg, *last_odometry_);
	addObservation(_msg.header.stamp);
}

void RawlogRecordNode::callbackLaser(const sensor_msgs::LaserScan& _msg)
{
	// ROS_INFO("callbackLaser");
	if (!last_range_scan_)
	{
		last_range_scan_ = CObservation2DRangeScan::Create();
	}
	mrpt::poses::CPose3D sensor_pose_on_robot;

	if (getStaticTF(_msg.header.frame_id, sensor_pose_on_robot))
	{
		mrpt::ros1bridge::fromROS(
			_msg, sensor_pose_on_robot, *last_range_scan_);

		addObservation(_msg.header.stamp);
	}
}

void RawlogRecordNode::callbackMarker(const marker_msgs::MarkerDetection& _msg)
{
	// ROS_INFO("callbackMarker");
	if (!last_bearing_range_)
	{
		last_bearing_range_ = CObservationBearingRange::Create();
	}
	if (!last_beacon_range_)
	{
		last_beacon_range_ = CObservationBeaconRanges::Create();
	}
	mrpt::poses::CPose3D sensor_pose_on_robot;

	if (getStaticTF(_msg.header.frame_id, sensor_pose_on_robot))
	{
		mrpt_msgs_bridge::fromROS(
			_msg, sensor_pose_on_robot, *last_bearing_range_);

		last_bearing_range_->sensor_std_range =
			base_param_.bearing_range_std_range;
		last_bearing_range_->sensor_std_yaw = base_param_.bearing_range_std_yaw;
		last_bearing_range_->sensor_std_pitch =
			base_param_.bearing_range_std_pitch;

		mrpt_msgs_bridge::fromROS(
			_msg, sensor_pose_on_robot, *last_beacon_range_);
		last_beacon_range_->stdError = base_param_.bearing_range_std_range;
		addObservation(_msg.header.stamp);
	}
}

void RawlogRecordNode::addObservation(const ros::Time& time)
{
	sync_attempts_sensor_frame_++;
	if (sync_attempts_sensor_frame_ > 10)
		ROS_INFO("Problem to syn data for sensor frame!");

	if (!last_odometry_) return;
	auto odometry = CObservationOdometry::Create();
	*odometry = *last_odometry_;
	pRawLog.insert(odometry);

	CObservation2DRangeScan::Ptr range_scan;
	CObservationBearingRange::Ptr bearing_range;
	CObservationBeaconRanges::Ptr beacon_range;

	if (base_param_.record_range_scan)
	{
		if (!last_range_scan_) return;
		if (fabs(mrpt::system::timeDifference(
				last_odometry_->timestamp, last_range_scan_->timestamp)) >
			param_.sensor_frame_sync_threshold)
		{
			return;
		}
		range_scan = CObservation2DRangeScan::Create();
		*range_scan = *last_range_scan_;
		pRawLog.insert(range_scan);
	}

	if (base_param_.record_bearing_range)
	{
		if (!last_bearing_range_) return;
		if (fabs(mrpt::system::timeDifference(
				last_odometry_->timestamp, last_bearing_range_->timestamp)) >
			param_.sensor_frame_sync_threshold)
		{
			return;
		}
		bearing_range = CObservationBearingRange::Create();
		*bearing_range = *last_bearing_range_;
		pRawLog.insert(bearing_range);
	}
	if (base_param_.record_beacon_range)
	{
		if (!last_beacon_range_) return;
		if (fabs(mrpt::system::timeDifference(
				last_odometry_->timestamp, last_beacon_range_->timestamp)) >
			param_.sensor_frame_sync_threshold)
		{
			return;
		}
		beacon_range = CObservationBeaconRanges::Create();
		*beacon_range = *last_beacon_range_;
		pRawLog.insert(beacon_range);
	}

	static std::shared_ptr<mrpt::poses::CPose2D> lastOdomPose;
	if (!lastOdomPose)
	{
		lastOdomPose = std::make_shared<mrpt::poses::CPose2D>();
		*lastOdomPose = odometry->odometry;
	}

	mrpt::poses::CPose2D incOdoPose = odometry->odometry - *lastOdomPose;

	CActionRobotMovement2D odom_move;
	odom_move.timestamp = odometry->timestamp;
	odom_move.computeFromOdometry(incOdoPose, base_param_.motionModelOptions);
	auto action = CActionCollection::Create();
	action->insert(odom_move);
	pRawLogASF.insert(action);

	auto sf = CSensoryFrame::Create();
	if (base_param_.record_range_scan)
	{
		CObservation::Ptr obs_range_scan = CObservation::Ptr(range_scan);
		sf->insert(obs_range_scan);
	}

	if (base_param_.record_bearing_range)
	{
		CObservation::Ptr obs_bearing_range = CObservation::Ptr(bearing_range);
		sf->insert(obs_bearing_range);
	}
	if (base_param_.record_beacon_range)
	{
		CObservation::Ptr obs_bearing_range = CObservation::Ptr(beacon_range);
		sf->insert(obs_bearing_range);
	}
	pRawLogASF.insert(sf);

	*lastOdomPose = odometry->odometry;

	sync_attempts_sensor_frame_ = 0;
}

bool RawlogRecordNode::getStaticTF(
	std::string source_frame, mrpt::poses::CPose3D& des)
{
	std::string target_frame_id = param_.base_frame_id;
	std::string source_frame_id = source_frame;
	std::string key = target_frame_id + "->" + source_frame_id;
	mrpt::poses::CPose3D pose;

	geometry_msgs::TransformStamped tfGeom;

	if (static_tf_.find(key) == static_tf_.end())
	{
		if (base_param_.debug)
		{
			ROS_INFO(
				"debug: updateLaserPose(): target_frame_id='%s' "
				"source_frame_id='%s'",
				target_frame_id.c_str(), source_frame_id.c_str());
		}

		try
		{
			tfGeom = tf_buffer_.lookupTransform(
				target_frame_id, source_frame_id, ros::Time(0));
		}
		catch (const tf2::TransformException& ex)
		{
			ROS_INFO("getStaticTF");
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
			return false;
		}

		tf2::Transform transform;
		tf2::fromMsg(tfGeom.transform, transform);

		tf2::Vector3 translation = transform.getOrigin();
		tf2::Quaternion quat = transform.getRotation();
		pose.x() = translation.x();
		pose.y() = translation.y();
		pose.z() = translation.z();
		tf2::Matrix3x3 Rsrc(quat);
		mrpt::math::CMatrixDouble33 Rdes;
		for (int c = 0; c < 3; c++)
			for (int r = 0; r < 3; r++) Rdes(r, c) = Rsrc.getRow(r)[c];
		pose.setRotationMatrix(Rdes);
		static_tf_[key] = pose;
		ROS_INFO(
			"Static tf '%s' with '%s'", key.c_str(), pose.asString().c_str());
	}
	des = static_tf_[key];
	return true;
}
