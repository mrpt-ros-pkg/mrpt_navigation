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

#ifndef MRPT_RAWLOG_RECORD_NODE_H
#define MRPT_RAWLOG_RECORD_NODE_H

#include <dynamic_reconfigure/server.h>
#include <marker_msgs/MarkerDetection.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include "mrpt_rawlog/RawLogRecordConfig.h"
#include "mrpt_rawlog_record/rawlog_record.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

/// ROS Node
class RawlogRecordNode : public RawlogRecord
{
   public:
	struct ParametersNode
	{
		static const int MOTION_MODEL_GAUSSIAN = 0;
		static const int MOTION_MODEL_THRUN = 1;
		ParametersNode(RawlogRecord::Parameters& base_params);
		ros::NodeHandle node;
		RawlogRecord::Parameters& base_param_;
		void callbackParameters(
			mrpt_rawlog::RawLogRecordConfig& config, uint32_t level);
		dynamic_reconfigure::Server<mrpt_rawlog::RawLogRecordConfig>
			reconfigureServer_;
		dynamic_reconfigure::Server<
			mrpt_rawlog::RawLogRecordConfig>::CallbackType reconfigureFnc_;
		void update(const unsigned long& loop_count);
		double rate;
		int parameter_update_skip;
		std::string odom_frame_id;
		std::string base_frame_id;
		double sensor_frame_sync_threshold;
	};

	RawlogRecordNode(ros::NodeHandle& n);
	~RawlogRecordNode();
	void init();
	void loop();
	void callbackLaser(
		const sensor_msgs::LaserScan&);	 /// callback function to catch motion
										 /// commands
	void callbackMarker(const marker_msgs::MarkerDetection&);
	void callbackOdometry(const nav_msgs::Odometry&);

   private:	 // functions
	ParametersNode param_{RawlogRecord::base_param_};
	void update();
	bool getStaticTF(std::string source_frame, mrpt::poses::CPose3D& des);
	ros::Subscriber subLaser_;
	ros::Subscriber subMarker_;
	ros::Subscriber subOdometry_;

	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener listenerTF_{tf_buffer_};

	mrpt::obs::CObservationBearingRange::Ptr last_bearing_range_;
	mrpt::obs::CObservationBeaconRanges::Ptr last_beacon_range_;
	mrpt::obs::CObservation2DRangeScan::Ptr last_range_scan_;
	mrpt::obs::CObservationOdometry::Ptr last_odometry_;
	unsigned int sync_attempts_sensor_frame_;
	std::map<std::string, mrpt::poses::CPose3D> static_tf_;
	ros::NodeHandle n_;
	void addObservation(const ros::Time& time);
	bool waitForTransform(
		mrpt::poses::CPose3D& des, const std::string& target_frame,
		const std::string& source_frame, const ros::Time& time,
		const ros::Duration& timeout,
		const ros::Duration& polling_sleep_duration = ros::Duration(0.01));

	void convert(
		const nav_msgs::Odometry& src, mrpt::obs::CObservationOdometry& des);
};

#endif	// MRPT_RAWLOG_RECORD_NODE_H
