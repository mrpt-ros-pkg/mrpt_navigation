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

#ifndef MRPT_RAWLOG_PLAY_NODE_H
#define MRPT_RAWLOG_PLAY_NODE_H

#include <dynamic_reconfigure/server.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt_msgs/ObservationRangeBeacon.h>
#include <mrpt_msgs/ObservationRangeBearing.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include "geometry_msgs/TransformStamped.h"
#include "mrpt_rawlog/RawLogRecordConfig.h"
#include "mrpt_rawlog_play/rawlog_play.h"
#include "ros/ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"

/// ROS Node
class RawlogPlayNode : public RawlogPlay
{
   public:
	struct ParametersNode : public Parameters
	{
		static const int MOTION_MODEL_GAUSSIAN = 0;
		static const int MOTION_MODEL_THRUN = 1;
		ParametersNode();
		ros::NodeHandle node;
		void callbackParameters(
			mrpt_rawlog::RawLogRecordConfig& config, uint32_t level);
		dynamic_reconfigure::Server<mrpt_rawlog::RawLogRecordConfig>
			reconfigureServer_;
		dynamic_reconfigure::Server<
			mrpt_rawlog::RawLogRecordConfig>::CallbackType reconfigureFnc_;
		void update(const unsigned long& loop_count);
		double rate;
		std::string base_frame;
		std::string odom_frame;
		int parameter_update_skip;
	};

	RawlogPlayNode(ros::NodeHandle& n);
	~RawlogPlayNode();
	void init();
	void loop();

   private:	 // functions
	ParametersNode* param();
	bool nextEntry();
	void publishSingleObservation(const mrpt::obs::CObservation::Ptr& o);

   private:	 // variables
	ros::NodeHandle n_;
	unsigned long loop_count_;
	sensor_msgs::LaserScan msg_laser_;
	mrpt_msgs::ObservationRangeBeacon msg_beacon_;
	mrpt_msgs::ObservationRangeBearing msg_landmark_;
	nav_msgs::Odometry msg_odom_;
	ros::Publisher pub_laser_;
	ros::Publisher pub_beacon_;
	ros::Publisher pub_landmark_;
	std::string odom_frame_;
	std::string base_frame_;
	tf2_ros::TransformBroadcaster tf_broadcaster_;
};

#endif	// MRPT_RAWLOG_PLAY_NODE_H
