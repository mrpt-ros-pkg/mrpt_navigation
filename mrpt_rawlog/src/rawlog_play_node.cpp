/***********************************************************************************
 * Revised BSD License                                                             *
 * Copyright (c) 2014, Markus Bader <markus.bader@tuwien.ac.at>                    *
 * All rights reserved.                                                            *
 *                                                                                 *
 * Redistribution and use in source and binary forms, with or without              *
 * modification, are permitted provided that the following conditions are met:     *
 *     * Redistributions of source code must retain the above copyright            *
 *       notice, this list of conditions and the following disclaimer.             *
 *     * Redistributions in binary form must reproduce the above copyright         *
 *       notice, this list of conditions and the following disclaimer in the       *
 *       documentation and/or other materials provided with the distribution.      *
 *     * Neither the name of the Vienna University of Technology nor the           *
 *       names of its contributors may be used to endorse or promote products      *
 *       derived from this software without specific prior written permission.     *
 *                                                                                 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND *
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED   *
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE          *
 * DISCLAIMED. IN NO EVENT SHALL Markus Bader BE LIABLE FOR ANY                    *
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES      *
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;    *
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND     *
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT      *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS   *
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                    *                       *
 ***********************************************************************************/

#include "rawlog_play_node.h"
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <mrpt_bridge/pose.h>
#include <mrpt_bridge/laser_scan.h>
#include <mrpt_bridge/time.h>

#include <mrpt/system/filesystem.h>

#include <mrpt/version.h>
#if MRPT_VERSION>=0x130
#	include <mrpt/obs/CSensoryFrame.h>
#	include <mrpt/obs/CRawlog.h>
#	include <mrpt/obs/CObservation2DRangeScan.h>
	using namespace mrpt::obs;
#else
#	include <mrpt/slam/CSensoryFrame.h>
#	include <mrpt/slam/CRawlog.h>
#	include <mrpt/slam/CObservation2DRangeScan.h>
	using namespace mrpt::slam;
#endif


int main(int argc, char **argv) {

    ros::init(argc, argv, "DataLogger");
    ros::NodeHandle n;
    RawlogPlayNode my_node(n);
    my_node.init();
    my_node.loop();
    return 0;
}

RawlogPlayNode::~RawlogPlayNode() {
}

RawlogPlayNode::RawlogPlayNode(ros::NodeHandle &n) :
    RawlogPlay(new RawlogPlayNode::ParametersNode()), n_(n), loop_count_(0) {

}

RawlogPlayNode::ParametersNode *RawlogPlayNode::param() {
    return (RawlogPlayNode::ParametersNode*) param_;
}

void RawlogPlayNode::init() {

    if(!mrpt::system::fileExists(param_->rawlog_file)) {
        ROS_ERROR("raw_file: %s does not exit", param_->rawlog_file.c_str());
    }
    rawlog_stream_.open(param_->rawlog_file);
    pub_laser_ = n_.advertise<sensor_msgs::LaserScan>("scan", 10);
    odom_frame_ = tf::resolve(param()->tf_prefix, param()->odom_frame);
    base_frame_ = tf::resolve(param()->tf_prefix, param()->base_frame);
    robotPose = mrpt::poses::CPose3DPDFGaussian();
}

bool RawlogPlayNode::nextEntry() {
	CActionCollectionPtr action;
	CSensoryFramePtr     observations;
	CObservationPtr      obs;

	if(!CRawlog::getActionObservationPairOrObservation( rawlog_stream_, action, observations, obs, entry_)) {
        ROS_INFO("end of stream!");
        return true;
    }
    mrpt::poses::CPose3D pose_laser;
    geometry_msgs::Pose msg_pose_laser;
    tf::Transform transform;

    // loop over laser overservations
    for(size_t i = 0;i < observations->size() ;i++){
		CObservation2DRangeScanPtr laser = observations->getObservationByClass<CObservation2DRangeScan>(i);
        if(laser.pointer() == NULL) break;
        mrpt_bridge::convert(*laser, msg_laser_, msg_pose_laser);
        laser->getSensorPose(pose_laser);
        mrpt_bridge::convert(pose_laser, transform);
        std::string childframe = tf::resolve(param()->tf_prefix, msg_laser_.header.frame_id);
        tf_broadcaster_.sendTransform(tf::StampedTransform(transform, msg_laser_.header.stamp, base_frame_, childframe));
        pub_laser_.publish(msg_laser_);
		laser = observations->getObservationByClass<CObservation2DRangeScan>();
    }
    mrpt::poses::CPose3DPDFGaussian out_pose_increment;
    action->getFirstMovementEstimation (out_pose_increment);
    robotPose -= out_pose_increment;

    msg_odom_.header.frame_id = "odom";
    msg_odom_.child_frame_id = base_frame_;
    msg_odom_.header.stamp = msg_laser_.header.stamp;
    msg_odom_.header.seq = msg_laser_.header.seq;
    mrpt_bridge::convert(robotPose, msg_odom_.pose);
    mrpt_bridge::convert(robotPose, transform);
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform, msg_odom_.header.stamp, base_frame_, odom_frame_));
    return false;

}

void RawlogPlayNode::loop() {
    bool end = false;
    for (ros::Rate rate(param()->rate); ros::ok() && !end; loop_count_++) {
        param()->update(loop_count_);
        end = nextEntry();
        ros::spinOnce();
        rate.sleep();
    }
}
