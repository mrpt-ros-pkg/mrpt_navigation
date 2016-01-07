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

#include "rawlog_record_node.h"
#include <boost/interprocess/sync/scoped_lock.hpp>

#include <mrpt_bridge/pose.h>
#include <mrpt_bridge/laser_scan.h>
#include <mrpt_bridge/time.h>

#include <mrpt/version.h>
#if MRPT_VERSION>=0x130
	using namespace mrpt::maps;
	using namespace mrpt::obs;
#else
	using namespace mrpt::slam;
#endif

int main(int argc, char **argv) {

	ros::init(argc, argv, "rawlog_record");
    ros::NodeHandle n;
    RawlogRecordNode my_node(n);
    my_node.init();
    my_node.loop();
    return 0;
}

RawlogRecordNode::~RawlogRecordNode() {
}

RawlogRecordNode::RawlogRecordNode(ros::NodeHandle &n) :
    RawlogRecord(new RawlogRecordNode::ParametersNode()), n_(n), loop_count_(0) {

}

RawlogRecordNode::ParametersNode *RawlogRecordNode::param() {
    return (RawlogRecordNode::ParametersNode*) param_;
}

void RawlogRecordNode::init() {
    updateRawLogName(mrpt::system::getCurrentLocalTime());
    ROS_INFO("rawlog file: %s", param_->raw_log_name.c_str());
    subLaser0_ = n_.subscribe("scan", 1, &RawlogRecordNode::callbackLaser, this);
    subLaser1_ = n_.subscribe("scan1", 1, &RawlogRecordNode::callbackLaser, this);
    subLaser2_ = n_.subscribe("scan2", 1, &RawlogRecordNode::callbackLaser, this);
}

void RawlogRecordNode::loop() {
    for (ros::Rate rate(param()->rate); ros::ok(); loop_count_++) {
        param()->update(loop_count_);
        ros::spinOnce();
        rate.sleep();
    }
}


bool RawlogRecordNode::waitForTransform(mrpt::poses::CPose3D &des, const std::string& target_frame, const std::string& source_frame, const ros::Time& time, const ros::Duration& timeout, const ros::Duration& polling_sleep_duration){
    tf::StampedTransform transform;
    try
    {
	if(param_->debug)
		ROS_INFO("debug: waitForTransform(): target_frame='%s' source_frame='%s'", target_frame.c_str(), source_frame.c_str() );

        listenerTF_.waitForTransform(target_frame, source_frame,  time, polling_sleep_duration);
        listenerTF_.lookupTransform(target_frame, source_frame,  time, transform);
    }
    catch(tf::TransformException)
    {
        ROS_INFO("Failed to get transform target_frame (%s) to source_frame (%s)", target_frame.c_str(), source_frame.c_str());
        return false;
    }
    mrpt_bridge::convert(transform, des);
    return true;
}

void RawlogRecordNode::callbackLaser (const sensor_msgs::LaserScan &_msg) {
    //ROS_INFO("callbackLaser");
	CObservation2DRangeScanPtr laser = CObservation2DRangeScan::Create();

    if(laser_poses_.find(_msg.header.frame_id) == laser_poses_.end()) {
        updateLaserPose (_msg.header.frame_id);
    } else {
        //mrpt::poses::CPose3D pose = laser_poses_[_msg.header.frame_id];
        //ROS_INFO("LASER POSE %4.3f, %4.3f, %4.3f, %4.3f, %4.3f, %4.3f",  pose.x(), pose.y(), pose.z(), pose.roll(), pose.pitch(), pose.yaw());
        mrpt_bridge::convert(_msg, laser_poses_[_msg.header.frame_id],  *laser);


        std::string base_frame_id = tf::resolve(param()->tf_prefix, param()->base_frame_id);
        std::string odom_frame_id = tf::resolve(param()->tf_prefix, param()->odom_frame_id);
        mrpt::poses::CPose3D poseOdom;
        if(this->waitForTransform(poseOdom, odom_frame_id, base_frame_id, _msg.header.stamp, ros::Duration(1))){
			CObservationOdometryPtr odometry = CObservationOdometry::Create();
            odometry->sensorLabel = odom_frame_id;
            odometry->hasEncodersInfo = false;
            odometry->hasVelocities = false;
            odometry->odometry.x() = poseOdom.x();
            odometry->odometry.y() = poseOdom.y();
            odometry->odometry.phi() = poseOdom.yaw();

            observation(laser, odometry);
        } else {
            ROS_INFO("Failed to get odom for laser observation!");
        }
    }
}

void RawlogRecordNode::updateLaserPose (std::string _frame_id) {
    std::string base_frame_id = tf::resolve(param()->tf_prefix, param()->base_frame_id);
    mrpt::poses::CPose3D pose;
    tf::StampedTransform transform;
    try {
	if(param_->debug)
		ROS_INFO("debug: updateLaserPose(): base_frame_id='%s' _frame_id='%s'", base_frame_id.c_str(), _frame_id.c_str() );

        listenerTF_.lookupTransform(base_frame_id, _frame_id, ros::Time(0), transform);
        ROS_INFO("Requesting laser pose for %s!", _frame_id.c_str());
        tf::Vector3 translation = transform.getOrigin();
        tf::Quaternion quat = transform.getRotation();
        pose.x() = translation.x();
        pose.y() = translation.y();
        pose.z() = translation.z();
        double roll, pitch, yaw;
        tf::Matrix3x3 Rsrc(quat);
        mrpt::math::CMatrixDouble33 Rdes;
        for(int c = 0; c < 3; c++)
            for(int r = 0; r < 3; r++)
                Rdes(r,c) = Rsrc.getRow(r)[c];
        pose.setRotationMatrix(Rdes);
        laser_poses_[_frame_id] = pose;
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

}

