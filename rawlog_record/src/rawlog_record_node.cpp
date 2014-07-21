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
#include <mrpt/base.h>
#include <mrpt/slam.h>
#include <mrpt/gui.h>


int main(int argc, char **argv) {

    ros::init(argc, argv, "DataLogger");
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
    subOdometry_ = n_.subscribe("odom", 1, &RawlogRecordNode::callbackOdometry, this);
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

void RawlogRecordNode::callbackLaser (const sensor_msgs::LaserScan &_msg) {
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutexRawLog);
    mrpt::slam::CObservation2DRangeScanPtr laser = mrpt::slam::CObservation2DRangeScan::Create();

    if(laser_poses_.find(_msg.header.frame_id) == laser_poses_.end()) {
        updateLaserPose (_msg.header.frame_id);
    } else {
        mrpt::poses::CPose3D pose = laser_poses_[_msg.header.frame_id];  
        ROS_INFO("LASER POSE %4.3f, %4.3f, %4.3f, %4.3f, %4.3f, %4.3f",
                 pose.x(), pose.y(), pose.z(), pose.roll(), pose.pitch(), pose.yaw());
        mrpt_bridge::laser_scan::ros2mrpt(_msg, laser_poses_[_msg.header.frame_id],  *laser);
        incommingLaserData(laser);
    }
}

void RawlogRecordNode::updateLaserPose (std::string _frame_id) {
    if(base_link_.empty()) return;
    mrpt::poses::CPose3D pose;
    tf::StampedTransform transform;
    try {
        listenerTF_.lookupTransform(base_link_, _frame_id, ros::Time(0), transform);
        tf::Vector3 translation = transform.getOrigin();
        tf::Quaternion quat = transform.getRotation();
        pose.x() = translation.x();
        pose.y() = translation.y();
        pose.z() = translation.z();
        double roll, pitch, yaw;
        tf::Matrix3x3 Rsrc(quat);
        mrpt::poses::CMatrixDouble33 Rdes;
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

void RawlogRecordNode::callbackOdometry (const nav_msgs::Odometry &_odom) {
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutexRawLog);

    if(base_link_.empty()) {
        base_link_ = _odom.child_frame_id;
    }
    
    mrpt::poses::CPose2D odoPose;
    mrpt_bridge::poses::ros2mrpt(_odom.pose.pose, odoPose);
    
    mrpt::slam::CObservationOdometryPtr odometry = mrpt::slam::CObservationOdometry::Create();
    odometry->sensorLabel = "ODOMETRY";
    odometry->hasEncodersInfo = false;
    odometry->hasVelocities = false;
    odometry->odometry = odoPose;
    mrpt_bridge::time::ros2mrpt(_odom.header.stamp, odometry->timestamp);
    incommingOdomData(odometry);
}
