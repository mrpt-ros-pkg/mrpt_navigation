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
#include <boost/interprocess/sync/scoped_lock.hpp>

#include <mrpt_bridge/pose.h>
#include <mrpt_bridge/laser_scan.h>
#include <mrpt_bridge/marker_msgs.h>
#include <mrpt_bridge/time.h>

#include <mrpt/version.h>
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
RawlogRecordNode::RawlogRecordNode(ros::NodeHandle& n)
    : RawlogRecord(new RawlogRecordNode::ParametersNode()),
      n_(n)
{
}

RawlogRecordNode::ParametersNode* RawlogRecordNode::param()
{
    return (RawlogRecordNode::ParametersNode*)param_;
}

void RawlogRecordNode::init()
{
    updateRawLogName(mrpt::system::getCurrentLocalTime());
    ROS_INFO("rawlog file: %s", param_->raw_log_name.c_str());
    if (param_->record_laser) {
        subLaser_ = n_.subscribe("scan", 1, &RawlogRecordNode::callbackLaser, this);
    }
    if (param_->record_bearing_range) {
        subMarker_ = n_.subscribe ( "marker", 1, &RawlogRecordNode::callbackMarker, this );
    }
    subOdometry_ = n_.subscribe ( "odom", 1, &RawlogRecordNode::callbackOdometry, this );
}

void RawlogRecordNode::loop()
{
    ros::spin();
}

bool RawlogRecordNode::waitForTransform(
    mrpt::poses::CPose3D& des, const std::string& target_frame,
    const std::string& source_frame, const ros::Time& time,
    const ros::Duration& timeout, const ros::Duration& polling_sleep_duration)
{
    tf::StampedTransform transform;
    try
    {
        if (param_->debug)
            ROS_INFO(
                "debug: waitForTransform(): target_frame='%s' "
                "source_frame='%s'",
                target_frame.c_str(), source_frame.c_str());

        listenerTF_.waitForTransform(
            target_frame, source_frame, time, polling_sleep_duration);
        listenerTF_.lookupTransform(
            target_frame, source_frame, time, transform);
    }
    catch (tf::TransformException)
    {
        ROS_INFO(
            "Failed to get transform target_frame (%s) to source_frame (%s)",
            target_frame.c_str(), source_frame.c_str());
        return false;
    }
    mrpt_bridge::convert(transform, des);
    return true;
}
bool RawlogRecordNode::waitForOdomTF(mrpt::obs::CObservationOdometry::Ptr &odometry, const ros::Time& time) {

    mrpt::poses::CPose3D poseOdom;
    std::string base_frame_id =
        tf::resolve(param()->tf_prefix, param()->base_frame_id);
    std::string odom_frame_id =
        tf::resolve(param()->tf_prefix, param()->odom_frame_id);
    if (this->waitForTransform( poseOdom, odom_frame_id, base_frame_id, time, ros::Duration(1))) {
        odometry->sensorLabel = odom_frame_id;
        odometry->hasEncodersInfo = false;
        odometry->hasVelocities = false;
        odometry->odometry.x() = poseOdom.x();
        odometry->odometry.y() = poseOdom.y();
        odometry->odometry.phi() = poseOdom.yaw();
        return true;
    } else {
        return false;
    }
}
bool RawlogRecordNode::getLastOdom(mrpt::obs::CObservationOdometry::Ptr &odometry) {

    if(last_odometery_) {
        std::string odom_frame_id = tf::resolve(param()->tf_prefix, param()->odom_frame_id);
        odometry->sensorLabel = odom_frame_id;
        odometry->hasEncodersInfo = false;
        odometry->hasVelocities = false;
        odometry->odometry.x() = last_odometery_->x();
        odometry->odometry.y() = last_odometery_->y();
        odometry->odometry.phi() = last_odometery_->yaw();
        return true;
    } else {
        return false;
    }
}


void RawlogRecordNode::callbackOdometry(const nav_msgs::Odometry& _msg)
{
    //ROS_INFO("callbackOdometry");
    if(!last_odometery_) {
        last_odometery_ = mrpt::make_aligned_shared<mrpt::poses::CPose3D>();
    }
    mrpt_bridge::convert(_msg.pose.pose, *last_odometery_);
}

void RawlogRecordNode::callbackLaser(const sensor_msgs::LaserScan& _msg)
{
    //ROS_INFO("callbackLaser");
    CObservation2DRangeScan::Ptr laser = mrpt::make_aligned_shared<CObservation2DRangeScan>();
    mrpt::poses::CPose3D sensor_pose_on_robot;

    if (getStaticTF(_msg.header.frame_id, sensor_pose_on_robot)) {
        mrpt_bridge::convert(_msg, sensor_pose_on_robot, last_2d_range_scan_);

        double time_diff = mrpt::system::timeDifference(last_2d_range_scan_.timestamp, last_bearing_range_.timestamp);
        //ROS_INFO("time_diff : %f", time_diff);
        if(fabs(time_diff) < 0.01) {
            addObservation(_msg.header.stamp);
        }
    }
}

void RawlogRecordNode::callbackMarker(const marker_msgs::MarkerDetection& _msg)
{
    //ROS_INFO("callbackMarker");
    CObservationBearingRange::Ptr bearing_range = mrpt::make_aligned_shared<CObservationBearingRange>();
    mrpt::poses::CPose3D sensor_pose_on_robot;

    if (getStaticTF(_msg.header.frame_id, sensor_pose_on_robot)) {
        mrpt_bridge::convert(_msg, sensor_pose_on_robot, last_bearing_range_);
        last_bearing_range_.sensor_std_range  = param_->bearing_range_std_range;
        last_bearing_range_.sensor_std_yaw    = param_->bearing_range_std_yaw;
        last_bearing_range_.sensor_std_pitch  = param_->bearing_range_std_pitch;

        double time_diff = mrpt::system::timeDifference(last_2d_range_scan_.timestamp, last_bearing_range_.timestamp);
        //ROS_INFO("time_diff : %f", time_diff);
        if(fabs(time_diff) < 0.01) {
            addObservation(_msg.header.stamp);
        }
    }
}

void RawlogRecordNode::addObservation(const ros::Time& time) {

    //ROS_INFO("addObservation");
    CObservationOdometry::Ptr odometry = mrpt::make_aligned_shared<CObservationOdometry>();
    //if (this->waitForOdomTF(odometry, time)) {
    if (this->getLastOdom(odometry)) {
        mrpt_bridge::convert(time, odometry->timestamp);
        pRawLog->addObservationMemoryReference(odometry);

        CObservationBearingRange::Ptr bearing_range = mrpt::make_aligned_shared<CObservationBearingRange>();
        *bearing_range = last_bearing_range_;
        pRawLog->addObservationMemoryReference(bearing_range);

        CObservation2DRangeScan::Ptr range_scan = mrpt::make_aligned_shared<CObservation2DRangeScan>();
        *range_scan = last_2d_range_scan_;
        pRawLog->addObservationMemoryReference(range_scan);

        if (odomLastPose_.empty())
        {
            odomLastPose_ = odometry->odometry;
        }

        mrpt::poses::CPose2D incOdoPose = odometry->odometry - odomLastPose_;

        CActionRobotMovement2D odom_move;
        odom_move.timestamp = odometry->timestamp;
        odom_move.computeFromOdometry(incOdoPose, param_->motionModelOptions);
        CActionCollection::Ptr action = mrpt::make_aligned_shared<CActionCollection>();
        action->insert(odom_move);
        pRawLogASF->addActionsMemoryReference(action);

        CSensoryFrame::Ptr sf = mrpt::make_aligned_shared<CSensoryFrame>();
        CObservation::Ptr obs_2d_range_scan = CObservation::Ptr(range_scan);
        CObservation::Ptr obs_bearing_range = CObservation::Ptr(bearing_range);
        sf->insert(obs_2d_range_scan);
        sf->insert(obs_bearing_range);
        pRawLogASF->addObservationsMemoryReference(sf);

        odomLastPose_ = odometry->odometry;

    }
    else
    {
        ROS_INFO("Failed to get odom for laser observation!");
    }
}

bool RawlogRecordNode::getStaticTF(std::string source_frame, mrpt::poses::CPose3D &des)
{
    std::string target_frame_id = tf::resolve(param()->tf_prefix, param()->base_frame_id);
    std::string source_frame_id = source_frame;
    std::string key = target_frame_id + "->" + source_frame_id;
    mrpt::poses::CPose3D pose;
    tf::StampedTransform transform;

    if (static_tf_.find(key) == markers_poses_.end()) {

        try
        {
            if (param_->debug)
                ROS_INFO(
                    "debug: updateLaserPose(): target_frame_id='%s' source_frame_id='%s'",
                    target_frame_id.c_str(), source_frame_id.c_str());

            listenerTF_.lookupTransform(
                target_frame_id, source_frame_id, ros::Time(0), transform);
            ROS_INFO("Requesting tf target_frame_id='%s' source_frame_id='%s'",
                     target_frame_id.c_str(), source_frame_id.c_str());
            tf::Vector3 translation = transform.getOrigin();
            tf::Quaternion quat = transform.getRotation();
            pose.x() = translation.x();
            pose.y() = translation.y();
            pose.z() = translation.z();
            tf::Matrix3x3 Rsrc(quat);
            mrpt::math::CMatrixDouble33 Rdes;
            for (int c = 0; c < 3; c++)
                for (int r = 0; r < 3; r++) Rdes(r, c) = Rsrc.getRow(r)[c];
            pose.setRotationMatrix(Rdes);
            static_tf_[key] = pose;
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            return false;
        }
    }
    des = static_tf_[key];
    return true;

}
