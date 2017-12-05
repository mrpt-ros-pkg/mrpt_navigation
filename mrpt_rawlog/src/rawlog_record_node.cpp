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
    if (param_->record_range_scan) {
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

void RawlogRecordNode::convert(const nav_msgs::Odometry& src, mrpt::obs::CObservationOdometry &des) {
    mrpt_bridge::convert(src.header.stamp, des.timestamp);
    mrpt_bridge::convert(src.pose.pose, des.odometry);
    std::string odom_frame_id = tf::resolve(param()->tf_prefix, param()->odom_frame_id);
    des.sensorLabel = odom_frame_id;
    des.hasEncodersInfo = false;
    des.hasVelocities = false;

}

void RawlogRecordNode::callbackOdometry(const nav_msgs::Odometry& _msg)
{
    //ROS_INFO("callbackOdometry");
    if(!last_odometry_) {
        last_odometry_ = CObservationOdometry::Create();
    }
    convert(_msg, *last_odometry_);
    addObservation(_msg.header.stamp);
}

void RawlogRecordNode::callbackLaser(const sensor_msgs::LaserScan& _msg)
{
    //ROS_INFO("callbackLaser");
    if(!last_range_scan_) {
        last_range_scan_ = CObservation2DRangeScan::Create();
    }
    mrpt::poses::CPose3D sensor_pose_on_robot;

    if (getStaticTF(_msg.header.frame_id, sensor_pose_on_robot)) {
        mrpt_bridge::convert(_msg, sensor_pose_on_robot, *last_range_scan_);

        addObservation(_msg.header.stamp);

    }
}

void RawlogRecordNode::callbackMarker(const marker_msgs::MarkerDetection& _msg)
{
    //ROS_INFO("callbackMarker");
    if(!last_bearing_range_) {
        last_bearing_range_ = CObservationBearingRange::Create();
    }
    if(!last_beacon_range_) {
        last_beacon_range_ = CObservationBeaconRanges::Create();
    }
    mrpt::poses::CPose3D sensor_pose_on_robot;

    if (getStaticTF(_msg.header.frame_id, sensor_pose_on_robot)) {
        mrpt_bridge::convert(_msg, sensor_pose_on_robot, *last_bearing_range_);
        last_bearing_range_->sensor_std_range  = param_->bearing_range_std_range;
        last_bearing_range_->sensor_std_yaw    = param_->bearing_range_std_yaw;
        last_bearing_range_->sensor_std_pitch  = param_->bearing_range_std_pitch;

        mrpt_bridge::convert(_msg, sensor_pose_on_robot, *last_beacon_range_);
        last_beacon_range_->stdError = param_->bearing_range_std_range;
        addObservation(_msg.header.stamp);
    }

}

void RawlogRecordNode::addObservation(const ros::Time& time) {


    sync_attempts_sensor_frame_++;
    if(sync_attempts_sensor_frame_ > 10) ROS_INFO("Problem to syn data for sensor frame!");

    if(!last_odometry_) return;
    auto odometry = CObservationOdometry::Create();
    *odometry = *last_odometry_;
    pRawLog->addObservationMemoryReference(odometry);

    CObservation2DRangeScan::Ptr range_scan;
    CObservationBearingRange::Ptr bearing_range;
    CObservationBeaconRanges::Ptr beacon_range;

    if (param_->record_range_scan) {
        if(!last_range_scan_) return;
        if( fabs(mrpt::system::timeDifference(last_odometry_->timestamp, last_range_scan_->timestamp)) > param()->sensor_frame_sync_threshold) {
            return;
        }
        range_scan = CObservation2DRangeScan::Create();
        *range_scan = *last_range_scan_;
        pRawLog->addObservationMemoryReference(range_scan);
    }

    if (param_->record_bearing_range) {
        if(!last_bearing_range_) return;
        if( fabs(mrpt::system::timeDifference(last_odometry_->timestamp, last_bearing_range_->timestamp)) > param()->sensor_frame_sync_threshold) {
            return;
        }
        bearing_range = CObservationBearingRange::Create();
        *bearing_range = *last_bearing_range_;
        pRawLog->addObservationMemoryReference(bearing_range);
    }
    if (param_->record_beacon_range) {
        if(!last_beacon_range_) return;
        if( fabs(mrpt::system::timeDifference(last_odometry_->timestamp, last_beacon_range_->timestamp)) > param()->sensor_frame_sync_threshold) {
            return;
        }
        beacon_range = CObservationBeaconRanges::Create();
        *beacon_range = *last_beacon_range_;
        pRawLog->addObservationMemoryReference(beacon_range);
    }


    static std::shared_ptr<mrpt::poses::CPose2D> lastOdomPose;
    if(!lastOdomPose) {
        lastOdomPose = std::make_shared<mrpt::poses::CPose2D>();
        *lastOdomPose = odometry->odometry;
    }

    mrpt::poses::CPose2D incOdoPose = odometry->odometry - *lastOdomPose;

    CActionRobotMovement2D odom_move;
    odom_move.timestamp = odometry->timestamp;
    odom_move.computeFromOdometry(incOdoPose, param_->motionModelOptions);
    auto action = CActionCollection::Create();
    action->insert(odom_move);
    pRawLogASF->addActionsMemoryReference(action);

    auto sf = CSensoryFrame::Create();
    if (param_->record_range_scan) {
        CObservation::Ptr obs_range_scan = CObservation::Ptr(range_scan);
        sf->insert(obs_range_scan);
    }

    if (param_->record_bearing_range) {
        CObservation::Ptr obs_bearing_range = CObservation::Ptr(bearing_range);
        sf->insert(obs_bearing_range);
    }
    if (param_->record_beacon_range) {
        CObservation::Ptr obs_bearing_range = CObservation::Ptr(beacon_range);
        sf->insert(obs_bearing_range);
    }
    pRawLogASF->addObservationsMemoryReference(sf);

    *lastOdomPose = odometry->odometry;

    sync_attempts_sensor_frame_ = 0;

}

bool RawlogRecordNode::getStaticTF(std::string source_frame, mrpt::poses::CPose3D &des)
{
    std::string target_frame_id = tf::resolve(param()->tf_prefix, param()->base_frame_id);
    std::string source_frame_id = source_frame;
    std::string key = target_frame_id + "->" + source_frame_id;
    mrpt::poses::CPose3D pose;
    tf::StampedTransform transform;

    if (static_tf_.find(key) == static_tf_.end()) {

        try
        {
            if (param_->debug)
                ROS_INFO(
                    "debug: updateLaserPose(): target_frame_id='%s' source_frame_id='%s'",
                    target_frame_id.c_str(), source_frame_id.c_str());

            listenerTF_.lookupTransform(
                target_frame_id, source_frame_id, ros::Time(0), transform);
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
            ROS_INFO("Static tf '%s' with '%s'",
                     key.c_str(), pose.asString().c_str());
        }
        catch (tf::TransformException ex)
        {
            ROS_INFO("getStaticTF");
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            return false;
        }
    }
    des = static_tf_[key];
    return true;

}
