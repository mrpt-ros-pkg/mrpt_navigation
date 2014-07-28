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

#include "mrpt_localization_node.h"
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <geometry_msgs/PoseArray.h>

#include <mrpt_bridge/pose.h>
#include <mrpt_bridge/laser_scan.h>
#include <mrpt_bridge/time.h>
#include <mrpt_bridge/map.h>
#include <mrpt/base.h>
#include <mrpt/slam.h>
#include <mrpt/gui.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "localization");
    ros::NodeHandle n;
    PFLocalizationNode my_node(n);
    my_node.init();
    my_node.loop();
    return 0;
}

PFLocalizationNode::~PFLocalizationNode() {
}

PFLocalizationNode::PFLocalizationNode(ros::NodeHandle &n) :
    PFLocalization(new PFLocalizationNode::Parameters()), n_(n), loop_count_(0) {
}

PFLocalizationNode::Parameters *PFLocalizationNode::param() {
    return (PFLocalizationNode::Parameters*) param_;
}

void PFLocalizationNode::init() {
    PFLocalization::init();
    subOdometry_ = n_.subscribe("odom", 1, &PFLocalizationNode::callbackOdometry, this);
    subLaser0_ = n_.subscribe("scan", 1, &PFLocalizationNode::callbackLaser, this);
    subLaser1_ = n_.subscribe("scan1", 1, &PFLocalizationNode::callbackLaser, this);
    subLaser2_ = n_.subscribe("scan2", 1, &PFLocalizationNode::callbackLaser, this);
    
    subLaser2_ = n_.subscribe("initialpose", 1, &PFLocalizationNode::callbackInitialpose, this);
    
    if(!param()->mapFile.empty()){
        mrpt_bridge::convert(*metric_map_.m_gridMaps[0], resp_.map);
        pub_map_ = n_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
        pub_metadata_= n_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
        service_map_ = n_.advertiseService("static_map", &PFLocalizationNode::mapCallback, this);
    }
    pub_Particles_ = n_.advertise<geometry_msgs::PoseArray>("particlecloud", 1, true);
}

void PFLocalizationNode::loop() {
    ROS_INFO("loop");
    for (ros::Rate rate(param()->rate); ros::ok(); loop_count_++) {
        param()->update(loop_count_);
        if(loop_count_%param()->map_update_skip == 0)   publishMap();
        if(loop_count_%param()->parameter_update_skip == 0)   publishParticles();
        publishTF_Map2Odom();
        //publishTF_Base2Map(); // does not work
        ros::spinOnce();
        rate.sleep();
    }
}



void PFLocalizationNode::callbackLaser (const sensor_msgs::LaserScan &_msg) {
    //ROS_INFO("callbackLaser");
    mrpt::slam::CObservation2DRangeScanPtr laser = mrpt::slam::CObservation2DRangeScan::Create();

    //printf("callbackLaser %s\n", _msg.header.frame_id.c_str());
    if(laser_poses_.find(_msg.header.frame_id) == laser_poses_.end()) {
        updateLaserPose (_msg.header.frame_id);
    } else {
        mrpt::poses::CPose3D pose = laser_poses_[_msg.header.frame_id];
        //ROS_INFO("LASER POSE %4.3f, %4.3f, %4.3f, %4.3f, %4.3f, %4.3f",  pose.x(), pose.y(), pose.z(), pose.roll(), pose.pitch(), pose.yaw());
        mrpt_bridge::convert(_msg, laser_poses_[_msg.header.frame_id],  *laser);
        incommingLaserData(laser);
    }
}

bool PFLocalizationNode::waitForMap(){
    clientMap_ = n_.serviceClient<nav_msgs::GetMap>("static_map");
    nav_msgs::GetMap srv;
    while (!clientMap_.call(srv) && ros::ok()){
        ROS_INFO("waiting for map service!");
        sleep(1);
    }
    updateMap (srv.response.map);
    clientMap_.shutdown();
}

void PFLocalizationNode::updateLaserPose (std::string _frame_id) {
    mrpt::poses::CPose3D pose;
    tf::StampedTransform transform;
    try {
        listenerTF_.lookupTransform(param()->base_frame_id, _frame_id, ros::Time(0), transform);
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

void PFLocalizationNode::callbackOdometry (const nav_msgs::Odometry &_odom) {
    //ROS_INFO("callbackOdometry");
    mrpt::poses::CPose2D odoPose;
    mrpt_bridge::convert(_odom.pose.pose, odoPose);

    mrpt::slam::CObservationOdometryPtr odometry = mrpt::slam::CObservationOdometry::Create();
    //odometry->sensorLabel = "ODOMETRY";
    odometry->hasEncodersInfo = false;
    odometry->hasVelocities = false;
    odometry->odometry = odoPose;
    mrpt_bridge::convert(_odom.header.stamp, odometry->timestamp);
    incommingOdomData(odometry);
}
void PFLocalizationNode::callbackInitialpose (const geometry_msgs::PoseWithCovarianceStamped& _msg){
    const geometry_msgs::PoseWithCovariance &pose = _msg.pose;
    mrpt_bridge::convert(pose, initialPose_);
    printf("callbackInitialpose");
    state_ = INIT;
}

void PFLocalizationNode::updateMap (const nav_msgs::OccupancyGrid &_msg) {
    ASSERT_( metric_map_.m_gridMaps.size()==1 );
    mrpt_bridge::convert(_msg, *metric_map_.m_gridMaps[0]);
}

bool PFLocalizationNode::mapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res )
{
    ROS_INFO("mapCallback: service requested!\n");
    res = resp_;
    return true;
}

void PFLocalizationNode::publishMap () {
    resp_.map.header.stamp = ros::Time::now();
    resp_.map.header.frame_id =  param()->global_frame_id;
    resp_.map.header.seq = loop_count_;
    if(pub_map_.getNumSubscribers() > 0){
        pub_map_.publish(resp_.map );
    }
    if(pub_metadata_.getNumSubscribers() > 0){
        pub_metadata_.publish( resp_.map.info );
    }
}

void PFLocalizationNode::publishParticles () {
    geometry_msgs::PoseArray poseArray;
    std::string global_frame = param()->global_frame_id;
    poseArray.header.frame_id = global_frame;
    poseArray.header.stamp = ros::Time::now();
    poseArray.header.seq = loop_count_;
    poseArray.poses.resize(pdf_.particlesCount());
    for(size_t i = 0; i < pdf_.particlesCount(); i++){
        mrpt::poses::CPose2D p = pdf_.getParticlePose(i);
        mrpt_bridge::convert(p, poseArray.poses[i]);
    }
    mrpt::poses::CPose2D p;
    pub_Particles_.publish(poseArray);
}

void PFLocalizationNode::publishTF_Map2Odom () {
    // Most of this code was copy and pase form ros::amcl
    // sorry it is realy ugly
    mrpt::poses::CPose2D robotPose;
    pdf_.getMean(robotPose);
    std::string global_frame_id = param()->global_frame_id;
    std::string odom_frame = param()->odom_frame_id;
    tf::Stamped<tf::Pose> odom_to_map;
    tf::Transform tmp_tf;
    mrpt_bridge::convert(robotPose, tmp_tf);
    ros::Time stamp;
    try
    {
        mrpt_bridge::convert(timeLastUpdate_, stamp);
        tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(),
                                              stamp,
                                              param()->base_frame_id);
        listenerTF_.transformPose(odom_frame,
                                  tmp_tf_stamped,
                                  odom_to_map);
    }
    catch(tf::TransformException)
    {
        ROS_INFO("Failed to subtract base to %s transform", odom_frame.c_str());
        return;
    }

    tf::Transform latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                                             tf::Point(odom_to_map.getOrigin()));

    // We want to send a transform that is good up until a
    // tolerance time so that odom can be used
    ros::Duration transform_tolerance_(0.5);
    ros::Time transform_expiration = (stamp + transform_tolerance_);
    tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                        transform_expiration,
                                        global_frame_id, odom_frame);
    tf_broadcaster_.sendTransform(tmp_tf_stamped);
    //ROS_INFO("%s, %s\n", global_frame_id.c_str(), odom_frame.c_str());
}

void PFLocalizationNode::publishTF_Base2Map () {
    // This does not work
    mrpt::poses::CPose2D robotPose;
    pdf_.getMean(robotPose);
    std::string global_frame_id = param()->global_frame_id;
    tf::Transform tmp_tf;
    mrpt_bridge::convert(robotPose, tmp_tf);
    ros::Time stamp;
    ros::Duration transform_tolerance_(0.5);
    mrpt_bridge::convert(timeLastUpdate_, stamp);
    ros::Time transform_expiration = (stamp + transform_tolerance_);
    tf::StampedTransform tmp_tf_stamped(tmp_tf.inverse(), transform_expiration, global_frame_id, param()->base_frame_id);
    tf_broadcaster_.sendTransform(tmp_tf_stamped);
}
