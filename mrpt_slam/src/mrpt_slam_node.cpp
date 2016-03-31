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

#include "mrpt_slam_node.h"
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <geometry_msgs/PoseArray.h>

#include <mrpt_bridge/pose.h>
#include <mrpt_bridge/laser_scan.h>
#include <mrpt_bridge/time.h>
#include <mrpt_bridge/map.h>
#include <mrpt_bridge/beacon.h>
#include <std_msgs/Header.h>


#include <mrpt/version.h>
#if MRPT_VERSION>=0x130
#	include <mrpt/obs/CObservationBeaconRanges.h>
	using namespace mrpt::obs;
#else
#	include <mrpt/slam/CObservationBeaconRanges.h>
	using namespace mrpt::slam;
#endif

int main(int argc, char **argv) {
    ros::init(argc, argv, "slam");
    ros::NodeHandle n;
    RBPFSlamNode my_node(n);
    my_node.init();
    my_node.loop();
	my_node.finale();
    return 0;
}

RBPFSlamNode::~RBPFSlamNode() {
}

RBPFSlamNode::RBPFSlamNode(ros::NodeHandle &n) :
    RBPFSlam(new RBPFSlamNode::Parameters(this)), n_(n), loop_count_(0) {
}

RBPFSlamNode::Parameters *RBPFSlamNode::param() {
    return (RBPFSlamNode::Parameters*) param_;
}

void RBPFSlamNode::init() {
    RBPFSlam::init();
	
    subInitPose_ = n_.subscribe("initialpose", 1, &RBPFSlamNode::callbackInitialpose, this);
	// Subscribe to one or more laser sources:
	std::vector<std::string> lstSources;
	mrpt::system::tokenize(param()->sensorSources," ,\t\n",lstSources);
	ROS_ASSERT_MSG(!lstSources.empty(), "*Fatal*: At least one sensor source must be provided in ~sensor_sources (e.g. \"scan\" or \"beacon\")");
	subSensors_.resize(lstSources.size());
	for (size_t i=0;i<lstSources.size();i++) {
		if(lstSources[i].find("scan") != std::string::npos) {
			ROS_ERROR_STREAM("Lidar scan readings not possible at this time");	
			subSensors_[i]  = n_.subscribe(lstSources[i],  1, &RBPFSlamNode::callbackLaser, this);
		}
		else {
        	subSensors_[i]  = n_.subscribe(lstSources[i],  1, &RBPFSlamNode::callbackBeacon, this);
		}
	}

	pub_map_ = n_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
	pub_metadata_= n_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
	service_map_ = n_.advertiseService("static_map", &RBPFSlamNode::mapCallback, this);
	pub_Particles_Beacons_ = n_.advertise<geometry_msgs::PoseArray>("particlecloud_beacons", 1, true);	
	pub_Particles_Robot_ = n_.advertise<geometry_msgs::PoseArray>("particlecloud_robot", 1, true);

}

void RBPFSlamNode::loop() {
    ROS_INFO("loop");
    for (ros::Rate rate(param()->rate); ros::ok(); loop_count_++) {
        param()->update(loop_count_);
        if((loop_count_%param()->map_update_skip == 0)) 				publishMap();
        if(loop_count_%param()->particlecloud_update_skip == 0)   publishParticles();
        publishTF();
        ros::spinOnce();
        rate.sleep();
    }
}

void RBPFSlamNode::finale() {
	gasObservations.clear();
	wifiObservations.clear();
}

bool RBPFSlamNode::waitForTransform(mrpt::poses::CPose3D &des, const std::string& target_frame, const std::string& source_frame, const ros::Time& time, const ros::Duration& timeout, const ros::Duration& polling_sleep_duration){
    tf::StampedTransform transform;
    try
    {
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


void RBPFSlamNode::callbackLaser (const sensor_msgs::LaserScan &_msg) {
#if MRPT_VERSION>=0x130
	using namespace mrpt::maps;
	using namespace mrpt::obs;
#else
	using namespace mrpt::slam;
#endif
    //ROS_INFO("callbackLaser");
	CObservation2DRangeScanPtr laser = CObservation2DRangeScan::Create();

    //printf("callbackLaser %s\n", _msg.header.frame_id.c_str());
    if(laser_poses_.find(_msg.header.frame_id) == laser_poses_.end()) {
        updateSensorPose (_msg.header.frame_id);
    } else {
        //mrpt::poses::CPose3D pose = laser_poses_[_msg.header.frame_id];
        //ROS_INFO("LASER POSE %4.3f, %4.3f, %4.3f, %4.3f, %4.3f, %4.3f",  pose.x(), pose.y(), pose.z(), pose.roll(), pose.pitch(), pose.yaw());
        mrpt_bridge::convert(_msg, laser_poses_[_msg.header.frame_id],  *laser);

		sf = CSensoryFrame::Create();
		CObservationOdometryPtr odometry;
		odometryForCallback(odometry, _msg.header);
	
		t_exec = tictac_.Tac();
		CObservationPtr obs = CObservationPtr(laser);
    	sf->insert(obs);
    	observation(sf, odometry);
		mapBuilderRBPF.processActionObservation( *action, *sf );
    	if(param()->gui_mrpt) show3DDebug(sf);
		action.clear_unique();
		sf.clear_unique();
    }
}

void RBPFSlamNode::callbackBeacon (const mrpt_msgs::ObservationRangeBeacon &_msg) {
#if MRPT_VERSION>=0x130
	using namespace mrpt::maps;
	using namespace mrpt::obs;
#else
	using namespace mrpt::slam;
#endif
    //ROS_INFO("callbackBeacon");
	CObservationBeaconRangesPtr beacon = CObservationBeaconRanges::Create();
    //printf("callbackBeacon %s\n", _msg.header.frame_id.c_str());
    if(beacon_poses_.find(_msg.header.frame_id) == beacon_poses_.end()) {
		updateSensorPose (_msg.header.frame_id);
    } else {
        //mrpt::poses::CPose3D pose = beacon_poses_[_msg.header.frame_id];
        //ROS_INFO("BEACON POSE %4.3f, %4.3f, %4.3f, %4.3f, %4.3f, %4.3f",  pose.x(), pose.y(), pose.z(), pose.roll(), pose.pitch(), pose.yaw());
        mrpt_bridge::convert(_msg, beacon_poses_[_msg.header.frame_id],  *beacon);
	
		sf = CSensoryFrame::Create();
		CObservationOdometryPtr odometry;
		odometryForCallback(odometry, _msg.header);
	
		t_exec = tictac_.Tac();
		CObservationPtr obs = CObservationPtr(beacon);
    	sf->insert(obs);
    	observation(sf, odometry);
		mapBuilderRBPF.processActionObservation( *action, *sf );
    	if(param()->gui_mrpt) show3DDebug(sf);
		action.clear_unique();
		sf.clear_unique();
    }
}

void RBPFSlamNode::odometryForCallback (CObservationOdometryPtr  &_odometry, const std_msgs::Header &_msg_header) {
    std::string base_frame_id = tf::resolve(param()->tf_prefix, param()->base_frame_id);
    std::string odom_frame_id = tf::resolve(param()->tf_prefix, param()->odom_frame_id);
    mrpt::poses::CPose3D poseOdom;
    if(this->waitForTransform(poseOdom, odom_frame_id, base_frame_id, _msg_header.stamp, ros::Duration(1))){
		_odometry = CObservationOdometry::Create();
        _odometry->sensorLabel = odom_frame_id;
        _odometry->hasEncodersInfo = false;
        _odometry->hasVelocities = false;
        _odometry->odometry.x() = poseOdom.x();
        _odometry->odometry.y() = poseOdom.y();
        _odometry->odometry.phi() = poseOdom.yaw();
    }
}

void RBPFSlamNode::updateSensorPose (std::string _frame_id) {
    mrpt::poses::CPose3D pose;
    tf::StampedTransform transform;
    try {
        std::string base_frame_id = tf::resolve(param()->tf_prefix, param()->base_frame_id);
        listenerTF_.lookupTransform(base_frame_id, _frame_id, ros::Time(0), transform);
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
        beacon_poses_[_frame_id] = pose;
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

}

void RBPFSlamNode::callbackInitialpose (const geometry_msgs::PoseWithCovarianceStamped& _msg) {
    log_info("callbackInitialpose");
    const geometry_msgs::PoseWithCovariance &pose = _msg.pose;
    mrpt_bridge::convert(pose, initialPose_);
    state_ = INIT;
}

bool RBPFSlamNode::mapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res )
{
    ROS_INFO("mapCallback: service requested!\n");
    res = resp_;
    return true;
}

void RBPFSlamNode::publishMap () {
    resp_.map.header.stamp = ros::Time::now();
    resp_.map.header.frame_id =  tf::resolve(param()->tf_prefix, param()->global_frame_id);
    resp_.map.header.seq = loop_count_;
    if(pub_map_.getNumSubscribers() > 0) {
        pub_map_.publish(resp_.map );
    }
    if(pub_metadata_.getNumSubscribers() > 0) {
        pub_metadata_.publish( resp_.map.info );
    }
}

void RBPFSlamNode::publishParticles ()
{
	metric_map_ = mapBuilderRBPF.mapPDF.getCurrentMostLikelyMetricMap();
	objs = mrpt::opengl::CSetOfObjects::Create();
	metric_map_->getAs3DObject( objs );
	
	//publishing the beacon particles
	if (pub_Particles_Beacons_.getNumSubscribers()>0)
	{	
		geometry_msgs::PoseArray poseArrayBeacons;
		poseArrayBeacons.header.frame_id = tf::resolve(param()->tf_prefix, param()->global_frame_id);
		poseArrayBeacons.header.stamp = ros::Time::now();
		poseArrayBeacons.header.seq = loop_count_;
		
		int objs_counter = 0;
		while (objs->getByClass<mrpt::opengl::CEllipsoid>(objs_counter )) { // couldn't find a size() member for T::SmartPtr getByClass( const size_t &ith = 0 )
			objs_counter++;
		}	
		poseArrayBeacons.poses.resize(objs_counter);
		mrpt::opengl::CEllipsoidPtr beacon_particle;

		for (size_t i = 0; i < objs_counter; i++) {
			beacon_particle = objs->getByClass<mrpt::opengl::CEllipsoid>(i);	
			mrpt_bridge::convert(beacon_particle->getPose(), poseArrayBeacons.poses[i]);		
		}
		pub_Particles_Beacons_.publish(poseArrayBeacons);
	}
	
	//publishing the robot particles
	if (pub_Particles_Robot_.getNumSubscribers()>0)
	{
		geometry_msgs::PoseArray poseArrayRobot;
		poseArrayRobot.header.frame_id = tf::resolve(param()->tf_prefix, param()->global_frame_id);
		poseArrayRobot.header.stamp = ros::Time::now();
		poseArrayRobot.header.seq = loop_count_;
		
		size_t particle_count = mapBuilderRBPF.mapPDF.particlesCount();
		poseArrayRobot.poses.resize(particle_count);
		mapBuilderRBPF.mapPDF.getEstimatedPosePDF(robotPoseEstimation);

		for (size_t i = 0; i < particle_count; i++) {
			mrpt_bridge::convert(robotPoseEstimation.getParticlePose(i), poseArrayRobot.poses[i]);
		}
		pub_Particles_Robot_.publish(poseArrayRobot);
	}	
}

void RBPFSlamNode::publishTF() {
	// Most of this code was copy and pase from ros::amcl
    // sorry it is realy ugly
	mrpt::poses::CPose3D		robotPose;
	mapBuilderRBPF.mapPDF.getEstimatedPosePDF(robotPoseEstimation);
	robotPoseEstimation.getMean(robotPose);
    std::string base_frame_id = tf::resolve(param()->tf_prefix, param()->base_frame_id);
    std::string global_frame_id = tf::resolve(param()->tf_prefix, param()->global_frame_id);
    std::string odom_frame_id = tf::resolve(param()->tf_prefix, param()->odom_frame_id);
    tf::Stamped<tf::Pose> odom_to_map;
    tf::Transform tmp_tf;
    ros::Time stamp;
    mrpt_bridge::convert(timeLastUpdate_, stamp);
    mrpt_bridge::convert(robotPose, tmp_tf);
    try
    {
        tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(), stamp,  base_frame_id);
		//ROS_INFO("subtract global_frame (%s) from odom_frame (%s)", global_frame_id.c_str(), odom_frame_id.c_str());
        listenerTF_.transformPose(odom_frame_id, tmp_tf_stamped, odom_to_map);
    }
    catch(tf::TransformException)
    {
		ROS_INFO("Failed to subtract global_frame (%s) from odom_frame (%s)", global_frame_id.c_str(), odom_frame_id.c_str());
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
                                        global_frame_id, odom_frame_id);
    tf_broadcaster_.sendTransform(tmp_tf_stamped);
    //ROS_INFO("%s, %s\n", global_frame_id.c_str(), odom_frame.c_str());
}
