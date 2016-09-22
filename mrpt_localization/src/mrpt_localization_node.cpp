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

#include <boost/interprocess/sync/scoped_lock.hpp>
#include <geometry_msgs/PoseArray.h>

#include <mrpt_bridge/pose.h>
#include <mrpt_bridge/laser_scan.h>
#include <mrpt_bridge/time.h>
#include <mrpt_bridge/map.h>
#include <mrpt_bridge/beacon.h>

#include <mrpt/version.h>
#if MRPT_VERSION>=0x130
  #include <mrpt/obs/CObservationBeaconRanges.h>
  using namespace mrpt::obs;
#else
  #include <mrpt/slam/CObservationBeaconRanges.h>
  using namespace mrpt::slam;
#endif

#include "mrpt_localization_node.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "localization");
  ros::NodeHandle n;
  PFLocalizationNode my_node(n);
  my_node.init();
  my_node.loop();
  return 0;
}

PFLocalizationNode::~PFLocalizationNode()
{
}

PFLocalizationNode::PFLocalizationNode(ros::NodeHandle &n) :
    PFLocalization(new PFLocalizationNode::Parameters(this)), n_(n), loop_count_(0), update_filter_(true)
{
}

PFLocalizationNode::Parameters *PFLocalizationNode::param()
{
  return (PFLocalizationNode::Parameters*)param_;
}

void PFLocalizationNode::init()
{
  // Use MRPT library the same log level as on ROS nodes (only for MRPT_VERSION >= 0x150)
  useROSLogLevel();

  PFLocalization::init();
  subInitPose_ = n_.subscribe("initialpose", 1, &PFLocalizationNode::callbackInitialpose, this);

  subOdometry_ = n_.subscribe("odom", 1, &PFLocalizationNode::callbackOdometry, this);

  // Subscribe to one or more laser sources:
  std::vector<std::string> lstSources;
  mrpt::system::tokenize(param()->sensorSources, " ,\t\n", lstSources);
  ROS_ASSERT_MSG(
      !lstSources.empty(),
      "*Fatal*: At least one sensor source must be provided in ~sensor_sources (e.g. \"scan\" or \"beacon\")");
  subSensors_.resize(lstSources.size());
  for (size_t i = 0; i < lstSources.size(); i++)
  {
    if (lstSources[i].find("scan") != std::string::npos)
    {
      subSensors_[i] = n_.subscribe(lstSources[i], 1, &PFLocalizationNode::callbackLaser, this);
    }
    else
    {
      subSensors_[i] = n_.subscribe(lstSources[i], 1, &PFLocalizationNode::callbackBeacon, this);
    }
  }

  if (!param()->mapFile.empty())
  {
    if (metric_map_.m_gridMaps.size())
    {
      mrpt_bridge::convert(*metric_map_.m_gridMaps[0], resp_.map);
    }
    pub_map_ = n_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    pub_metadata_ = n_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
    service_map_ = n_.advertiseService("static_map", &PFLocalizationNode::mapCallback, this);
  }
  pub_Particles_ = n_.advertise<geometry_msgs::PoseArray>("particlecloud", 1, true);

  pub_pose_ = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>("mrpt_pose", 2, true);
}

void PFLocalizationNode::loop()
{
  ROS_INFO("loop");
  for (ros::Rate rate(param()->rate); ros::ok(); loop_count_++)
  {
    param()->update(loop_count_);
    if ((loop_count_ % param()->map_update_skip == 0) && (metric_map_.m_gridMaps.size()))
      publishMap();
    if (loop_count_ % param()->particlecloud_update_skip == 0)
      publishParticles();
    if (param()->tf_broadcast) // tf always needed
      publishTF();
    if (param()->pose_broadcast && update_filter_)
      publishPose();
    ros::spinOnce();
    rate.sleep();
  }
}

bool PFLocalizationNode::waitForTransform(mrpt::poses::CPose3D &des, const std::string& target_frame,
                                          const std::string& source_frame, const ros::Time& time,
                                          const ros::Duration& timeout, const ros::Duration& polling_sleep_duration)
{
  tf::StampedTransform transform;
  try
  {
    listenerTF_.waitForTransform(target_frame, source_frame, time, polling_sleep_duration);
    listenerTF_.lookupTransform(target_frame, source_frame, time, transform);
  }
  catch (tf::TransformException& e)
  {
    ROS_WARN("Failed to get transform target_frame (%s) to source_frame (%s): %s",
             target_frame.c_str(), source_frame.c_str(), e.what());
    return false;
  }
  mrpt_bridge::convert(transform, des);
  return true;
}

void PFLocalizationNode::callbackLaser(const sensor_msgs::LaserScan &_msg)
{
#if MRPT_VERSION>=0x130
  using namespace mrpt::maps;
  using namespace mrpt::obs;
#else
  using namespace mrpt::slam;
#endif

  if (!update_filter_)
    return;             // not updating filter; we must be stopped

  //ROS_INFO("callbackLaser");
  CObservation2DRangeScanPtr laser = CObservation2DRangeScan::Create();

  //printf("callbackLaser %s\n", _msg.header.frame_id.c_str());
  if (laser_poses_.find(_msg.header.frame_id) == laser_poses_.end())
  {
    updateSensorPose(_msg.header.frame_id);
  }
  else
  {
    //mrpt::poses::CPose3D pose = laser_poses_[_msg.header.frame_id];
    //ROS_INFO("LASER POSE %4.3f, %4.3f, %4.3f, %4.3f, %4.3f, %4.3f",  pose.x(), pose.y(), pose.z(), pose.roll(), pose.pitch(), pose.yaw());
    mrpt_bridge::convert(_msg, laser_poses_[_msg.header.frame_id], *laser);

    CSensoryFramePtr sf = CSensoryFrame::Create();
    CObservationOdometryPtr odometry;
    odometryForCallback(odometry, _msg.header);

    CObservationPtr obs = CObservationPtr(laser);
    sf->insert(obs);
    observation(sf, odometry);
    if (param()->gui_mrpt)
      show3DDebug(sf);
  }
}

void PFLocalizationNode::callbackBeacon(const mrpt_msgs::ObservationRangeBeacon &_msg)
{
#if MRPT_VERSION>=0x130
  using namespace mrpt::maps;
  using namespace mrpt::obs;
#else
  using namespace mrpt::slam;
#endif

  //ROS_INFO("callbackBeacon");
  CObservationBeaconRangesPtr beacon = CObservationBeaconRanges::Create();
  //printf("callbackBeacon %s\n", _msg.header.frame_id.c_str());
  if (beacon_poses_.find(_msg.header.frame_id) == beacon_poses_.end())
  {
    updateSensorPose(_msg.header.frame_id);
  }
  else
  {
    //mrpt::poses::CPose3D pose = beacon_poses_[_msg.header.frame_id];
    //ROS_INFO("BEACON POSE %4.3f, %4.3f, %4.3f, %4.3f, %4.3f, %4.3f",  pose.x(), pose.y(), pose.z(), pose.roll(), pose.pitch(), pose.yaw());
    mrpt_bridge::convert(_msg, beacon_poses_[_msg.header.frame_id], *beacon);

    CSensoryFramePtr sf = CSensoryFrame::Create();
    CObservationOdometryPtr odometry;
    odometryForCallback(odometry, _msg.header);

    CObservationPtr obs = CObservationPtr(beacon);
    sf->insert(obs);
    observation(sf, odometry);
    if (param()->gui_mrpt)
      show3DDebug(sf);
  }
}

void PFLocalizationNode::odometryForCallback(CObservationOdometryPtr &_odometry, const std_msgs::Header &_msg_header)
{
  std::string base_frame_id = tf::resolve(param()->tf_prefix, param()->base_frame_id);
  std::string odom_frame_id = tf::resolve(param()->tf_prefix, param()->odom_frame_id);
  mrpt::poses::CPose3D poseOdom;
  if (this->waitForTransform(poseOdom, odom_frame_id, base_frame_id, _msg_header.stamp, ros::Duration(1)))
  {
    _odometry = CObservationOdometry::Create();
    _odometry->sensorLabel = odom_frame_id;
    _odometry->hasEncodersInfo = false;
    _odometry->hasVelocities = false;
    _odometry->odometry.x() = poseOdom.x();
    _odometry->odometry.y() = poseOdom.y();
    _odometry->odometry.phi() = poseOdom.yaw();
  }
}

bool PFLocalizationNode::waitForMap()
{
  int wait_counter = 0;
  int wait_limit = 1;
  clientMap_ = n_.serviceClient<nav_msgs::GetMap>("static_map");
  nav_msgs::GetMap srv;
  while (!clientMap_.call(srv) && ros::ok() && wait_counter < wait_limit)
  {
    ROS_INFO("waiting for map service!");
    sleep(1);
    wait_counter++;
  }
  if (wait_counter != wait_limit)
  {
    ROS_INFO_STREAM("Map service complete.");
    updateMap(srv.response.map);
    clientMap_.shutdown();
    return true;
  }

  ROS_WARN_STREAM("No map received.");
  clientMap_.shutdown();
  return false;
}

void PFLocalizationNode::updateSensorPose(std::string _frame_id)
{
  mrpt::poses::CPose3D pose;
  tf::StampedTransform transform;
  try
  {
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
    for (int c = 0; c < 3; c++)
      for (int r = 0; r < 3; r++)
        Rdes(r, c) = Rsrc.getRow(r)[c];
    pose.setRotationMatrix(Rdes);
    laser_poses_[_frame_id] = pose;
    beacon_poses_[_frame_id] = pose;
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

void PFLocalizationNode::callbackInitialpose(const geometry_msgs::PoseWithCovarianceStamped& _msg)
{
  log_info("callbackInitialpose");
  const geometry_msgs::PoseWithCovariance &pose = _msg.pose;
  mrpt_bridge::convert(pose, initialPose_);
  state_ = INIT;
}

void PFLocalizationNode::callbackOdometry(const nav_msgs::Odometry& _msg)
{
  if (_msg.twist.twist.linear.x != 0.0 || _msg.twist.twist.linear.y != 0.0 || _msg.twist.twist.linear.z != 0.0 ||
      _msg.twist.twist.angular.x != 0.0 || _msg.twist.twist.angular.y != 0.0 || _msg.twist.twist.angular.z != 0.0)
    // only update filter if we are moving or for the first iterations after startup
    update_filter_ = true;
  else if (update_counter_ > 100)
    update_filter_ = false;
}

void PFLocalizationNode::updateMap(const nav_msgs::OccupancyGrid &_msg)
{
  ASSERT_(metric_map_.m_gridMaps.size()==1);
  mrpt_bridge::convert(_msg, *metric_map_.m_gridMaps[0]);
}

bool PFLocalizationNode::mapCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
{
  ROS_INFO("mapCallback: service requested!\n");
  res = resp_;
  return true;
}

void PFLocalizationNode::publishMap()
{
  resp_.map.header.stamp = ros::Time::now();
  resp_.map.header.frame_id = tf::resolve(param()->tf_prefix, param()->global_frame_id);
  resp_.map.header.seq = loop_count_;
  if (pub_map_.getNumSubscribers() > 0)
  {
    pub_map_.publish(resp_.map);
  }
  if (pub_metadata_.getNumSubscribers() > 0)
  {
    pub_metadata_.publish(resp_.map.info);
  }
}

void PFLocalizationNode::publishParticles()
{
  if (pub_Particles_.getNumSubscribers() > 0)
  {
    geometry_msgs::PoseArray poseArray;
    poseArray.header.frame_id = tf::resolve(param()->tf_prefix, param()->global_frame_id);
    poseArray.header.stamp = ros::Time::now();
    poseArray.header.seq = loop_count_;
    poseArray.poses.resize(pdf_.particlesCount());
    for (size_t i = 0; i < pdf_.particlesCount(); i++)
    {
      mrpt::poses::CPose2D p = pdf_.getParticlePose(i);
      mrpt_bridge::convert(p, poseArray.poses[i]);
    }
    mrpt::poses::CPose2D p;
    pub_Particles_.publish(poseArray);
  }
}

void PFLocalizationNode::publishTF()
{
  // Most of this code was copy and pase from ros::amcl
  // sorry it is realy ugly
  mrpt::poses::CPose2D robotPose;
  pdf_.getMean(robotPose);
  std::string base_frame_id = tf::resolve(param()->tf_prefix, param()->base_frame_id);
  std::string global_frame_id = tf::resolve(param()->tf_prefix, param()->global_frame_id);
  std::string odom_frame_id = tf::resolve(param()->tf_prefix, param()->odom_frame_id);
  tf::Stamped<tf::Pose> odom_to_map;
  tf::Transform tmp_tf;
  ros::Time stamp = ros::Time::now();
  if (update_filter_)
    mrpt_bridge::convert(timeLastUpdate_, stamp);
  mrpt_bridge::convert(robotPose, tmp_tf);

  try
  {
    tf::Stamped<tf::Pose> tmp_tf_stamped(tmp_tf.inverse(), stamp, base_frame_id);
    //ROS_INFO("subtract global_frame (%s) from odom_frame (%s)", global_frame_id.c_str(), odom_frame_id.c_str());
    listenerTF_.transformPose(odom_frame_id, tmp_tf_stamped, odom_to_map);
  }
  catch (tf::TransformException& e)
  {
    ROS_WARN("Failed to subtract global_frame (%s) from odom_frame (%s): %s",
             global_frame_id.c_str(), odom_frame_id.c_str(), e.what());
    return;
  }

  tf::Transform latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                                           tf::Point(odom_to_map.getOrigin()));

  // We want to send a transform that is good up until a
  // tolerance time so that odom can be used
  ros::Time transform_expiration = (stamp + ros::Duration(param()->transform_tolerance));
  tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(), transform_expiration, global_frame_id, odom_frame_id);
  tf_broadcaster_.sendTransform(tmp_tf_stamped);
  //ROS_INFO("%s, %s\n", global_frame_id.c_str(), odom_frame.c_str());
}

/**
 * publishPose()
 * @beief publish the current pose of the robot
 * @param msg  Laser Scan Message
 **/
void PFLocalizationNode::publishPose()
{
  mrpt::math::CMatrixDouble33 cov;  // cov for x, y, phi (meter, meter, radian)
  mrpt::poses::CPose2D mean;

  pdf_.getCovarianceAndMean(cov, mean);

  geometry_msgs::PoseWithCovarianceStamped p;

  // Fill in the header
  mrpt_bridge::convert(timeLastUpdate_, p.header.stamp);
  p.header.frame_id = tf::resolve(param()->tf_prefix, param()->global_frame_id);
  if (loop_count_ == 0)
    p.header.stamp = ros::Time::now();  // on first iteration timestamp is nonsense

  // Copy in the pose
  mrpt_bridge::convert(mean, p.pose.pose);

  // Copy in the covariance, converting from 3-D to 6-D
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      int ros_i = i;
      int ros_j = j;
      if (i == 2 || j == 2)
      {
        ros_i = i == 2 ? 5 : i;
        ros_j = j == 2 ? 5 : j;
      }
      p.pose.covariance[ros_i * 6 + ros_j] = cov(i, j);
    }
  }

  pub_pose_.publish(p);
}

void PFLocalizationNode::useROSLogLevel()
{
#if MRPT_VERSION>=0x150
  // Set ROS log level also on MRPT internal log system; level enums are fully compatible
  std::map<std::string, ros::console::levels::Level> loggers;
  ros::console::get_loggers(loggers);
  if (loggers.find("ros.roscpp") != loggers.end())
  pdf_.setVerbosityLevel(static_cast<mrpt::utils::VerbosityLevel>(loggers["ros.roscpp"]));
  if (loggers.find("ros.mrpt_localization") != loggers.end())
  pdf_.setVerbosityLevel(static_cast<mrpt::utils::VerbosityLevel>(loggers["ros.mrpt_localization"]));
#endif
}
