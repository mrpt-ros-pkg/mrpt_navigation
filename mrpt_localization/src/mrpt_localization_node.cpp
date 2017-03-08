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
#include <pose_cov_ops/pose_cov_ops.h>

#include <mrpt_bridge/pose.h>
#include <mrpt_bridge/laser_scan.h>
#include <mrpt_bridge/time.h>
#include <mrpt_bridge/map.h>
#include <mrpt_bridge/beacon.h>

#include <mrpt/version.h>
#if MRPT_VERSION>=0x130
  #include <mrpt/obs/CObservationBeaconRanges.h>
  using namespace mrpt::obs;
  #if MRPT_VERSION>=0x150
    #include <mrpt/obs/CObservationRobotPose.h>
  #endif
#else
  #include <mrpt/slam/CObservationBeaconRanges.h>
  using namespace mrpt::slam;
#endif

#include "mrpt_localization_node.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "localization");
  ros::NodeHandle nh;
  PFLocalizationNode my_node(nh);
  my_node.init();
  my_node.loop();
  return 0;
}

PFLocalizationNode::~PFLocalizationNode()
{
}

PFLocalizationNode::PFLocalizationNode(ros::NodeHandle &n) :
    PFLocalization(new PFLocalizationNode::Parameters(this)), nh_(n), loop_count_(0)
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
  sub_init_pose_ = nh_.subscribe("initialpose", 1, &PFLocalizationNode::callbackInitialpose, this);

  sub_odometry_ = nh_.subscribe("odom", 1, &PFLocalizationNode::callbackOdometry, this);

  // Subscribe to one or more laser sources:
  std::vector<std::string> sources;
  mrpt::system::tokenize(param()->sensor_sources, " ,\t\n", sources);
  ROS_ASSERT_MSG(
      !sources.empty(),
      "*Fatal*: At least one sensor source must be provided in ~sensor_sources (e.g. \"scan\" or \"beacon\")");
  sub_sensors_.resize(sources.size());
  for (size_t i = 0; i < sources.size(); i++)
  {
    if (sources[i].find("scan") != std::string::npos)
    {
      sub_sensors_[i] = nh_.subscribe(sources[i], 1, &PFLocalizationNode::callbackLaser, this);
    }
    else if (sources[i].find("beacon") != std::string::npos)
    {
      sub_sensors_[i] = nh_.subscribe(sources[i], 1, &PFLocalizationNode::callbackBeacon, this);
    }
    else
    {
      sub_sensors_[i] = nh_.subscribe(sources[i], 1, &PFLocalizationNode::callbackRobotPose, this);
    }
  }

  if (!param()->map_file.empty())
  {
    if (metric_map_.m_gridMaps.size())
    {
      mrpt_bridge::convert(*metric_map_.m_gridMaps[0], resp_.map);
    }
    pub_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    pub_metadata_ = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
    service_map_ = nh_.advertiseService("static_map", &PFLocalizationNode::mapCallback, this);
  }
  pub_particles_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 1, true);

  pub_pose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("mrpt_pose", 2, true);
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
    if (param()->tf_broadcast)
      publishTF();
    if (param()->pose_broadcast)
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
    tf_listener_.waitForTransform(target_frame, source_frame, time, timeout, polling_sleep_duration);
    tf_listener_.lookupTransform(target_frame, source_frame, time, transform);
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

  time_last_input_ = ros::Time::now();

  //ROS_INFO("callbackLaser");
  CObservation2DRangeScanPtr laser = CObservation2DRangeScan::Create();

  //printf("callbackLaser %s\n", _msg.header.frame_id.c_str());
  if (laser_poses_.find(_msg.header.frame_id) == laser_poses_.end())
  {
    updateSensorPose(_msg.header.frame_id);
  }
  else if (state_ != IDLE) // updating filter; we must be moving or update_while_stopped set to true
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

  time_last_input_ = ros::Time::now();

  //ROS_INFO("callbackBeacon");
  CObservationBeaconRangesPtr beacon = CObservationBeaconRanges::Create();
  //printf("callbackBeacon %s\n", _msg.header.frame_id.c_str());
  if (beacon_poses_.find(_msg.header.frame_id) == beacon_poses_.end())
  {
    updateSensorPose(_msg.header.frame_id);
  }
  else if (state_ != IDLE) // updating filter; we must be moving or update_while_stopped set to true
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

void PFLocalizationNode::callbackRobotPose(const geometry_msgs::PoseWithCovarianceStamped &_msg)
{
#if MRPT_VERSION>=0x150
  using namespace mrpt::maps;
  using namespace mrpt::obs;

  time_last_input_ = ros::Time::now();

  // Robot pose externally provided; we update filter regardless state_ attribute's value, as these
  // corrections are typically independent from robot motion (e.g. inputs from GPS or tracking system)
  // XXX admittedly an arbitrary choice; feel free to open an issue if you think it doesn't make sense

  static std::string base_frame_id = tf::resolve(param()->tf_prefix, param()->base_frame_id);
  static std::string global_frame_id = tf::resolve(param()->tf_prefix, param()->global_frame_id);

  tf::StampedTransform map_to_obs_tf;
  try
  {
    tf_listener_.waitForTransform(global_frame_id, _msg.header.frame_id, ros::Time(0.0), ros::Duration(0.5));
    tf_listener_.lookupTransform(global_frame_id, _msg.header.frame_id, ros::Time(0.0), map_to_obs_tf);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // Transform observation into global frame, including covariance. For that, we must first obtain
  // the global frame -> observation frame tf as a Pose msg, as required by pose_cov_ops::compose
  geometry_msgs::Pose map_to_obs_pose;
  tf::pointTFToMsg(map_to_obs_tf.getOrigin(), map_to_obs_pose.position);
  tf::quaternionTFToMsg(map_to_obs_tf.getRotation(), map_to_obs_pose.orientation);
  geometry_msgs::PoseWithCovarianceStamped obs_pose_world;
  obs_pose_world.header.stamp = _msg.header.stamp;
  obs_pose_world.header.frame_id = global_frame_id;
  pose_cov_ops::compose(map_to_obs_pose, _msg.pose, obs_pose_world.pose);

  // Ensure the covariance matrix can be inverted (no zeros in the diagonal)
  for (int i = 0; i < obs_pose_world.pose.covariance.size(); ++i)
  {
    if (i/6 == i%6 && obs_pose_world.pose.covariance[i] <= 0.0)
      obs_pose_world.pose.covariance[i] = std::numeric_limits<double>().infinity();
  }

  // Covert the received pose into an observation the filter can integrate
  CObservationRobotPosePtr feature = CObservationRobotPose::Create();

  feature->sensorLabel = _msg.header.frame_id;
  mrpt_bridge::convert(_msg.header.stamp, feature->timestamp);
  mrpt_bridge::convert(obs_pose_world.pose, feature->pose);

  CSensoryFramePtr sf = CSensoryFrame::Create();
  CObservationOdometryPtr odometry;
  odometryForCallback(odometry, _msg.header);

  CObservationPtr obs = CObservationPtr(feature);
  sf->insert(obs);
  observation(sf, odometry);
  if (param()->gui_mrpt)
    show3DDebug(sf);
#else
  // not implemented
#endif
}

void PFLocalizationNode::odometryForCallback(CObservationOdometryPtr &_odometry, const std_msgs::Header &_msg_header)
{
  std::string base_frame_id = tf::resolve(param()->tf_prefix, param()->base_frame_id);
  std::string odom_frame_id = tf::resolve(param()->tf_prefix, param()->odom_frame_id);
  mrpt::poses::CPose3D poseOdom;
  if (this->waitForTransform(poseOdom, odom_frame_id, base_frame_id, _msg_header.stamp, ros::Duration(1.0)))
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
  client_map_ = nh_.serviceClient<nav_msgs::GetMap>("static_map");
  nav_msgs::GetMap srv;
  while (!client_map_.call(srv) && ros::ok() && wait_counter < wait_limit)
  {
    ROS_INFO("waiting for map service!");
    sleep(1);
    wait_counter++;
  }
  if (wait_counter != wait_limit)
  {
    ROS_INFO_STREAM("Map service complete.");
    updateMap(srv.response.map);
    client_map_.shutdown();
    return true;
  }

  ROS_WARN_STREAM("No map received.");
  client_map_.shutdown();
  return false;
}

void PFLocalizationNode::updateSensorPose(std::string _frame_id)
{
  mrpt::poses::CPose3D pose;
  tf::StampedTransform transform;
  try
  {
    std::string base_frame_id = tf::resolve(param()->tf_prefix, param()->base_frame_id);
    tf_listener_.lookupTransform(base_frame_id, _frame_id, ros::Time(0), transform);
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
  const geometry_msgs::PoseWithCovariance &pose = _msg.pose;
  mrpt_bridge::convert(pose, initial_pose_);
  update_counter_ = 0;
  state_ = INIT;
}

void PFLocalizationNode::callbackOdometry(const nav_msgs::Odometry& _msg)
{
  // We always update the filter if update_while_stopped is true, regardless robot is moving or
  // not; otherwise, update filter if we are moving or at initialization (100 first iterations)
  bool moving = std::abs(_msg.twist.twist.linear.x) > 1e-3 ||
                std::abs(_msg.twist.twist.linear.y) > 1e-3 ||
                std::abs(_msg.twist.twist.linear.z) > 1e-3 ||
                std::abs(_msg.twist.twist.angular.x) > 1e-3 ||
                std::abs(_msg.twist.twist.angular.y) > 1e-3 ||
                std::abs(_msg.twist.twist.angular.z) > 1e-3;
  if (param()->update_while_stopped || moving)
  {
    if (state_ == IDLE)
    {
      state_ = RUN;
    }
  }
  else if (state_ == RUN && update_counter_ >= 100)
  {
    state_ = IDLE;
  }
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
  if (pub_particles_.getNumSubscribers() > 0)
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
    pub_particles_.publish(poseArray);
  }
}

/**
 * @brief Publish map -> odom tf; as the filter provides map -> base, we multiply it by base -> odom
 */
void PFLocalizationNode::publishTF()
{
  static std::string base_frame_id = tf::resolve(param()->tf_prefix, param()->base_frame_id);
  static std::string odom_frame_id = tf::resolve(param()->tf_prefix, param()->odom_frame_id);
  static std::string global_frame_id = tf::resolve(param()->tf_prefix, param()->global_frame_id);

  mrpt::poses::CPose2D robot_pose;
  pdf_.getMean(robot_pose);
  tf::StampedTransform base_on_map_tf, odom_on_base_tf;
  mrpt_bridge::convert(robot_pose, base_on_map_tf);
  ros::Time time_last_update(0.0);
  if (state_ == RUN)
  {
    mrpt_bridge::convert(time_last_update_, time_last_update);

    // Last update time can be too far in the past if we where not updating filter, due to robot stopped or no
    // observations for a while (we optionally show a warning in the second case)
    // We use time zero if so when getting base -> odom tf to prevent an extrapolation into the past exception
    if ((ros::Time::now() - time_last_update).toSec() > param()->no_update_tolerance)
    {
      if ((ros::Time::now() - time_last_input_).toSec() > param()->no_inputs_tolerance)
      {
        ROS_WARN_THROTTLE(2.0, "No observations received for %.2fs (tolerance %.2fs); are robot sensors working?",
                          (ros::Time::now() - time_last_input_).toSec(), param()->no_inputs_tolerance);
      }
      else
      {
        ROS_DEBUG_THROTTLE(2.0, "No filter updates for %.2fs (tolerance %.2fs); probably robot stopped for a while",
                           (ros::Time::now() - time_last_update).toSec(), param()->no_update_tolerance);
      }

      time_last_update = ros::Time(0.0);
    }
  }

  try
  {
    // Get base -> odom transform
    tf_listener_.waitForTransform(base_frame_id, odom_frame_id, time_last_update, ros::Duration(0.1));
    tf_listener_.lookupTransform(base_frame_id, odom_frame_id, time_last_update, odom_on_base_tf);
  }
  catch (tf::TransformException& e)
  {
    ROS_WARN_THROTTLE(2.0, "Transform from base frame (%s) to odom frame (%s) failed: %s",
                      base_frame_id.c_str(), odom_frame_id.c_str(), e.what());
    ROS_WARN_THROTTLE(2.0, "Ensure that your mobile base driver is broadcasting %s -> %s tf",
                      odom_frame_id.c_str(), base_frame_id.c_str());
    return;
  }

  // We want to send a transform that is good up until a tolerance time so that odom can be used
  ros::Time transform_expiration =
      (time_last_update.isZero() ? ros::Time::now() : time_last_update)
      + ros::Duration(param()->transform_tolerance);
  tf::StampedTransform tmp_tf_stamped(base_on_map_tf * odom_on_base_tf, transform_expiration,
                                      global_frame_id, odom_frame_id);
  tf_broadcaster_.sendTransform(tmp_tf_stamped);
}

/**
 * @brief Publish the current pose of the robot
 **/
void PFLocalizationNode::publishPose()
{
  mrpt::math::CMatrixDouble33 cov;  // cov for x, y, phi (meter, meter, radian)
  mrpt::poses::CPose2D mean;

  pdf_.getCovarianceAndMean(cov, mean);

  geometry_msgs::PoseWithCovarianceStamped p;

  // Fill in the header
  p.header.frame_id = tf::resolve(param()->tf_prefix, param()->global_frame_id);
  if (loop_count_ < 10 || state_ == IDLE)
    p.header.stamp = ros::Time::now();  // on first iterations timestamp differs a lot from ROS time
  else
    mrpt_bridge::convert(time_last_update_, p.header.stamp);

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
