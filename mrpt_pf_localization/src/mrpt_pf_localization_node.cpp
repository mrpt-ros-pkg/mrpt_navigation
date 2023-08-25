/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2023, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include "mrpt_pf_localization_node.h"

#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CObservationRobotPose.h>
#include <mrpt/ros2bridge/laser_scan.h>
#include <mrpt/ros2bridge/map.h>
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/time.h>
#include <mrpt/system/COutputLogger.h>
#include <pose_cov_ops/pose_cov_ops.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <mrpt_msgs_bridge/beacon.hpp>

using namespace mrpt::obs;
using namespace mrpt::system;

using mrpt::maps::COccupancyGridMap2D;

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<PFLocalizationNode>();

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;
}

PFLocalizationNode::~PFLocalizationNode() {}
PFLocalizationNode::PFLocalizationNode()
	: first_map_received_(false), loop_count_(0)
{
}

void PFLocalizationNode::init()
{
	// Use MRPT library the same log level as on ROS nodes (only for
	// MRPT_VERSION >= 0x150)
	useROSLogLevel();

	PFLocalization::init();

	sub_init_pose_ = nh_.subscribe(
		"initialpose", 1, &PFLocalizationNode::callbackInitialpose, this);

	sub_odometry_ =
		nh_.subscribe("odom", 1, &PFLocalizationNode::callbackOdometry, this);

	// Subscribe to one or more laser sources:
	std::vector<std::string> sources;
	mrpt::system::tokenize(param()->sensor_sources, " ,\t\n", sources);
	ROS_ASSERT_MSG(
		!sources.empty(),
		"*Fatal*: At least one sensor source must be provided in "
		"~sensor_sources (e.g. \"scan\" or \"beacon\")");
	sub_sensors_.resize(sources.size());
	for (size_t i = 0; i < sources.size(); i++)
	{
		if (sources[i].find("scan") != std::string::npos)
		{
			sub_sensors_[i] = nh_.subscribe(
				sources[i], 1, &PFLocalizationNode::callbackLaser, this);
		}
		else if (sources[i].find("beacon") != std::string::npos)
		{
			sub_sensors_[i] = nh_.subscribe(
				sources[i], 1, &PFLocalizationNode::callbackBeacon, this);
		}
		else
		{
			sub_sensors_[i] = nh_.subscribe(
				sources[i], 1, &PFLocalizationNode::callbackRobotPose, this);
		}
	}

	if (!param()->map_file.empty())
	{
		if (metric_map_->countMapsByClass<COccupancyGridMap2D>())
		{
			mrpt::ros2bridge::toROS(
				*metric_map_->mapByClass<COccupancyGridMap2D>(), resp_.map);
		}
		pub_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
		pub_metadata_ =
			nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
		service_map_ = nh_.advertiseService(
			"static_map", &PFLocalizationNode::mapCallback, this);
	}
	pub_particles_ =
		nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 1, true);

	pub_pose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
		"mrpt_pose", 2, true);
}

void PFLocalizationNode::loop()
{
	MRPT_LOG_INFO_FMT("loop");
	for (ros::Rate rate(param()->rate); ros::ok(); loop_count_++)
	{
		param()->update(loop_count_);

		if ((loop_count_ % param()->map_update_skip == 0) &&
			(metric_map_->countMapsByClass<COccupancyGridMap2D>()))
			publishMap();
		if (loop_count_ % param()->particlecloud_update_skip == 0)
			publishParticles();
		if (param()->tf_broadcast) publishTF();
		if (param()->pose_broadcast) publishPose();

		ros::spinOnce();
		rate.sleep();
	}
}

bool PFLocalizationNode::waitForTransform(
	mrpt::poses::CPose3D& des, const std::string& target_frame,
	const std::string& source_frame, const ros::Time& time,
	const int& timeoutMilliseconds, const ros::Duration& polling_sleep_duration)
{
	rclcpp::Duration timeout(std::chrono::milliseconds(timeoutMilliseconds));
	try
	{
		geometry_msgs::msg::TransformStamped ref_to_trgFrame =
			tf_buffer_->lookupTransform(
				target_frame, source_frame, tf2::TimePointZero,
				tf2::durationFromSec(timeout.seconds()));

		des = mrpt::ros2bridge::fromROS(ref_to_trgFrame.transform);
		return true;
	}
	catch (const tf2::TransformException& ex)
	{
		RCLCPP_ERROR(nh_->get_logger(), "%s", ex.what());
		return false;
	}
}

void PFLocalizationNode::callbackLaser(const sensor_msgs::msg::LaserScan& _msg)
{
	using namespace mrpt::maps;
	using namespace mrpt::obs;

	time_last_input_ = ros::Time::now();

	// MRPT_LOG_INFO_FMT("callbackLaser");
	auto laser = CObservation2DRangeScan::Create();

	// printf("callbackLaser %s\n", _msg.header.frame_id.c_str());
	if (laser_poses_.find(_msg.header.frame_id) == laser_poses_.end())
	{
		updateSensorPose(_msg.header.frame_id);
	}
	else if (state_ != IDLE)  // updating filter; we must be moving or
	// update_while_stopped set to true
	{
		if (param()->update_sensor_pose)
		{
			updateSensorPose(_msg.header.frame_id);
		}
		// mrpt::poses::CPose3D pose = laser_poses_[_msg.header.frame_id];
		// MRPT_LOG_INFO_FMT("LASER POSE %4.3f, %4.3f, %4.3f, %4.3f, %4.3f,
		// %4.3f", pose.x(), pose.y(), pose.z(), pose.roll(), pose.pitch(),
		// pose.yaw());
		mrpt::ros2bridge::fromROS(
			_msg, laser_poses_[_msg.header.frame_id], *laser);

		auto sf = CSensoryFrame::Create();
		CObservationOdometry::Ptr odometry;
		odometryForCallback(odometry, _msg.header);

		CObservation::Ptr obs = CObservation::Ptr(laser);
		sf->insert(obs);
		observation(sf, odometry);
		if (param()->gui_mrpt) show3DDebug(sf);
	}
}

void PFLocalizationNode::callbackBeacon(
	const mrpt_msgs::msg::ObservationRangeBeacon& _msg)
{
	using namespace mrpt::maps;
	using namespace mrpt::obs;

	time_last_input_ = ros::Time::now();

	// MRPT_LOG_INFO_FMT("callbackBeacon");
	auto beacon = CObservationBeaconRanges::Create();
	// printf("callbackBeacon %s\n", _msg.header.frame_id.c_str());
	if (beacon_poses_.find(_msg.header.frame_id) == beacon_poses_.end())
	{
		updateSensorPose(_msg.header.frame_id);
	}
	else if (state_ != IDLE)  // updating filter; we must be moving or
	// update_while_stopped set to true
	{
		if (param()->update_sensor_pose)
		{
			updateSensorPose(_msg.header.frame_id);
		}
		// mrpt::poses::CPose3D pose = beacon_poses_[_msg.header.frame_id];
		// MRPT_LOG_INFO_FMT("BEACON POSE %4.3f, %4.3f, %4.3f, %4.3f, %4.3f,
		// %4.3f", pose.x(), pose.y(), pose.z(), pose.roll(), pose.pitch(),
		// pose.yaw());
		mrpt_msgs_bridge::fromROS(
			_msg, beacon_poses_[_msg.header.frame_id], *beacon);

		auto sf = CSensoryFrame::Create();
		CObservationOdometry::Ptr odometry;
		odometryForCallback(odometry, _msg.header);

		CObservation::Ptr obs = CObservation::Ptr(beacon);
		sf->insert(obs);
		observation(sf, odometry);
		if (param()->gui_mrpt) show3DDebug(sf);
	}
}

void PFLocalizationNode::callbackRobotPose(
	const geometry_msgs::msg::PoseWithCovarianceStamped& _msg)
{
	using namespace mrpt::maps;
	using namespace mrpt::obs;

	time_last_input_ = ros::Time::now();

	// Robot pose externally provided; we update filter regardless state_
	// attribute's value, as these
	// corrections are typically independent from robot motion (e.g. inputs from
	// GPS or tracking system)
	// XXX admittedly an arbitrary choice; feel free to open an issue if you
	// think it doesn't make sense

	static std::string base_frame_id = param()->base_frame_id;
	static std::string global_frame_id = param()->global_frame_id;

	geometry_msgs::TransformStamped map_to_obs_tf_msg;
	try
	{
		map_to_obs_tf_msg = tf_buffer_.lookupTransform(
			global_frame_id, _msg.header.frame_id, ros::Time(0.0),
			ros::Duration(0.5));
	}
	catch (const tf2::TransformException& e)
	{
		ROS_WARN(
			"Failed to get transform target_frame (%s) to source_frame (%s): "
			"%s",
			global_frame_id.c_str(), _msg.header.frame_id.c_str(), e.what());
		return;
	}
	tf2::Transform map_to_obs_tf;
	tf2::fromMsg(map_to_obs_tf_msg.transform, map_to_obs_tf);

	// Transform observation into global frame, including covariance. For that,
	// we must first obtain
	// the global frame -> observation frame tf as a Pose msg, as required by
	// pose_cov_ops::compose
	geometry_msgs::Pose map_to_obs_pose;
	tf2::toMsg(map_to_obs_tf, map_to_obs_pose);

	geometry_msgs::PoseWithCovarianceStamped obs_pose_world;
	obs_pose_world.header.stamp = _msg.header.stamp;
	obs_pose_world.header.frame_id = global_frame_id;
	pose_cov_ops::compose(map_to_obs_pose, _msg.pose, obs_pose_world.pose);

	// Ensure the covariance matrix can be inverted (no zeros in the diagonal)
	for (unsigned int i = 0; i < obs_pose_world.pose.covariance.size(); ++i)
	{
		if (i / 6 == i % 6 && obs_pose_world.pose.covariance[i] <= 0.0)
			obs_pose_world.pose.covariance[i] =
				std::numeric_limits<double>().infinity();
	}

	// Covert the received pose into an observation the filter can integrate
	auto feature = CObservationRobotPose::Create();

	feature->sensorLabel = _msg.header.frame_id;
	feature->timestamp = mrpt::ros2bridge::fromROS(_msg.header.stamp);
	feature->pose = mrpt::ros2bridge::fromROS(obs_pose_world.pose);

	auto sf = CSensoryFrame::Create();
	CObservationOdometry::Ptr odometry;
	odometryForCallback(odometry, _msg.header);

	CObservation::Ptr obs = CObservation::Ptr(feature);
	sf->insert(obs);
	observation(sf, odometry);
	if (param()->gui_mrpt) show3DDebug(sf);
}

void PFLocalizationNode::odometryForCallback(
	CObservationOdometry::Ptr& _odometry,
	const std_msgs::msg::Header& _msg_header)
{
	std::string base_frame_id = param()->base_frame_id;
	std::string odom_frame_id = param()->odom_frame_id;
	mrpt::poses::CPose3D poseOdom;
	if (this->waitForTransform(
			poseOdom, odom_frame_id, base_frame_id, _msg_header.stamp,
			ros::Duration(1.0)))
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
	int wait_limit = 10;

	if (param()->use_map_topic)
	{
		sub_map_ =
			nh_.subscribe("map", 1, &PFLocalizationNode::callbackMap, this);
		MRPT_LOG_INFO_FMT("Subscribed to map topic.");

		while (!first_map_received_ && ros::ok() && wait_counter < wait_limit)
		{
			MRPT_LOG_INFO_FMT("waiting for map callback..");
			ros::Duration(0.5).sleep();
			ros::spinOnce();
			wait_counter++;
		}
		if (wait_counter != wait_limit)
		{
			return true;
		}
	}
	else
	{
		client_map_ = nh_.serviceClient<nav_msgs::GetMap>("static_map");
		nav_msgs::GetMap srv;
		while (!client_map_.call(srv) && ros::ok() && wait_counter < wait_limit)
		{
			MRPT_LOG_INFO_FMT("waiting for map service!");
			ros::Duration(0.5).sleep();
			wait_counter++;
		}
		client_map_.shutdown();
		if (wait_counter != wait_limit)
		{
			MRPT_LOG_INFO_STREAM("Map service complete.");
			updateMap(srv.response.map);
			return true;
		}
	}

	ROS_WARN_STREAM("No map received.");
	return false;
}

void PFLocalizationNode::callbackMap(const nav_msgs::msg::OccupancyGrid& msg)
{
	if (param()->first_map_only && first_map_received_)
	{
		return;
	}

	MRPT_LOG_INFO_STREAM("Map received.");
	updateMap(msg);

	first_map_received_ = true;
}

void PFLocalizationNode::updateSensorPose(std::string _frame_id)
{
	mrpt::poses::CPose3D pose;

	std::string base_frame_id = param()->base_frame_id;

	geometry_msgs::TransformStamped transformStmp;
	try
	{
		ros::Duration timeout(1.0);

		transformStmp = tf_buffer_.lookupTransform(
			base_frame_id, _frame_id, ros::Time(0), timeout);
	}
	catch (const tf2::TransformException& e)
	{
		ROS_WARN(
			"Failed to get transform target_frame (%s) to source_frame (%s): "
			"%s",
			base_frame_id.c_str(), _frame_id.c_str(), e.what());
		return;
	}
	tf2::Transform transform;
	tf2::fromMsg(transformStmp.transform, transform);

	tf2::Vector3 translation = transform.getOrigin();
	tf2::Quaternion quat = transform.getRotation();
	pose.x() = translation.x();
	pose.y() = translation.y();
	pose.z() = translation.z();
	tf2::Matrix3x3 Rsrc(quat);
	mrpt::math::CMatrixDouble33 Rdes;
	for (int c = 0; c < 3; c++)
	{
		for (int r = 0; r < 3; r++)
		{
			Rdes(r, c) = Rsrc.getRow(r)[c];
		}
	}

	pose.setRotationMatrix(Rdes);
	laser_poses_[_frame_id] = pose;
	beacon_poses_[_frame_id] = pose;
}

void PFLocalizationNode::callbackInitialpose(
	const geometry_msgs::msg::PoseWithCovarianceStamped& _msg)
{
	const geometry_msgs::PoseWithCovariance& pose = _msg.pose;

	// SE(3) -> SE(2) explicit conversion:
	initial_pose_ =
		mrpt::poses::CPosePDFGaussian(mrpt::ros2bridge::fromROS(pose));

	update_counter_ = 0;
	state_ = INIT;
}

void PFLocalizationNode::callbackOdometry(const nav_msgs::msg::Odometry& _msg)
{
	// We always update the filter if update_while_stopped is true, regardless
	// robot is moving or
	// not; otherwise, update filter if we are moving or at initialization (100
	// first iterations)
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

void PFLocalizationNode::updateMap(const nav_msgs::msg::OccupancyGrid& _msg)
{
	ASSERT_(metric_map_->countMapsByClass<COccupancyGridMap2D>());
	mrpt::ros2bridge::fromROS(
		_msg, *metric_map_->mapByClass<COccupancyGridMap2D>());
}

bool PFLocalizationNode::mapCallback(
	nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res)
{
	MRPT_LOG_INFO_FMT("mapCallback: service requested!\n");
	res = resp_;
	return true;
}

void PFLocalizationNode::publishMap()
{
	resp_.map.header.stamp = ros::Time::now();
	resp_.map.header.frame_id = param()->global_frame_id;
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
		poseArray.header.frame_id = param()->global_frame_id;
		poseArray.header.stamp = ros::Time::now();
		poseArray.header.seq = loop_count_;
		poseArray.poses.resize(pdf_.particlesCount());
		for (size_t i = 0; i < pdf_.particlesCount(); i++)
		{
			const auto p = mrpt::math::TPose3D(pdf_.getParticlePose(i));
			poseArray.poses[i] = mrpt::ros2bridge::toROS_Pose(p);
		}
		mrpt::poses::CPose2D p;
		pub_particles_.publish(poseArray);
	}
}

/**
 * @brief Publish map -> odom tf; as the filter provides map -> base, we
 * multiply it by base -> odom
 */
void PFLocalizationNode::publishTF()
{
	static std::string base_frame_id = param()->base_frame_id;
	static std::string odom_frame_id = param()->odom_frame_id;
	static std::string global_frame_id = param()->global_frame_id;

	const mrpt::poses::CPose2D robotPoseFromPF = [this]() {
		return pdf_.getMeanVal();
	}();

	tf2::Transform baseOnMap_tf;
	tf2::fromMsg(mrpt::ros2bridge::toROS_Pose(robotPoseFromPF), baseOnMap_tf);

	ros::Time time_last_update(0.0);
	if (state_ == RUN)
	{
		time_last_update = mrpt::ros2bridge::toROS(time_last_update_);

		// Last update time can be too far in the past if we where not updating
		// filter, due to robot stopped or no
		// observations for a while (we optionally show a warning in the second
		// case)
		// We use time zero if so when getting base -> odom tf to prevent an
		// extrapolation into the past exception
		if ((ros::Time::now() - time_last_update).toSec() >
			param()->no_update_tolerance)
		{
			if ((ros::Time::now() - time_last_input_).toSec() >
				param()->no_inputs_tolerance)
			{
				ROS_WARN_THROTTLE(
					2.0,
					"No observations received for %.2fs (tolerance %.2fs); are "
					"robot sensors working?",
					(ros::Time::now() - time_last_input_).toSec(),
					param()->no_inputs_tolerance);
			}
			else
			{
				MRPT_LOG_DEBUG_FMT_THROTTLE(
					2.0,
					"No filter updates for %.2fs (tolerance %.2fs); probably "
					"robot stopped for a while",
					(ros::Time::now() - time_last_update).toSec(),
					param()->no_update_tolerance);
			}

			time_last_update = ros::Time(0.0);
		}
	}

	tf2::Transform odomOnBase_tf;

	{
		geometry_msgs::TransformStamped transform;
		try
		{
			transform = tf_buffer_.lookupTransform(
				base_frame_id, odom_frame_id, time_last_update,
				ros::Duration(0.1));
		}
		catch (const tf2::TransformException& e)
		{
			ROS_WARN_THROTTLE(
				2.0,
				"Failed to get transform target_frame (%s) to source_frame "
				"(%s): "
				"%s",
				base_frame_id.c_str(), odom_frame_id.c_str(), e.what());
			ROS_WARN_THROTTLE(
				2.0,
				"Ensure that your mobile base driver is broadcasting %s -> %s "
				"tf",
				odom_frame_id.c_str(), base_frame_id.c_str());

			return;
		}
		tf2::Transform tx;
		tf2::fromMsg(transform.transform, tx);
		odomOnBase_tf = tx;
	}

	// We want to send a transform that is good up until a tolerance time so
	// that odom can be used
	ros::Time transform_expiration =
		(time_last_update.isZero() ? ros::Time::now() : time_last_update) +
		ros::Duration(param()->transform_tolerance);

	tf2::Stamped<tf2::Transform> tmp_tf_stamped(
		baseOnMap_tf * odomOnBase_tf, transform_expiration, global_frame_id);

	geometry_msgs::TransformStamped tfGeom = tf2::toMsg(tmp_tf_stamped);
	tfGeom.child_frame_id = odom_frame_id;

	tf_broadcaster_.sendTransform(tfGeom);
}

/**
 * @brief Publish the current pose of the robot
 **/
void PFLocalizationNode::publishPose()
{
	// cov for x, y, phi (meter, meter, radian)
	const auto [cov, mean] = pdf_.getCovarianceAndMean();

	geometry_msgs::PoseWithCovarianceStamped p;

	// Fill in the header
	p.header.frame_id = param()->global_frame_id;
	if (loop_count_ < 10 || state_ == IDLE)
	{
		// on first iterations timestamp differs a lot from ROS time
		p.header.stamp = ros::Time::now();
	}
	else
	{
		p.header.stamp = mrpt::ros2bridge::toROS(time_last_update_);
	}

	// Copy in the pose
	p.pose.pose = mrpt::ros2bridge::toROS_Pose(mean);

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
	// Set ROS log level also on MRPT internal log system; level enums are fully
	// compatible
	std::map<std::string, ros::console::levels::Level> loggers;
	ros::console::get_loggers(loggers);
	if (loggers.find("ros.roscpp") != loggers.end())
		pdf_.setVerbosityLevel(
			static_cast<VerbosityLevel>(loggers["ros.roscpp"]));
	if (loggers.find("ros.mrpt_pf_localization") != loggers.end())
		pdf_.setVerbosityLevel(
			static_cast<VerbosityLevel>(loggers["ros.mrpt_pf_localization"]));
}

PFLocalizationNode::Parameters::Parameters(PFLocalizationNode* p)
	: PFLocalization::Parameters(p)
{
	node.param<double>("transform_tolerance", transform_tolerance, 0.1);
	MRPT_LOG_INFO_FMT("transform_tolerance: %f", transform_tolerance);
	node.param<double>("no_update_tolerance", no_update_tolerance, 1.0);
	MRPT_LOG_INFO_FMT("no_update_tolerance: %f", no_update_tolerance);
	node.param<double>(
		"no_inputs_tolerance", no_inputs_tolerance,
		std::numeric_limits<double>::infinity());
	MRPT_LOG_INFO_FMT(
		"no_inputs_tolerance: %f", no_inputs_tolerance);  // disabled by default
	node.param<double>("rate", rate, MRPT_LOCALIZATION_NODE_DEFAULT_RATE);
	MRPT_LOG_INFO_FMT("rate: %f", rate);
	node.getParam("gui_mrpt", gui_mrpt);
	MRPT_LOG_INFO_FMT("gui_mrpt: %s", gui_mrpt ? "true" : "false");
	node.param<int>(
		"parameter_update_skip", parameter_update_skip,
		MRPT_LOCALIZATION_NODE_DEFAULT_PARAMETER_UPDATE_SKIP);
	MRPT_LOG_INFO_FMT("parameter_update_skip: %i", parameter_update_skip);
	node.getParam("ini_file", ini_file);
	MRPT_LOG_INFO_FMT("ini_file: %s", ini_file.c_str());
	node.getParam("map_file", map_file);
	MRPT_LOG_INFO_FMT("map_file: %s", map_file.c_str());
	node.getParam("sensor_sources", sensor_sources);
	MRPT_LOG_INFO_FMT("sensor_sources: %s", sensor_sources.c_str());
	node.param<std::string>("global_frame_id", global_frame_id, "map");
	MRPT_LOG_INFO_FMT("global_frame_id: %s", global_frame_id.c_str());
	node.param<std::string>("odom_frame_id", odom_frame_id, "odom");
	MRPT_LOG_INFO_FMT("odom_frame_id: %s", odom_frame_id.c_str());
	node.param<std::string>("base_frame_id", base_frame_id, "base_link");
	MRPT_LOG_INFO_FMT("base_frame_id: %s", base_frame_id.c_str());
	node.param<bool>("pose_broadcast", pose_broadcast, false);
	MRPT_LOG_INFO_FMT("pose_broadcast: %s", pose_broadcast ? "true" : "false");
	node.param<bool>("tf_broadcast", tf_broadcast, true);
	MRPT_LOG_INFO_FMT("tf_broadcast: %s", tf_broadcast ? "true" : "false");
	node.param<bool>("use_map_topic", use_map_topic, false);
	MRPT_LOG_INFO_FMT("use_map_topic: %s", use_map_topic ? "true" : "false");
	node.param<bool>("first_map_only", first_map_only, false);
	MRPT_LOG_INFO_FMT("first_map_only: %s", first_map_only ? "true" : "false");
	node.param<bool>("debug", debug, true);
	MRPT_LOG_INFO_FMT("debug: %s", debug ? "true" : "false");

	reconfigure_cb_ = boost::bind(
		&PFLocalizationNode::Parameters::callbackParameters, this, _1, _2);
	reconfigure_server_.setCallback(reconfigure_cb_);
}

void PFLocalizationNode::Parameters::update(const unsigned long& loop_count)
{
	if (loop_count % parameter_update_skip) return;
	node.getParam("debug", debug);
	if (loop_count == 0)
		MRPT_LOG_INFO_FMT("debug: %s", debug ? "true" : "false");
	{
		int v = particlecloud_update_skip;
		node.param<int>(
			"particlecloud_update_skip", particlecloud_update_skip,
			MRPT_LOCALIZATION_NODE_DEFAULT_PARTICLECLOUD_UPDATE_SKIP);
		if (v != particlecloud_update_skip)
			MRPT_LOG_INFO_FMT(
				"particlecloud_update_skip: %i", particlecloud_update_skip);
	}
	{
		int v = map_update_skip;
		node.param<int>(
			"map_update_skip", map_update_skip,
			MRPT_LOCALIZATION_NODE_DEFAULT_MAP_UPDATE_SKIP);
		if (v != map_update_skip)
			MRPT_LOG_INFO_FMT("map_update_skip: %i", map_update_skip);
	}
}

void PFLocalizationNode::Parameters::callbackParameters(
	mrpt_pf_localization::MotionConfig& config, uint32_t level)
{
	if (config.motion_noise_type == MOTION_MODEL_GAUSSIAN)
	{
		motion_model_options->modelSelection =
			CActionRobotMovement2D::mmGaussian;

		motion_model_options->gaussianModel.a1 = config.gaussian_alpha_1;
		motion_model_options->gaussianModel.a2 = config.gaussian_alpha_2;
		motion_model_options->gaussianModel.a3 = config.gaussian_alpha_3;
		motion_model_options->gaussianModel.a4 = config.gaussian_alpha_4;
		motion_model_options->gaussianModel.minStdXY = config.gaussian_alpha_xy;
		motion_model_options->gaussianModel.minStdPHI =
			config.gaussian_alpha_phi;
		MRPT_LOG_INFO_FMT("gaussianModel.type: gaussian");
		MRPT_LOG_INFO_FMT(
			"gaussianModel.a1: %f", motion_model_options->gaussianModel.a1);
		MRPT_LOG_INFO_FMT(
			"gaussianModel.a2: %f", motion_model_options->gaussianModel.a2);
		MRPT_LOG_INFO_FMT(
			"gaussianModel.a3: %f", motion_model_options->gaussianModel.a3);
		MRPT_LOG_INFO_FMT(
			"gaussianModel.a4: %f", motion_model_options->gaussianModel.a4);
		MRPT_LOG_INFO_FMT(
			"gaussianModel.minStdXY: %f",
			motion_model_options->gaussianModel.minStdXY);
		MRPT_LOG_INFO_FMT(
			"gaussianModel.minStdPHI: %f",
			motion_model_options->gaussianModel.minStdPHI);
	}
	else
	{
		MRPT_LOG_INFO_FMT(
			"We support at the moment only gaussian motion models");
	}
	*use_motion_model_default_options = config.use_default_motion;
	MRPT_LOG_INFO_FMT(
		"use_motion_model_default_options: %s",
		use_motion_model_default_options ? "true" : "false");
	motion_model_default_options->gaussianModel.minStdXY =
		config.default_noise_xy;
	MRPT_LOG_INFO_FMT(
		"default_noise_xy: %f",
		motion_model_default_options->gaussianModel.minStdXY);
	motion_model_default_options->gaussianModel.minStdPHI =
		config.default_noise_phi;
	MRPT_LOG_INFO_FMT(
		"default_noise_phi: %f",
		motion_model_default_options->gaussianModel.minStdPHI);
	update_while_stopped = config.update_while_stopped;
	MRPT_LOG_INFO_FMT(
		"update_while_stopped: %s", update_while_stopped ? "true" : "false");
	update_sensor_pose = config.update_sensor_pose;
	MRPT_LOG_INFO_FMT(
		"update_sensor_pose: %s", update_sensor_pose ? "true" : "false");
}