/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include "mrpt_pf_localization_node.h"

#include <mp2p_icp/metricmap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRobotPose.h>
#include <mrpt/poses/Lie/SO.h>	// SO(3) logarithm
#include <mrpt/ros2bridge/gps.h>
#include <mrpt/ros2bridge/laser_scan.h>
#include <mrpt/ros2bridge/map.h>
#include <mrpt/ros2bridge/point_cloud2.h>
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/time.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/version.h>
#include <pose_cov_ops/pose_cov_ops.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <mrpt_msgs_bridge/beacon.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#if MRPT_VERSION >= 0x020b08
#include <mrpt/system/hyperlink.h>
#else
namespace mrpt::system	// backwards compatibility:
{
std::string hyperlink(
	const std::string& text, const std::string& uri, bool force_format = false,
	bool show_uri_anyway = false)
{
	return text;
}
}  // namespace mrpt::system
#endif

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<PFLocalizationNode>();
	rclcpp::spin(std::dynamic_pointer_cast<rclcpp::Node>(node));
	rclcpp::shutdown();
	return 0;
}

PFLocalizationNode::PFLocalizationNode(const rclcpp::NodeOptions& options)
	: rclcpp::Node("mrpt_pf_localization_node", options)
{
	using namespace std::string_literals;
	using std::placeholders::_1;

	// Redirect MRPT logger to ROS logger:
	core_.logging_enable_console_output = false;  // No console, go thru ROS
	core_.logRegisterCallback(
		[this](
			std::string_view msg, const mrpt::system::VerbosityLevel level,
			[[maybe_unused]] std::string_view loggerName,
			[[maybe_unused]] const mrpt::Clock::time_point timestamp)
		{
			switch (level)
			{
				case mrpt::system::LVL_DEBUG:
					// intentional: DEBUG -> INFO, to enable core DEBUG but node
					// INFO levels (see launch argument "log_level_core")
					RCLCPP_INFO_STREAM(this->get_logger(), msg);
					break;
				case mrpt::system::LVL_INFO:
					RCLCPP_INFO_STREAM(this->get_logger(), msg);
					break;
				case mrpt::system::LVL_WARN:
					RCLCPP_WARN_STREAM(this->get_logger(), msg);
					break;
				case mrpt::system::LVL_ERROR:
					RCLCPP_ERROR_STREAM(this->get_logger(), msg);
					break;
				default:
					break;
			};
		});

	// Params:
	// -----------------
	reload_params_from_ros();

	// Create all publishers and subscribers:
	// ------------------------------------------
	sub_init_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
		nodeParams_.topic_initialpose, rclcpp::SystemDefaultsQoS(),
		std::bind(&PFLocalizationNode::callbackInitialpose, this, _1));

	// See: REP-2003: https://ros.org/reps/rep-2003.html
	const auto mapQoS = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
	const auto sensorQoS = rclcpp::SensorDataQoS();

	subMap_ = this->create_subscription<mrpt_msgs::msg::GenericObject>(
		nodeParams_.topic_map, mapQoS, std::bind(&PFLocalizationNode::callbackMap, this, _1));

	subOdometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
		nodeParams_.topic_odometry, rclcpp::SystemDefaultsQoS(),
		std::bind(&PFLocalizationNode::callbackOdometry, this, _1));

	// Subscribe to one or more sensor sources:
	size_t numSensors = 0;

	{
		std::vector<std::string> sources;
		mrpt::system::tokenize(nodeParams_.topic_sensors_2d_scan, " ,\t\n", sources);
		for (const auto& topic : sources)
		{
			numSensors++;
			subs_2dlaser_.push_back(this->create_subscription<sensor_msgs::msg::LaserScan>(
				topic, sensorQoS,
				[topic, this](const sensor_msgs::msg::LaserScan& msg)
				{ callbackLaser(msg, topic); }));
		}
	}
	{
		std::vector<std::string> sources;
		mrpt::system::tokenize(nodeParams_.topic_sensors_point_clouds, " ,\t\n", sources);
		for (const auto& topic : sources)
		{
			numSensors++;
			subs_point_clouds_.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(
				topic, sensorQoS,
				[topic, this](const sensor_msgs::msg::PointCloud2& msg)
				{ callbackPointCloud(msg, topic); }));
		}
	}

	ASSERTMSG_(
		numSensors > 0,
		"At least one sensor input source must be defined! Refer to the "s +
			mrpt::system::hyperlink(
				"package documentation.", "https://github.com/mrpt-ros-pkg/mrpt_navigation",
				true /*force format*/));

	// optionally, subscribe to GPS/GNSS:
	subGNSS_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
		nodeParams_.topic_gnss, sensorQoS,
		[this](const sensor_msgs::msg::NavSatFix& msg) { callbackGNSS(msg); });

	// Publishers:
	pubParticles_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
		nodeParams_.pub_topic_particles, rclcpp::SystemDefaultsQoS());

	pubPose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
		nodeParams_.pub_topic_pose, rclcpp::SystemDefaultsQoS());

#if 0
		else if (sources[i].find("beacon") != std::string::npos)
		{
			sub_sensors_[i] = subscribe(
				sources[i], 1, &PFLocalizationNode::callbackBeacon, this);
		}
		else
		{
			sub_sensors_[i] = subscribe(
				sources[i], 1, &PFLocalizationNode::callbackRobotPose, this);
		}


	// On params change, reload all params:
	// -----------------------------------------
	// Trigger on change -> call:
#endif

	// Create the tf2 buffer and listener
	// ----------------------------------------
	tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

	// Create timer:
	// ------------------------------------------
	timer_ = this->create_wall_timer(
		std::chrono::microseconds(mrpt::round(1.0e6 / nodeParams_.rate_hz)),
		[this]() { this->loop(); });

	ASSERT_GT_(nodeParams_.transform_tolerance, 1e-3);
	timerPubTF_ = this->create_wall_timer(
		std::chrono::microseconds(mrpt::round(0.5 * 1.0e6 * nodeParams_.transform_tolerance)),
		[this]()
		{
			this->publishTF();
			// publishParticles() && publishPose() are done inside loop()
		});
}

PFLocalizationNode::~PFLocalizationNode() = default;

void PFLocalizationNode::reload_params_from_ros()
{
	// Use MRPT library the same log level as on ROS nodes (only for
	// MRPT_VERSION >= 0x150)
	useROSLogLevel();

	// Unify all ROS params into a in-memory YAML block and pass it to the core
	// object:
	mrpt::containers::yaml paramsBlock = mrpt::containers::yaml::Map();

	const auto& paramsIf = this->get_node_parameters_interface();
	const auto& allParams = paramsIf->get_parameter_overrides();

	for (const auto& kv : allParams)
	{
		// Get param name:
		std::string name = kv.first;

		// ROS2 param names may be nested. Convert that back into YAML nodes:
		// e.g. "foo.bar" -> ["foo"]["bar"].
		mrpt::containers::yaml::map_t* targetYamlNode = &paramsBlock.node().asMap();

		for (auto pos = name.find("."); pos != std::string::npos; pos = name.find("."))
		{
			// Split:
			const std::string parentKey = name.substr(0, pos);
			const std::string childKey = name.substr(pos + 1);
			name = childKey;

			// Use subnode:
			if (auto it = targetYamlNode->find(parentKey); it == targetYamlNode->end())
			{  // create new:
				(*targetYamlNode)[parentKey] = mrpt::containers::yaml::Map();
				targetYamlNode = &(*targetYamlNode)[parentKey].asMap();
			}
			else
			{  // reuse
				targetYamlNode = &it->second.asMap();
			}
		}

		// Get param value:
		switch (kv.second.get_type())
		{
			case rclcpp::ParameterType::PARAMETER_BOOL:
				(*targetYamlNode)[name] = kv.second.get<bool>();
				break;
			case rclcpp::ParameterType::PARAMETER_DOUBLE:
				(*targetYamlNode)[name] = kv.second.get<double>();
				break;
			case rclcpp::ParameterType::PARAMETER_INTEGER:
				(*targetYamlNode)[name] = kv.second.get<int>();
				break;
			case rclcpp::ParameterType::PARAMETER_STRING:
				(*targetYamlNode)[name] = kv.second.get<std::string>();
				break;
			default:
				RCLCPP_WARN(
					get_logger(), "ROS2 parameter not handled: '%s' type: '%i'", name.c_str(),
					static_cast<int>(kv.second.get_type()));
				break;
		}
	}

	RCLCPP_DEBUG_STREAM(get_logger(), paramsBlock);

	mrpt::containers::yaml relocalizationCfg;
	if (paramsBlock.has("relocalization_params_file"))
	{
		relocalizationCfg.loadFromFile(paramsBlock["relocalization_params_file"]);
	}

	core_.init_from_yaml(paramsBlock, relocalizationCfg);
	nodeParams_.loadFrom(paramsBlock);
}

void PFLocalizationNode::loop()
{
	// Populate PF input with a "fake" odometry from twist estimation
	// if we have nothing better:
	createOdometryFromTwist();

	// PF algorithm:
	core_.step();

	// Estimate twist for the case of not having odometry:
	updateEstimatedTwist();

	// Publish to ROS:
	publishParticlesAndStampedPose();

	// /tf data is published in its own timer, save data here in this thread:
	update_tf_pub_data();

	MRPT_TODO("pub quality metrics");

	loopCount_++;  // used to compute decimation for publishing msgs
}

bool PFLocalizationNode::waitForTransform(
	mrpt::poses::CPose3D& des, const std::string& frame, const std::string& referenceFrame,
	const int timeoutMilliseconds)
{
	const rclcpp::Duration timeout(0, 1000 * timeoutMilliseconds);
	try
	{
		geometry_msgs::msg::TransformStamped ref_to_trgFrame = tf_buffer_->lookupTransform(
			referenceFrame, frame, tf2::TimePointZero, tf2::durationFromSec(timeout.seconds()));

		tf2::Transform tf;
		tf2::fromMsg(ref_to_trgFrame.transform, tf);
		des = mrpt::ros2bridge::fromROS(tf);

		RCLCPP_DEBUG(
			get_logger(), "[waitForTransform] Found pose %s -> %s: %s", referenceFrame.c_str(),
			frame.c_str(), des.asString().c_str());

		return true;
	}
	catch (const tf2::TransformException& ex)
	{
		RCLCPP_ERROR(get_logger(), "[waitForTransform] %s", ex.what());
		return false;
	}
}

void PFLocalizationNode::callbackLaser(
	const sensor_msgs::msg::LaserScan& msg, const std::string& topicName)
{
	RCLCPP_DEBUG(get_logger(), "Received 2D scan (%s)", topicName.c_str());

	// get sensor pose on the robot:
	mrpt::poses::CPose3D sensorPose;
	bool sensorPoseOK =
		waitForTransform(sensorPose, msg.header.frame_id, nodeParams_.base_link_frame_id);
	if (!sensorPoseOK) return;	// error msg already printed in waitForTransform()

	auto obs = mrpt::obs::CObservation2DRangeScan::Create();
	mrpt::ros2bridge::fromROS(msg, sensorPose, *obs);

	obs->sensorLabel = topicName;

	last_sensor_stamp_ = obs->timestamp;

	core_.on_observation(obs);
}

void PFLocalizationNode::callbackPointCloud(
	const sensor_msgs::msg::PointCloud2& msg, const std::string& topicName)
{
	RCLCPP_DEBUG(get_logger(), "Received point cloud (%s)", topicName.c_str());

	// get sensor pose on the robot:
	mrpt::poses::CPose3D sensorPose;
	bool sensorPoseOK =
		waitForTransform(sensorPose, msg.header.frame_id, nodeParams_.base_link_frame_id);
	if (!sensorPoseOK) return;	// error msg already printed in waitForTransform()

	auto obs = mrpt::obs::CObservationPointCloud::Create();
	obs->sensorLabel = topicName;
	auto pts = mrpt::maps::CSimplePointsMap::Create();
	obs->pointcloud = pts;
	mrpt::ros2bridge::fromROS(msg, *pts);

	obs->sensorLabel = topicName;

	last_sensor_stamp_ = obs->timestamp;

	core_.on_observation(obs);
}

void PFLocalizationNode::callbackBeacon(const mrpt_msgs::msg::ObservationRangeBeacon& _msg)
{
#if 0
	using namespace mrpt::maps;
	using namespace mrpt::obs;

	time_last_input_ = ros::Time::now();

	// MRPT_LOG_INFO_FMT("callbackBeacon");
	auto beacon = CObservationBeaconRanges::Create();
	// printf("callbackBeacon %s\n", _msg.header.frame_id.c_str());
	{
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
#endif
}

void PFLocalizationNode::callbackRobotPose(
	const geometry_msgs::msg::PoseWithCovarianceStamped& _msg)
{
#if 0
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
			"Failed to get transform frame (%s) to referenceFrame (%s): "
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
#endif
}

void PFLocalizationNode::callbackMap(const mrpt_msgs::msg::GenericObject& obj)
{
	RCLCPP_INFO(get_logger(), "[callbackMap] Received a metric map via ROS topic");

	mrpt::serialization::CSerializable::Ptr o;
	mrpt::serialization::OctetVectorToObject(obj.data, o);

	ASSERT_(o);
	auto mm = std::dynamic_pointer_cast<mp2p_icp::metric_map_t>(o);
	ASSERTMSG_(
		mm, mrpt::format(
				"Expected incoming map of type mp2p_icp::metric_map_t but it "
				"is '%s'",
				o->GetRuntimeClass()->className));

	RCLCPP_INFO_STREAM(get_logger(), "[callbackMap] Map contents: " << mm->contents_summary());

	core_.set_map_from_metric_map(*mm);
}

void PFLocalizationNode::callbackInitialpose(
	const geometry_msgs::msg::PoseWithCovarianceStamped& msg)
{
	const geometry_msgs::msg::PoseWithCovariance& pose = msg.pose;

	const auto initial_pose = mrpt::ros2bridge::fromROS(pose);

	RCLCPP_INFO_STREAM(get_logger(), "[callbackInitialpose] Received: " << initial_pose);

	// Send to core PF runner:
	core_.relocalize_here(initial_pose);
}

void PFLocalizationNode::callbackOdometry(const nav_msgs::msg::Odometry& msg)
{
	auto obs = mrpt::obs::CObservationOdometry::Create();
	obs->timestamp = mrpt::ros2bridge::fromROS(msg.header.stamp);
	obs->sensorLabel = "odom";

	obs->hasVelocities = true;
	obs->velocityLocal = {
		msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z};

	// SE(3) -> SE(2):
	obs->odometry = mrpt::poses::CPose2D(mrpt::ros2bridge::fromROS(msg.pose.pose));

	last_sensor_stamp_ = obs->timestamp;

	core_.on_observation(obs);
}

void PFLocalizationNode::callbackGNSS(const sensor_msgs::msg::NavSatFix& msg)
{
	RCLCPP_DEBUG_STREAM(get_logger(), "Received GNSS observation");

	// get sensor pose on the robot:
	mrpt::poses::CPose3D sensorPose;
	bool sensorPoseOK =
		waitForTransform(sensorPose, msg.header.frame_id, nodeParams_.base_link_frame_id);
	if (!sensorPoseOK) return;	// error msg already printed in waitForTransform()

	auto obs = mrpt::obs::CObservationGPS::Create();

	bool ok = mrpt::ros2bridge::fromROS(msg, *obs);
	if (!ok)
	{
		RCLCPP_WARN_STREAM(get_logger(), "Could not convert ROS NavSatFix to an MRPT observation.");
		return;
	}

	obs->sensorLabel = "gps";

	// Only count this as sensor timestamp if it's the first one for
	// initialization, so we have a valid stamp to publish the first set of
	// particles:
	if (!last_sensor_stamp_) last_sensor_stamp_ = obs->timestamp;

	core_.on_observation(obs);
}

void PFLocalizationNode::publishParticlesAndStampedPose()
{
	const mrpt::poses::CPose3DPDFParticles::Ptr parts = core_.getLastPoseEstimation();

	if (!parts)
	{
		// No solution yet
		return;
	}

	if (!last_sensor_stamp_.has_value()) return;

	const auto stamp = mrpt::ros2bridge::toROS(*last_sensor_stamp_);

	// publish particles:
	if (pubParticles_->get_subscription_count())
	{
		geometry_msgs::msg::PoseArray poseArray;
		poseArray.header.frame_id = nodeParams_.global_frame_id;
		poseArray.header.stamp = stamp;

		if (!parts)
		{  // no solution yet
			poseArray.poses.resize(0);
		}
		else
		{
			poseArray.poses.resize(parts->size());
			for (size_t i = 0; i < parts->size(); i++)
			{
				const auto p = parts->getParticlePose(i);
				poseArray.poses[i] = mrpt::ros2bridge::toROS_Pose(p);
			}
		}
		pubParticles_->publish(poseArray);
	}

	if (pubPose_->get_subscription_count())
	{
		geometry_msgs::msg::PoseWithCovarianceStamped p;
		p.header.frame_id = nodeParams_.global_frame_id;
		p.header.stamp = stamp;

		mrpt::poses::CPose3DPDFGaussian pdf;
		pdf.copyFrom(*parts);

		p.pose = mrpt::ros2bridge::toROS_Pose(pdf);

		pubPose_->publish(p);
	}
}

/**
 * @brief Publish map -> odom tf; as the filter provides map -> base, we
 * multiply it by base -> odom
 */
void PFLocalizationNode::update_tf_pub_data()
{
	std::string base_frame_id = nodeParams_.base_link_frame_id;
	std::string odom_frame_id = nodeParams_.odom_frame_id;
	std::string global_frame_id = nodeParams_.global_frame_id;

	const auto posePdf = core_.getLastPoseEstimation();
	if (!posePdf) return;  // No solution yet.
	if (!last_sensor_stamp_) return;

	const auto estimatedPose = posePdf->getMeanVal();

	MRPT_TODO("Use param: no_update_tolerance");

	mrpt::poses::CPose3D T_base_to_odom;
	bool base_to_odom_ok = this->waitForTransform(T_base_to_odom, odom_frame_id, base_frame_id);
	// Note: this wait above typ takes ~50 us

	if (!base_to_odom_ok)
	{
		// Ignore error and assume we don't have odom and this localization is
		// the only source of localization?
	}

	// We want to send a transform that is good up until a tolerance time so
	// that odom can be used

	const tf2::Transform baseOnMap_tf = mrpt::ros2bridge::toROS_tfTransform(estimatedPose);

	const tf2::Transform odomOnBase_tf = mrpt::ros2bridge::toROS_tfTransform(T_base_to_odom);

	const auto tf_tolerance = tf2::durationFromSec(nodeParams_.transform_tolerance);

	tf2::TimePoint transform_expiration =
		tf2_ros::fromMsg(mrpt::ros2bridge::toROS(*last_sensor_stamp_)) + tf_tolerance;

	tf2::Stamped<tf2::Transform> tmp_tf_stamped(
		baseOnMap_tf * odomOnBase_tf, transform_expiration, global_frame_id);

	auto lck = mrpt::lockHelper(tfMapOdomToPublishMtx_);

	tfMapOdomToPublish_ = tf2::toMsg(tmp_tf_stamped);
	tfMapOdomToPublish_->child_frame_id = odom_frame_id;
}

void PFLocalizationNode::publishTF()
{
	auto lck = mrpt::lockHelper(tfMapOdomToPublishMtx_);

	if (!tfMapOdomToPublish_.has_value()) return;

	tf_broadcaster_->sendTransform(*tfMapOdomToPublish_);

	const auto tf_tolerance_1_2 = tf2::durationFromSec(0.5 * nodeParams_.transform_tolerance);

	RCLCPP_DEBUG_STREAM(
		get_logger(),
		"[publishTF] last_sensor_stamp="
			<< mrpt::system::dateTimeToString(
				   mrpt::ros2bridge::fromROS(tfMapOdomToPublish_->header.stamp))
			<< " now="
			<< mrpt::system::dateTimeToString(mrpt::ros2bridge::fromROS(get_clock()->now())));

	// Increase timestamp to keep it valid on next re-publish and until a better
	// odom->map is found.
	tfMapOdomToPublish_->header.stamp =
		tf2_ros::toMsg(tf2_ros::fromMsg(tfMapOdomToPublish_->header.stamp) + tf_tolerance_1_2);
}

void PFLocalizationNode::useROSLogLevel()
{
	const auto rosLogLevel = rcutils_logging_get_logger_level(get_logger().get_name());

	mrpt::system::VerbosityLevel lvl = core_.getMinLoggingLevel();

	if (rosLogLevel <= RCUTILS_LOG_SEVERITY_DEBUG)	// 10
		lvl = mrpt::system::LVL_DEBUG;
	else if (rosLogLevel <= RCUTILS_LOG_SEVERITY_INFO)	// 20
		lvl = mrpt::system::LVL_INFO;
	else if (rosLogLevel <= RCUTILS_LOG_SEVERITY_WARN)	// 30
		lvl = mrpt::system::LVL_WARN;
	else if (rosLogLevel <= RCUTILS_LOG_SEVERITY_ERROR)	 // 40
		lvl = mrpt::system::LVL_ERROR;

	core_.setVerbosityLevel(lvl);
}

void PFLocalizationNode::NodeParameters::loadFrom(const mrpt::containers::yaml& cfg)
{
	MCP_LOAD_OPT(cfg, rate_hz);
	MCP_LOAD_OPT(cfg, transform_tolerance);
	MCP_LOAD_OPT(cfg, no_update_tolerance);
	MCP_LOAD_OPT(cfg, no_inputs_tolerance);

	MCP_LOAD_OPT(cfg, base_link_frame_id);
	MCP_LOAD_OPT(cfg, odom_frame_id);
	MCP_LOAD_OPT(cfg, global_frame_id);

	MCP_LOAD_OPT(cfg, topic_map);
	MCP_LOAD_OPT(cfg, topic_initialpose);
	MCP_LOAD_OPT(cfg, topic_odometry);

	MCP_LOAD_OPT(cfg, pub_topic_particles);
	MCP_LOAD_OPT(cfg, pub_topic_pose);

	MCP_LOAD_OPT(cfg, topic_sensors_2d_scan);
	MCP_LOAD_OPT(cfg, topic_sensors_point_clouds);
	MCP_LOAD_OPT(cfg, topic_gnss);
}

void PFLocalizationNode::updateEstimatedTwist()
{
	const mrpt::poses::CPose3DPDFParticles::Ptr parts = core_.getLastPoseEstimation();

	// No solution yet
	if (!parts) return;

	if (!last_sensor_stamp_) return;

	const auto curStamp = *last_sensor_stamp_;

	// estimate twist:
	if (!prevParts_)
	{
		prevParts_ = *parts;
		prevStamp_ = curStamp;
		return;
	}

	// else, we can estimate the twist if we have two observations:
	if (curStamp == prevStamp_) return;	 // No new observation yet, keep waiting...

	// get diff:
	const auto prevPose = prevParts_->getMeanVal();
	const auto curPose = parts->getMeanVal();

	const double dt = mrpt::system::timeDifference(*prevStamp_, curStamp);

	const double max_time_to_use_velocity_model = 5.0;	// [s]

	if (dt < max_time_to_use_velocity_model)
	{
		ASSERT_GT_(dt, .0);

		auto& tw = estimated_twist_.emplace();

		const auto incrPose = curPose - prevPose;

		tw.vx = incrPose.x() / dt;
		tw.vy = incrPose.y() / dt;
		tw.vz = incrPose.z() / dt;

		const auto logRot = mrpt::poses::Lie::SO<3>::log(incrPose.getRotationMatrix());

		tw.wx = logRot[0] / dt;
		tw.wy = logRot[1] / dt;
		tw.wz = logRot[2] / dt;
	}

	// for the next iteration:
	prevParts_ = *parts;
	prevStamp_ = curStamp;
}

void PFLocalizationNode::createOdometryFromTwist()
{
	using mrpt::math::TVector3D;

	// no twist estimation, cannot do anything
	if (!estimated_twist_ || !prevStamp_) return;

	if (core_.input_queue_has_odometry()) return;  // already has a real odometry

	const auto lastStamp = core_.input_queue_last_stamp();
	if (!lastStamp) return;	 // no real observation with stamp

	const double dt = mrpt::system::timeDifference(*prevStamp_, *lastStamp);

	const TVector3D v(estimated_twist_->vx, estimated_twist_->vy, estimated_twist_->vz);

	const TVector3D w(estimated_twist_->wx, estimated_twist_->wy, estimated_twist_->wz);

	// Integrate rotation:
	const TVector3D w_dt = w * dt;
	const auto R = mrpt::poses::Lie::SO<3>::exp(mrpt::math::CVectorFixedDouble<3>(w_dt));

	// Integrate translation:
	const TVector3D v_dt = v * dt;

	// Build SE(3):
	const auto deltaT = mrpt::poses::CPose3D::FromRotationAndTranslation(R, v_dt);

	// Set fake odometry:
	core_.set_fake_odometry_increment(deltaT);

	RCLCPP_DEBUG_STREAM(get_logger(), "createOdometryFromTwist: dt=" << dt << " deltaT=" << deltaT);
}
