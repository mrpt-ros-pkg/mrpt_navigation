/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include "mrpt_reactivenav2d/mrpt_reactivenav2d_node.hpp"

#include <stdexcept>

using namespace mrpt::nav;
using mrpt::maps::CSimplePointsMap;
using namespace mrpt::system;
using namespace mrpt::config;

/**  Constructor: Inits ROS system */
ReactiveNav2DNode::ReactiveNav2DNode(const rclcpp::NodeOptions& options)
	: Node("mrpt_reactivenav2d", options)
{
	// Load params
	read_parameters();

	ASSERT_(navPeriod_ > 0);

	if (cfgFileReactive_.empty())
	{
		RCLCPP_ERROR(
			this->get_logger(),
			"Mandatory param 'cfg_file_reactive' is missing!");
		throw;
	}

	if (!mrpt::system::fileExists(cfgFileReactive_))
	{
		RCLCPP_ERROR(
			this->get_logger(), "Config file not found: %s",
			cfgFileReactive_.c_str());
		throw;
	}

	rnavEngine_.enableLogFile(saveNavLog_);

	// Load reactive config:
	// ----------------------------------------------------
	if (!cfgFileReactive_.empty())
	{
		try
		{
			CConfigFile cfgFil(cfgFileReactive_);
			rnavEngine_.loadConfigFile(cfgFil);
		}
		catch (std::exception& e)
		{
			RCLCPP_ERROR(
				this->get_logger(),
				"Exception initializing reactive navigation engine:\n%s",
				e.what());
			throw;
		}
	}
	// load robot shape: (1) default, (2) via params, (3) via topic
	// ----------------------------------------------------------------
	// m_reactive_nav_engine.changeRobotShape();

	// Init this subscriber first so we know asap the desired robot shape,
	// if provided via a topic:
	if (!subTopicRobotShape_.empty())
	{
		subRobotShape_ = this->create_subscription<geometry_msgs::msg::Polygon>(
			subTopicRobotShape_, 1,
			[this](const geometry_msgs::msg::Polygon::SharedPtr poly) {
				this->on_set_robot_shape(poly);
			});

		RCLCPP_INFO(
			this->get_logger(),
			"Params say robot shape will arrive via topic '%s'... waiting 3 "
			"seconds for it.",
			subTopicRobotShape_.c_str());

		// Use rate object to implement sleep
		rclcpp::Rate rate(1);  // 1 Hz
		for (int i = 0; i < 3; i++)
		{
			rclcpp::spin_some(this->get_node_base_interface());
			rate.sleep();
		}
		RCLCPP_INFO(this->get_logger(), "Wait done.");
	}
	else
	{
		// Load robot shape: 1/2 polygon
		// ---------------------------------------------
		CConfigFile c(cfgFileReactive_);
		std::string s = "CReactiveNavigationSystem";

		std::vector<float> xs, ys;
		c.read_vector(
			s, "RobotModel_shape2D_xs", std::vector<float>(), xs, false);
		c.read_vector(
			s, "RobotModel_shape2D_ys", std::vector<float>(), ys, false);
		ASSERTMSG_(
			xs.size() == ys.size(),
			"Config parameters `RobotModel_shape2D_xs` and "
			"`RobotModel_shape2D_ys` "
			"must have the same length!");
		if (!xs.empty())
		{
			mrpt::math::CPolygon poly;
			poly.resize(xs.size());
			for (size_t i = 0; i < xs.size(); i++)
			{
				poly[i].x = xs[i];
				poly[i].y = ys[i];
			}

			std::lock_guard<std::mutex> csl(rnavEngineMtx_);
			rnavEngine_.changeRobotShape(poly);
		}

		// Load robot shape: 2/2 circle
		// ---------------------------------------------
		if (const double robot_radius = c.read_double(
				s, "RobotModel_circular_shape_radius", -1.0, false);
			robot_radius > 0)
		{
			std::lock_guard<std::mutex> csl(rnavEngineMtx_);
			rnavEngine_.changeRobotCircularShapeRadius(robot_radius);
		}
	}

	// Init ROS publishers:
	// -----------------------
	pubCmdVel_ =
		this->create_publisher<geometry_msgs::msg::Twist>(pubTopicCmdVel_, 1);

	pubSelectedPtg_ =
		this->create_publisher<visualization_msgs::msg::MarkerArray>(
			pubTopicSelectedPtg_, 1);

	// Init ROS subs:
	// -----------------------
	subOdometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
		subTopicOdometry_, 1,
		[this](const nav_msgs::msg::Odometry::SharedPtr odom) {
			this->on_odometry_received(odom);
		});

	subWpSeq_ = this->create_subscription<mrpt_msgs::msg::WaypointSequence>(
		subTopicWpSeq_, 1,
		[this](const mrpt_msgs::msg::WaypointSequence::SharedPtr msg) {
			this->on_waypoint_seq_received(msg);
		});

	subNavGoal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
		subTopicNavGoal_, 1,
		[this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
			this->on_goal_received(msg);
		});

	subLocalObs_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
		subTopicLocalObstacles_, 1,
		[this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
			this->on_local_obstacles(msg);
		});

	// Init tf buffers
	// ----------------------------------------------------
	tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

	// Init timer:
	// ----------------------------------------------------
	timerRunNav_ = this->create_wall_timer(
		std::chrono::duration<double>(navPeriod_),
		[this]() { this->on_do_navigation(); });

}  // end ctor

void ReactiveNav2DNode::read_parameters()
{
	declare_parameter<std::string>(
		"cfg_file_reactive", "reactive2d_config.ini");
	get_parameter("cfg_file_reactive", cfgFileReactive_);
	RCLCPP_INFO(
		this->get_logger(), "cfg_file_reactive %s", cfgFileReactive_.c_str());

	declare_parameter<double>(
		"target_allowed_distance", targetAllowedDistance_);
	get_parameter("target_allowed_distance", targetAllowedDistance_);
	RCLCPP_INFO(
		this->get_logger(), "target_allowed_distance: %f",
		targetAllowedDistance_);

	declare_parameter<double>("nav_period", navPeriod_);
	get_parameter("nav_period", navPeriod_);
	RCLCPP_INFO(this->get_logger(), "nav_period: %f", navPeriod_);

	declare_parameter<std::string>("frameid_reference", frameidReference_);
	get_parameter("frameid_reference", frameidReference_);
	RCLCPP_INFO(
		this->get_logger(), "frameid_reference: %s", frameidReference_.c_str());

	declare_parameter<std::string>("frameid_robot", frameidRobot_);
	get_parameter("frameid_robot", frameidRobot_);
	RCLCPP_INFO(this->get_logger(), "frameid_robot: %s", frameidRobot_.c_str());

	declare_parameter<std::string>("topic_wp_seq", subTopicWpSeq_);
	get_parameter("topic_wp_seq", subTopicWpSeq_);
	RCLCPP_INFO(this->get_logger(), "topic_wp_seq: %s", subTopicWpSeq_.c_str());

	declare_parameter<std::string>("topic_reactive_nav_goal", subTopicNavGoal_);
	get_parameter("topic_reactive_nav_goal", subTopicNavGoal_);
	RCLCPP_INFO(
		this->get_logger(), "topic_reactive_nav_goal: %s",
		subTopicNavGoal_.c_str());

	declare_parameter<std::string>("topic_odometry", subTopicOdometry_);
	get_parameter("topic_odometry", subTopicOdometry_);
	RCLCPP_INFO(
		this->get_logger(), "topic_odometry: %s", subTopicOdometry_.c_str());

	declare_parameter<std::string>("topic_cmd_vel", pubTopicCmdVel_);
	get_parameter("topic_cmd_vel", pubTopicCmdVel_);
	RCLCPP_INFO(
		this->get_logger(), "topic_cmd_vel: %s", pubTopicCmdVel_.c_str());

	declare_parameter<std::string>("topic_obstacles", subTopicLocalObstacles_);
	get_parameter("topic_obstacles", subTopicLocalObstacles_);
	RCLCPP_INFO(
		this->get_logger(), "topic_obstacles: %s",
		subTopicLocalObstacles_.c_str());

	declare_parameter<std::string>("topic_robot_shape", subTopicRobotShape_);
	get_parameter("topic_robot_shape", subTopicRobotShape_);
	RCLCPP_INFO(
		this->get_logger(), "topic_robot_shape: %s",
		subTopicRobotShape_.c_str());

	declare_parameter<bool>("save_nav_log", false);
	get_parameter("save_nav_log", saveNavLog_);
	RCLCPP_INFO(
		this->get_logger(), "save_nav_log: %s", saveNavLog_ ? "yes" : "no");

	declare_parameter<std::string>("ptg_plugin_files", "");
	get_parameter("ptg_plugin_files", pluginFile_);
	RCLCPP_INFO(
		this->get_logger(), "ptg_plugin_files: %s", pluginFile_.c_str());

	if (!pluginFile_.empty())
	{
		RCLCPP_INFO_STREAM(
			this->get_logger(), "About to load plugins: " << pluginFile_);
		std::string errorMsgs;
		if (!mrpt::system::loadPluginModules(pluginFile_, errorMsgs))
		{
			RCLCPP_ERROR_STREAM(
				this->get_logger(),
				"Error loading rnav plugins: " << errorMsgs);
		}
		RCLCPP_INFO_STREAM(this->get_logger(), "Pluginns loaded OK.");
	}
}

/**
 * @brief Issue a navigation command
 * @param target The target location
 */
void ReactiveNav2DNode::navigate_to(const mrpt::math::TPose2D& target)
{
	RCLCPP_INFO(
		this->get_logger(), "[navigateTo] Starting navigation to %s",
		target.asString().c_str());

	CAbstractPTGBasedReactive::TNavigationParamsPTG navParams;

	CAbstractNavigator::TargetInfo target_info;
	target_info.target_coords.x = target.x;
	target_info.target_coords.y = target.y;
	target_info.targetAllowedDistance = targetAllowedDistance_;
	target_info.targetIsRelative = false;

	// API for single targets:
	navParams.target = target_info;

	// Optional: restrict the PTGs to use
	// navParams.restrict_PTG_indices.push_back(1);

	{
		std::lock_guard<std::mutex> csl(rnavEngineMtx_);
		rnavEngine_.navigate(&navParams);
	}
}

/** Callback: On run navigation */
void ReactiveNav2DNode::on_do_navigation()
{
	// 1st time init:
	// ----------------------------------------------------
	if (!initialized_)
	{
		initialized_ = true;
		RCLCPP_INFO(
			this->get_logger(),
			"[ReactiveNav2DNode] Initializing reactive navigation "
			"engine...");
		{
			std::lock_guard<std::mutex> csl(rnavEngineMtx_);
			rnavEngine_.initialize();
		}
		RCLCPP_INFO(
			this->get_logger(),
			"[ReactiveNav2DNode] Reactive navigation engine init done!");
	}

	rnavEngine_.enableKeepLogRecords();

	CTimeLoggerEntry tle(profiler_, "on_do_navigation");
	// Main nav loop (in whatever state nav is: IDLE, NAVIGATING, etc.)
	rnavEngine_.navigationStep();

	tle.stop();

	// get last decision and publish it to the ROS system for debugging:
	mrpt::nav::CLogFileRecord lr;
	rnavEngine_.getLastLogRecord(lr);

	publish_last_log_record_to_ros(lr);
}

void ReactiveNav2DNode::on_odometry_received(
	const nav_msgs::msg::Odometry::SharedPtr& msg)
{
	std::lock_guard<std::mutex> csl(odometryMtx_);
	tf2::Quaternion quat(
		msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	tf2::Matrix3x3 mat(quat);
	double roll, pitch, yaw;
	mat.getRPY(roll, pitch, yaw);
	odometry_.odometry = mrpt::poses::CPose2D(
		msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);

	odometry_.velocityLocal.vx = msg->twist.twist.linear.x;
	odometry_.velocityLocal.vy = msg->twist.twist.linear.y;
	odometry_.velocityLocal.omega = msg->twist.twist.angular.z;
	odometry_.hasVelocities = true;

	RCLCPP_DEBUG_STREAM(this->get_logger(), "Odometry updated");
}

void ReactiveNav2DNode::on_waypoint_seq_received(
	const mrpt_msgs::msg::WaypointSequence::SharedPtr& wps)
{
	update_waypoint_sequence(std::move(wps));
}

void ReactiveNav2DNode::update_waypoint_sequence(
	const mrpt_msgs::msg::WaypointSequence::SharedPtr& msg)
{
	mrpt::nav::TWaypointSequence wps;

	mrpt::poses::CPose3D relPose = mrpt::poses::CPose3D::Identity();

	// Convert to the "m_frameid_reference" frame of coordinates:
	if (msg->header.frame_id != frameidReference_)
		waitForTransform(relPose, frameidReference_, msg->header.frame_id);

	for (const auto& wp : msg->waypoints)
	{
		auto trg = mrpt::ros2bridge::fromROS(wp.target);
		trg = relPose + trg;  // local to global frame, if needed.

		auto waypoint = mrpt::nav::TWaypoint(
			trg.x(), trg.y(), wp.allowed_distance, wp.allow_skip);

		// regular number, not NAN
		if (trg.yaw() == trg.yaw() && !wp.ignore_heading)
			waypoint.target_heading = trg.yaw();

		wps.waypoints.push_back(waypoint);
	}

	RCLCPP_INFO_STREAM(this->get_logger(), "New navigateWaypoints() command");
	{
		std::lock_guard<std::mutex> csl(rnavEngineMtx_);
		rnavEngine_.navigateWaypoints(wps);
	}
}

void ReactiveNav2DNode::on_goal_received(
	const geometry_msgs::msg::PoseStamped::SharedPtr& trg_ptr)
{
	geometry_msgs::msg::PoseStamped trg = *trg_ptr;

	RCLCPP_INFO(
		this->get_logger(),
		"Nav target received via topic sub: (%.03f,%.03f, %.03fdeg) "
		"[frame_id=%s]",
		trg.pose.position.x, trg.pose.position.y,
		trg.pose.orientation.z * 180.0 / M_PI, trg.header.frame_id.c_str());

	auto trgPose = mrpt::ros2bridge::fromROS(trg.pose);

	// Convert to the "m_frameid_reference" frame of coordinates:
	if (trg.header.frame_id != frameidReference_)
	{
		mrpt::poses::CPose3D relPose;
		waitForTransform(relPose, frameidReference_, trg_ptr->header.frame_id);
		trgPose = relPose + trgPose;
	}

	this->navigate_to(mrpt::poses::CPose2D(trgPose).asTPose());
}

void ReactiveNav2DNode::on_local_obstacles(
	const sensor_msgs::msg::PointCloud2::SharedPtr& obs)
{
	std::lock_guard<std::mutex> csl(lastObstaclesMtx_);
	mrpt::ros2bridge::fromROS(*obs, lastObstacles_);
	RCLCPP_DEBUG(
		this->get_logger(), "Local obstacles received: %u points",
		static_cast<unsigned int>(lastObstacles_.size()));
}

void ReactiveNav2DNode::on_set_robot_shape(
	const geometry_msgs::msg::Polygon::SharedPtr& newShape)
{
	RCLCPP_INFO_STREAM(
		this->get_logger(),
		"[onSetRobotShape] Robot shape received via topic:");
	for (const auto& point : newShape->points)
	{
		RCLCPP_INFO_STREAM(
			this->get_logger(), "Point - x: " << point.x << ", y: " << point.y
											  << ", z: " << point.z);
	}

	mrpt::math::CPolygon poly;
	poly.resize(newShape->points.size());
	for (size_t i = 0; i < newShape->points.size(); i++)
	{
		poly[i].x = newShape->points[i].x;
		poly[i].y = newShape->points[i].y;
	}

	{
		std::lock_guard<std::mutex> csl(rnavEngineMtx_);
		rnavEngine_.changeRobotShape(poly);
	}
}

bool ReactiveNav2DNode::waitForTransform(
	mrpt::poses::CPose3D& des, const std::string& target_frame,
	const std::string& source_frame, const int timeoutMilliseconds)
{
	const rclcpp::Duration timeout(0, 1000 * timeoutMilliseconds);
	try
	{
		geometry_msgs::msg::TransformStamped ref_to_trgFrame =
			tfBuffer_->lookupTransform(
				target_frame, source_frame, tf2::TimePointZero,
				tf2::durationFromSec(timeout.seconds()));

		tf2::Transform tf;
		tf2::fromMsg(ref_to_trgFrame.transform, tf);
		des = mrpt::ros2bridge::fromROS(tf);

		RCLCPP_DEBUG(
			get_logger(), "[waitForTransform] Found pose %s -> %s: %s",
			source_frame.c_str(), target_frame.c_str(), des.asString().c_str());

		return true;
	}
	catch (const tf2::TransformException& ex)
	{
		RCLCPP_ERROR(get_logger(), "%s", ex.what());
		return false;
	}
}

void ReactiveNav2DNode::publish_last_log_record_to_ros(
	const mrpt::nav::CLogFileRecord& lr)
{
	pubSelectedPtg_->publish(log_to_margers(lr));
}

visualization_msgs::msg::MarkerArray ReactiveNav2DNode::log_to_margers(
	const mrpt::nav::CLogFileRecord& lr)
{
	if (!lr.nPTGs || lr.nSelectedPTG < 0 ||
		lr.nSelectedPTG >= static_cast<int32_t>(lr.nPTGs))
		return visualization_msgs::msg::MarkerArray();

	visualization_msgs::msg::MarkerArray msg;

	const auto* ptg = rnavEngine_.getPTG(lr.nSelectedPTG);
	const auto& ipp = lr.infoPerPTG.at(lr.nSelectedPTG);
	const auto k = ptg->alpha2index(ipp.desiredDirection);
	const auto nSteps = ptg->getPathStepCount(k);

	const double timeToDraw = 2.0;	// [s]
	ASSERT_(ptg->getPathStepDuration() > 0);

	const auto nStepsToDraw =
		std::min<size_t>(nSteps - 1, timeToDraw / ptg->getPathStepDuration());

	msg.markers.resize(1);
	auto& m = msg.markers[0];

	m.header.frame_id = frameidRobot_;
	m.header.stamp = this->get_clock()->now();
	m.frame_locked = true;

	m.pose = mrpt::ros2bridge::toROS_Pose(mrpt::poses::CPose3D::Identity());
	m.action = visualization_msgs::msg::Marker::ADD;
	m.type = visualization_msgs::msg::Marker::LINE_STRIP;

	m.ns = "rnav.ptg";
	m.id = 0;
	m.scale.x = 0.02;  // line width
	m.scale.y = 0.02;
	m.scale.z = 0.01;

	m.points.resize(nStepsToDraw);
	m.color.a = 0.8;
	m.color.r = 1.0;
	m.color.g = 0.0;
	m.color.b = 0.0;

	for (size_t i = 0; i < nStepsToDraw; i++)
	{
		const auto p = ptg->getPathPose(k, i);

		m.points[i].x = p.x;
		m.points[i].y = p.y;
		m.points[i].z = 0;
	}

	return msg;
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

	auto node = std::make_shared<ReactiveNav2DNode>();

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;
}
