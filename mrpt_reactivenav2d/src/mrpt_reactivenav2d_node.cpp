#include <cassert>
#include <stdexcept>
#include "mrpt_reactivenav2d/mrpt_reactivenav2d_node.hpp"

using namespace mrpt::nav;
using mrpt::maps::CSimplePointsMap;
using namespace mrpt::system;
using namespace mrpt::config;

/**  Constructor: Inits ROS system */
ReactiveNav2DNode::ReactiveNav2DNode(const rclcpp::NodeOptions& options)
: Node("mrpt_reactivenav2d", options)
, m_1st_time_init(false)
, m_target_allowed_distance(0.40f)
, m_nav_period(0.100)
, m_save_nav_log(false)
, m_reactive_if(*this)
, m_reactive_nav_engine(m_reactive_if)
{
	// Load params
	read_parameters();

	assert(m_nav_period > 0);

	if (m_cfg_file_reactive.empty()) 
	{
		RCLCPP_ERROR(this->get_logger(), 
		"Mandatory param 'cfg_file_reactive' is missing!");
		throw;
	}

	if (!mrpt::system::fileExists(m_cfg_file_reactive)) 
	{
		RCLCPP_ERROR(this->get_logger(),
			"Config file not found: %s", m_cfg_file_reactive.c_str());
		throw;
	}

	m_reactive_nav_engine.enableLogFile(m_save_nav_log);

	// Load reactive config:
	// ----------------------------------------------------
	if(!m_cfg_file_reactive.empty())
	{
		try
		{
			CConfigFile cfgFil(m_cfg_file_reactive);
			m_reactive_nav_engine.loadConfigFile(cfgFil);
		}
		catch (std::exception& e)
		{
			RCLCPP_ERROR(this->get_logger(),
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
	if (!m_sub_topic_robot_shape.empty())
	{
		m_sub_robot_shape = this->create_subscription<geometry_msgs::msg::Polygon>(
			m_sub_topic_robot_shape, 1,
			[this](const geometry_msgs::msg::Polygon::SharedPtr poly)
			{this->on_set_robot_shape(poly);});

		RCLCPP_INFO(this->get_logger(),
			"Params say robot shape will arrive via topic '%s'... waiting 3 seconds for it.",
			m_sub_topic_robot_shape.c_str());

		// Use rate object to implement sleep
		rclcpp::Rate rate(1); // 1 Hz
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
		CConfigFile c(m_cfg_file_reactive);
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

			std::lock_guard<std::mutex> csl(m_reactive_nav_engine_cs);
			m_reactive_nav_engine.changeRobotShape(poly);
		}

		// Load robot shape: 2/2 circle
		// ---------------------------------------------
		if (const double robot_radius = c.read_double(
				s, "RobotModel_circular_shape_radius", -1.0, false);
			robot_radius > 0)
		{
			std::lock_guard<std::mutex> csl(m_reactive_nav_engine_cs);
			m_reactive_nav_engine.changeRobotCircularShapeRadius(
				robot_radius);
		}
	}

	// Init ROS publishers:
	// -----------------------
	m_pub_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>(m_pub_topic_cmd_vel, 1);

	// Init ROS subs:
	// -----------------------
	m_sub_odometry = this->create_subscription<nav_msgs::msg::Odometry>(
						m_sub_topic_odometry, 1, 
						[this](const nav_msgs::msg::Odometry::SharedPtr odom)
						{this->on_odometry_received(odom);});

	m_sub_wp_seq = this->create_subscription<mrpt_msgs::msg::WaypointSequence>(
						m_sub_topic_wp_seq, 1, 
						[this](const mrpt_msgs::msg::WaypointSequence::SharedPtr msg)
						{this->on_waypoint_seq_received(msg);});

	m_sub_nav_goal = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    					m_sub_topic_reactive_nav_goal, 1, 
    					[this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
						{ this->on_goal_received(msg); });

	m_sub_local_obs = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    					m_sub_topic_local_obstacles, 1, 
    					[this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) 
						{ this->on_local_obstacles(msg); });

	// Init tf buffers
	// ----------------------------------------------------
	m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  	m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

	// Init timer:
	// ----------------------------------------------------
  	m_timer_run_nav = this->create_wall_timer(
    					std::chrono::duration<double>(m_nav_period),
    					[this]() { this->on_do_navigation(); });
	

}  // end ctor

void ReactiveNav2DNode::read_parameters()
{
	this->declare_parameter<std::string>("cfg_file_reactive", "reactive2d_config.ini");
  	this->get_parameter("cfg_file_reactive", m_cfg_file_reactive);
  	RCLCPP_INFO(this->get_logger(), "cfg_file_reactive %s", m_cfg_file_reactive.c_str());

	this->declare_parameter<double>("target_allowed_distance", 0.40);
  	this->get_parameter("target_allowed_distance", m_target_allowed_distance);
  	RCLCPP_INFO(this->get_logger(), "target_allowed_distance: %f", m_target_allowed_distance);

	this->declare_parameter<double>("nav_period", 0.10);
  	this->get_parameter("nav_period", m_nav_period);
  	RCLCPP_INFO(this->get_logger(), "nav_period: %f", m_nav_period);

	this->declare_parameter<std::string>("frameid_reference", "odom");
	this->get_parameter("frameid_reference", m_frameid_reference);
	RCLCPP_INFO(this->get_logger(), "frameid_reference: %s", m_frameid_reference.c_str());

  	this->declare_parameter<std::string>("frameid_robot", "base_link");
  	this->get_parameter("frameid_robot", m_frameid_robot);
  	RCLCPP_INFO(this->get_logger(), "frameid_robot: %s", m_frameid_robot.c_str());

	this->declare_parameter<std::string>("topic_wp_seq", "waypointsequence");
  	this->get_parameter("topic_wp_seq", m_sub_topic_wp_seq);
  	RCLCPP_INFO(this->get_logger(), "topic_wp_seq: %s", m_sub_topic_wp_seq.c_str());

	this->declare_parameter<std::string>("topic_odometry", "/odometry");
  	this->get_parameter("topic_odometry", m_sub_topic_odometry);
  	RCLCPP_INFO(this->get_logger(), "topic_odometry: %s", m_sub_topic_odometry.c_str());

	this->declare_parameter<std::string>("topic_cmd_vel", "/cmd_vel");
  	this->get_parameter("topic_cmd_vel", m_pub_topic_cmd_vel);
  	RCLCPP_INFO(this->get_logger(), "topic_cmd_vel: %s", m_pub_topic_cmd_vel.c_str());

	this->declare_parameter<std::string>("topic_obstacles", "/pointcloud");
  	this->get_parameter("topic_obstacles", m_sub_topic_local_obstacles);
  	RCLCPP_INFO(this->get_logger(), "topic_obstacles: %s", m_sub_topic_local_obstacles.c_str());

	this->declare_parameter<bool>("save_nav_log", false);
  	this->get_parameter("save_nav_log", m_save_nav_log);
  	RCLCPP_INFO(this->get_logger(), "save_nav_log: %b", m_save_nav_log);

	this->declare_parameter<std::string>("ptg_plugin_files", "");
  	this->get_parameter("ptg_plugin_files", m_plugin_file);
  	RCLCPP_INFO(this->get_logger(), "ptg_plugin_files: %s", m_plugin_file.c_str());

	if (!m_plugin_file.empty())
	{
		RCLCPP_INFO_STREAM( this->get_logger(), 
				"About to load plugins: " << m_plugin_file);
		std::string errorMsgs;
		if (!mrpt::system::loadPluginModules(m_plugin_file, errorMsgs))
		{
			RCLCPP_ERROR_STREAM(this->get_logger(), "Error loading rnav plugins: " << errorMsgs);
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
	RCLCPP_INFO(this->get_logger(), 
		"[navigateTo] Starting navigation to %s",
		target.asString().c_str());

	CAbstractPTGBasedReactive::TNavigationParamsPTG navParams;

	CAbstractNavigator::TargetInfo target_info;
	target_info.target_coords.x = target.x;
	target_info.target_coords.y = target.y;
	target_info.targetAllowedDistance = m_target_allowed_distance;
	target_info.targetIsRelative = false;

	// API for single targets:
	navParams.target = target_info;

	// Optional: restrict the PTGs to use
	// navParams.restrict_PTG_indices.push_back(1);

	{
		std::lock_guard<std::mutex> csl(m_reactive_nav_engine_cs);
		m_reactive_nav_engine.navigate(&navParams);
	}
}

/** Callback: On run navigation */
void ReactiveNav2DNode::on_do_navigation()
{
	// 1st time init:
	// ----------------------------------------------------
	if (!m_1st_time_init)
	{
		m_1st_time_init = true;
		RCLCPP_INFO( this->get_logger(),
			"[ReactiveNav2DNode] Initializing reactive navigation "
			"engine...");
		{
			std::lock_guard<std::mutex> csl(m_reactive_nav_engine_cs);
			m_reactive_nav_engine.initialize();
		}
		RCLCPP_INFO(this->get_logger(),
			"[ReactiveNav2DNode] Reactive navigation engine init done!");
	}

	CTimeLoggerEntry tle(m_profiler, "on_do_navigation");
	// Main nav loop (in whatever state nav is: IDLE, NAVIGATING, etc.)
	m_reactive_nav_engine.navigationStep();
}

void ReactiveNav2DNode::on_odometry_received(const nav_msgs::msg::Odometry::SharedPtr& msg)
{
	std::lock_guard<std::mutex> csl(m_odometry_cs);
	tf2::Quaternion quat(
		msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	tf2::Matrix3x3 mat(quat);
	double roll, pitch, yaw;
	mat.getRPY(roll, pitch, yaw);
	m_odometry.odometry = mrpt::poses::CPose2D(
		msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);

	m_odometry.velocityLocal.vx = msg->twist.twist.linear.x;
	m_odometry.velocityLocal.vy = msg->twist.twist.linear.y;
	m_odometry.velocityLocal.omega = msg->twist.twist.angular.z;
	m_odometry.hasVelocities = true;

	RCLCPP_DEBUG_STREAM(this->get_logger(), "Odometry updated");
}

void ReactiveNav2DNode::on_waypoint_seq_received(const mrpt_msgs::msg::WaypointSequence::SharedPtr& wps)
{
	update_waypoint_sequence(std::move(wps));
}

void ReactiveNav2DNode::update_waypoint_sequence(const mrpt_msgs::msg::WaypointSequence::SharedPtr& msg)
{
	mrpt::nav::TWaypointSequence wps;

	for (const auto& wp : msg->waypoints)
	{
		tf2::Quaternion quat(
			wp.target.orientation.x, wp.target.orientation.y,
			wp.target.orientation.z, wp.target.orientation.w);
		tf2::Matrix3x3 mat(quat);
		double roll, pitch, yaw;
		mat.getRPY(roll, pitch, yaw);
		auto waypoint = mrpt::nav::TWaypoint(
			wp.target.position.x, wp.target.position.y, wp.allowed_distance,
			wp.allow_skip);

		if (yaw == yaw && !wp.ignore_heading)  // regular number, not NAN
			waypoint.target_heading = yaw;

		wps.waypoints.push_back(waypoint);
	}

	RCLCPP_INFO_STREAM(this->get_logger(), "New navigateWaypoints() command");
	{
		std::lock_guard<std::mutex> csl(m_reactive_nav_engine_cs);
		m_reactive_nav_engine.navigateWaypoints(wps);
	}
}

void ReactiveNav2DNode::on_goal_received(const geometry_msgs::msg::PoseStamped::SharedPtr& trg_ptr)
{
	geometry_msgs::msg::PoseStamped trg = *trg_ptr;

	RCLCPP_INFO(this->get_logger(),
		"Nav target received via topic sub: (%.03f,%.03f, %.03fdeg) "
		"[frame_id=%s]",
		trg.pose.position.x, trg.pose.position.y,
		trg.pose.orientation.z * 180.0 / M_PI, trg.header.frame_id.c_str());

	// Convert to the "m_frameid_reference" frame of coordinates:
	if (trg.header.frame_id != m_frameid_reference)
	{
		rclcpp::Duration timeout(0.2);
		try
		{
			geometry_msgs::msg::TransformStamped ref_to_trgFrame =
				m_tf_buffer->lookupTransform(
					trg.header.frame_id, m_frameid_reference,
					tf2::TimePointZero, tf2::durationFromSec(timeout.seconds()));

			tf2::doTransform(trg, trg, ref_to_trgFrame);
		}
		catch (const tf2::TransformException& ex)
		{
			RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
			return;
		}
	}

	this->navigate_to(mrpt::math::TPose2D(
		trg.pose.position.x, trg.pose.position.y, trg.pose.orientation.z));
}

void ReactiveNav2DNode::on_local_obstacles(const sensor_msgs::msg::PointCloud2::SharedPtr& obs)
{
	std::lock_guard<std::mutex> csl(m_last_obstacles_cs);
	mrpt::ros2bridge::fromROS(*obs, m_last_obstacles);
	RCLCPP_DEBUG(this->get_logger(), 
		"Local obstacles received: %u points", static_cast<unsigned
		int>(m_last_obstacles.size()));
}

void ReactiveNav2DNode::on_set_robot_shape(const geometry_msgs::msg::Polygon::SharedPtr& newShape)
{
	RCLCPP_INFO_STREAM(this->get_logger(), "[onSetRobotShape] Robot shape received via topic:");
    for (const auto& point : newShape->points)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Point - x: " << point.x << ", y: " << point.y << ", z: " << point.z);
    }

	mrpt::math::CPolygon poly;
	poly.resize(newShape->points.size());
	for (size_t i = 0; i < newShape->points.size(); i++)
	{
		poly[i].x = newShape->points[i].x;
		poly[i].y = newShape->points[i].y;
	}

	{
		std::lock_guard<std::mutex> csl(m_reactive_nav_engine_cs);
		m_reactive_nav_engine.changeRobotShape(poly);
	}
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ReactiveNav2DNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}