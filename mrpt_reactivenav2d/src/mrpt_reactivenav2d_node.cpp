#include "mrpt_reactivenav2d/mrpt_reactivenav2d_node.hpp"


/**  Constructor: Inits ROS system */
ReactiveNav2DNode::ReactiveNav2DNode(int argc, char** argv)
	: m_auxinit(argc, argv),
		m_nh(),
		m_localn("~"),
		m_1st_time_init(false),
		m_target_allowed_distance(0.40f),
		m_nav_period(0.100),
		m_pub_topic_reactive_nav_goal("reactive_nav_goal"),
		m_sub_topic_local_obstacles("local_map_pointcloud"),
		m_sub_topic_robot_shape(""),
		m_frameid_reference("map"),
		m_frameid_robot("base_link"),
		m_save_nav_log(false),
		m_reactive_if(*this),
		m_reactive_nav_engine(m_reactive_if)
{
	// Load params:
	std::string cfg_file_reactive;
	m_localn.param(
		"cfg_file_reactive", cfg_file_reactive, cfg_file_reactive);
	m_localn.param(
		"target_allowed_distance", m_target_allowed_distance,
		m_target_allowed_distance);
	m_localn.param("nav_period", m_nav_period, m_nav_period);
	m_localn.param(
		"frameid_reference", m_frameid_reference, m_frameid_reference);
	m_localn.param("frameid_robot", m_frameid_robot, m_frameid_robot);
	m_localn.param(
		"topic_robot_shape", m_sub_topic_robot_shape,
		m_sub_topic_robot_shape);
	m_localn.param("save_nav_log", m_save_nav_log, m_save_nav_log);

	ROS_ASSERT(m_nav_period > 0);
	ROS_ASSERT_MSG(
		!cfg_file_reactive.empty(),
		"Mandatory param 'cfg_file_reactive' is missing!");
	ROS_ASSERT_MSG(
		mrpt::system::fileExists(cfg_file_reactive),
		"Config file not found: '%s'", cfg_file_reactive.c_str());

	m_reactive_nav_engine.enableLogFile(m_save_nav_log);

	// Load reactive config:
	// ----------------------------------------------------
	try
	{
		CConfigFile cfgFil(cfg_file_reactive);
		m_reactive_nav_engine.loadConfigFile(cfgFil);
	}
	catch (std::exception& e)
	{
		ROS_ERROR(
			"Exception initializing reactive navigation engine:\n%s",
			e.what());
		throw;
	}

	// load robot shape: (1) default, (2) via params, (3) via topic
	// ----------------------------------------------------------------
	// m_reactive_nav_engine.changeRobotShape();

	// Init this subscriber first so we know asap the desired robot shape,
	// if provided via a topic:
	if (!m_sub_topic_robot_shape.empty())
	{
		m_sub_robot_shape = m_nh.subscribe<geometry_msgs::Polygon>(
			m_sub_topic_robot_shape, 1,
			&ReactiveNav2DNode::onRosSetRobotShape, this);
		ROS_INFO(
			"Params say robot shape will arrive via topic '%s'... waiting "
			"3 seconds for it.",
			m_sub_topic_robot_shape.c_str());
		ros::Duration(3.0).sleep();
		for (size_t i = 0; i < 100; i++) ros::spinOnce();
		ROS_INFO("Wait done.");
	}

	// Init ROS publishers:
	// -----------------------
	m_pub_cmd_vel = m_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	// Init ROS subs:
	// -----------------------
	// "/reactive_nav_goal", "/move_base_simple/goal" (
	// geometry_msgs/PoseStamped )
	m_sub_nav_goal = m_nh.subscribe<geometry_msgs::PoseStamped>(
		m_pub_topic_reactive_nav_goal, 1,
		&ReactiveNav2DNode::onRosGoalReceived, this);
	m_sub_local_obs = m_nh.subscribe<sensor_msgs::PointCloud>(
		m_sub_topic_local_obstacles, 1,
		&ReactiveNav2DNode::onRosLocalObstacles, this);

	// Init timers:
	// ----------------------------------------------------
	m_timer_run_nav = m_nh.createTimer(
		ros::Duration(m_nav_period), &ReactiveNav2DNode::onDoNavigation,
		this);

}  // end ctor

/**
 * @brief Issue a navigation command
 * @param target The target location
 */
void ReactiveNav2DNode::navigate_to(const mrpt::math::TPose2D& target)
{
	ROS_INFO(
		"[navigateTo] Starting navigation to %s",
		target.asString().c_str());

	CAbstractPTGBasedReactive::TNavigationParamsPTG navParams;

	CAbstractNavigator::TargetInfo target_info;
	target_info.target_coords.x = target.x;
	target_info.target_coords.y = target.y;
	target_info.targetAllowedDistance = m_target_allowed_distance;
	target_info.targetIsRelative = false;

	// API for multiple waypoints:
	//...
	// navParams.multiple_targets.push_back(target_info);

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
		ROS_INFO(
			"[ReactiveNav2DNode] Initializing reactive navigation "
			"engine...");
		{
			std::lock_guard<std::mutex> csl(m_reactive_nav_engine_cs);
			m_reactive_nav_engine.initialize();
		}
		ROS_INFO(
			"[ReactiveNav2DNode] Reactive navigation engine init done!");
	}

	CTimeLoggerEntry tle(m_profiler, "onDoNavigation");

	m_reactive_nav_engine.navigationStep();
}

void ReactiveNav2DNode::on_goal_received(const geometry_msgs::msg::PoseStamped::SharedPtr& trg_ptr)
{
	geometry_msgs::PoseStamped trg = *trg_ptr;

	ROS_INFO(
		"Nav target received via topic sub: (%.03f,%.03f, %.03fdeg) "
		"[frame_id=%s]",
		trg.pose.position.x, trg.pose.position.y,
		trg.pose.orientation.z * 180.0 / M_PI, trg.header.frame_id.c_str());

	// Convert to the "m_frameid_reference" frame of coordinates:
	if (trg.header.frame_id != m_frameid_reference)
	{
		ros::Duration timeout(0.2);
		try
		{
			geometry_msgs::TransformStamped ref_to_trgFrame =
				m_tf_buffer.lookupTransform(
					trg.header.frame_id, m_frameid_reference, ros::Time(0),
					timeout);

			tf2::doTransform(trg, trg, ref_to_trgFrame);
		}
		catch (const tf2::TransformException& ex)
		{
			ROS_ERROR("%s", ex.what());
			return;
		}
	}

	this->navigateTo(mrpt::math::TPose2D(
		trg.pose.position.x, trg.pose.position.y, trg.pose.orientation.z));
}

void ReactiveNav2DNode::on_local_obstacles(const sensor_msgs::msg::PointCloud::SharedPtr& obs)
{
	std::lock_guard<std::mutex> csl(m_last_obstacles_cs);
	mrpt::ros2bridge::fromROS(*obs, m_last_obstacles);
	// ROS_DEBUG("Local obstacles received: %u points", static_cast<unsigned
	// int>(m_last_obstacles.size()) );
}

void ReactiveNav2DNode::on_set_robot_shape(const geometry_msgs::msg::Polygon::SharedPtr& newShape)
{
	ROS_INFO_STREAM(
		"[onRosSetRobotShape] Robot shape received via topic: "
		<< *newShape);

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