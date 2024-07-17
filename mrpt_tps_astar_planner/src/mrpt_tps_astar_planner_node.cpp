/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include <mpp/algos/CostEvaluatorCostMap.h>
#include <mpp/algos/CostEvaluatorPreferredWaypoint.h>
#include <mpp/algos/NavEngine.h>
#include <mpp/algos/TPS_Astar.h>
#include <mpp/algos/refine_trajectory.h>
#include <mpp/algos/trajectories.h>
#include <mpp/algos/viz.h>
#include <mpp/data/EnqueuedMotionCmd.h>
#include <mpp/data/MotionPrimitivesTree.h>
#include <mpp/data/PlannerOutput.h>
#include <mpp/interfaces/ObstacleSource.h>
#include <mpp/interfaces/VehicleMotionInterface.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/kinematics/CVehicleVelCmd_DiffDriven.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TTwist2D.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/ros2bridge/map.h>
#include <mrpt/ros2bridge/point_cloud2.h>
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/time.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mrpt_msgs/msg/waypoint.hpp>
#include <mrpt_msgs/msg/waypoint_sequence.hpp>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// for debugging
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/stock_objects.h>

const char* NODE_NAME = "mrpt_tps_astar_planner_node";

/**
 * The main ROS2 node class.
 */
class TPS_Astar_Planner_Node : public rclcpp::Node
{
   public:
	TPS_Astar_Planner_Node();
	virtual ~TPS_Astar_Planner_Node() = default;

   private:
	/// CTimeLogger instance for profiling
	mrpt::system::CTimeLogger profiler_;

	/// Message for waypoint sequence
	mrpt_msgs::msg::WaypointSequence wps_msg_;

	/// Navigation goal position
	mrpt::math::TPose2D nav_goal_{0, 0, 0};

	/// Navigation start position
	mrpt::math::TPose2D start_pose_{0, 0, 0};

	/// Robot velocity at start
	mrpt::math::TTwist2D start_vel_{0, 0, 0};

	/// Subscriber to Goal position
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;

	/// Mutex for gridmaps_ & obstacle_points_
	std::mutex obstacles_cs_;

	/// Subscribers to gridmaps
	struct InfoPerGridMapSource
	{
		rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub;
		mrpt::maps::COccupancyGridMap2D::Ptr grid;
		mrpt::maps::CSimplePointsMap::Ptr grid_obstacles;
	};
	std::deque<InfoPerGridMapSource> gridmaps_;

	/// Subscriber to obstacle points
	struct InfoPerPointMapSource
	{
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
		mrpt::maps::CPointsMap::Ptr obstacle_points;
	};

	std::deque<InfoPerPointMapSource> obstacle_points_;

	/// Subscriber to topic from rnav that tells to replan
	/// TODO(JL): Switch into a service!
	rclcpp::Subscription<
		geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_replan_;

	/// Publisher for waypoint sequence
	rclcpp::Publisher<mrpt_msgs::msg::WaypointSequence>::SharedPtr pub_wp_seq_;

	// tf2 buffer and listener
	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	/// Flag for MRPT GUI
	bool gui_mrpt_ = false;

	/// goal topic subscriber name
	std::string topic_goal_sub_ = "tps_astar_nav_goal";

	/// map topic subscriber name(s) (multiple if separated by ',')
	std::string topic_gridmap_sub_ = "/map";

	/// obstacles topic subscriber name(s) (multiple if separated by ',')
	std::string topic_obstacle_points_sub_ = "";

	/// topics (from topic_gridmap_sub_, topic_obstacle_points_sub_) that shall
	/// be subscribed with transient QoS (normally, all static maps) (multiple
	/// if separated by ',')
	std::string topic_static_maps_ = "/map";

	/// replan topic subscriber name
	std::string topic_replan_sub_;

	/// waypoint sequence topic publisher name
	std::string topic_wp_seq_pub_;

	/// Parameter file for PTGs
	std::string ptg_ini_file_ = "ptgs.ini";

	/// Parameters file for Costmap evaluator
	std::string costmap_params_file_ = "global-costmap-params.yaml";

	/// Parameters file for waypoints preferences
	std::string wp_params_file_ = "waypoints-params.yaml";

	/// Parameters file for planner
	std::string planner_params_file_ = "planner-params.yaml";

	double mid_waypoints_allowed_distance_ = 0.5;
	double final_waypoint_allowed_distance_ = 0.4;

	/// Pointer to MRPT 3D display window
	mrpt::gui::CDisplayWindow3D::Ptr win_3d_;

	/// Path planner algorithm
	mpp::Planner::Ptr planner_;

	/// Path planner input
	mpp::PlannerInput planner_input_;

	/// Parameters for the cost evaluator
	mpp::CostEvaluatorCostMap::Parameters costMapParams_;

	/// bool indicating path plan is done
	bool path_plan_done_ = false;

   private:
	/**
	 * @brief wait for transform between map frame and the robot frame
	 *
	 * @param des position of the robot with respect to map frame
	 * @param target_frame the robot tf frame
	 * @param source_frame the map tf frame
	 * @param timeout_milliseconds timeout for transform wait
	 *
	 * @return true if there is transform from map to the robot
	 */
	[[nodiscard]] bool wait_for_transform(
		mrpt::poses::CPose3D& des, const std::string& target_frame,
		const std::string& source_frame, const int timeout_milliseconds = 50);

	/**
	 * @brief Reads a parameter from the node's parameter server.
	 *
	 * This function attempts to retrieve parameters and assign it to class
	 * member vars.
	 */
	void read_parameters();

	/**
	 * @brief Initialize A* planner with required params
	 */
	void initialize_planner();

	/**
	 * @brief Callback function when a new goal location is received
	 * @param _goal is a PoseStamped object pointer
	 */
	void callback_goal(const geometry_msgs::msg::PoseStamped& goal);

	/**
	 * @brief Callback function when a new map is received
	 * @param _map is an occupancy grid object pointer
	 */
	void callback_map(
		const nav_msgs::msg::OccupancyGrid::SharedPtr& m,
		InfoPerGridMapSource& e);

	/**
	 * @brief Callback function to update the obstacles around the Robot in case
	 * of replan
	 * @param _pc pointcloud object pointer from sensors
	 */
	void callback_obstacles(
		const sensor_msgs::msg::PointCloud2::SharedPtr& pc,
		InfoPerPointMapSource& e);

	/**
	 * @brief Callback function to prompt for a replan
	 * @param _pose current localization location of the robot on the map
	 */
	void callback_replan(
		const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& _pose);

	/**
	 * @brief Mutex locked method to update the map when new one is received
	 * @param _map is an occupancy grid object pointer
	 */
	void update_map(
		const nav_msgs::msg::OccupancyGrid::SharedPtr& _msg,
		InfoPerGridMapSource& e);

	/**
	 * @brief Mutex locked method to update local obstacle map
	 * @param _pc PointCloud2 object
	 */
	void update_obstacles(
		const sensor_msgs::msg::PointCloud2::SharedPtr& _pc,
		InfoPerPointMapSource& e);

	/**
	 * @brief Method to perform the path plan
	 * @param start robot initial pose
	 * @param goal  robot goal pose
	 * @return true if path plan was successful false if path plan failed
	 */
	bool do_path_plan(mrpt::math::TPose2D& start, mrpt::math::TPose2D& goal);

	/**
	 * @brief Debug method to visualize the planning
	 */
	void init_3d_debug();

	/**
	 * @brief Publisher method to publish waypoint sequence
	 * @param wps Waypoint sequence object
	 */
	void publish_waypoint_sequence(const mrpt_msgs::msg::WaypointSequence& wps);
};

TPS_Astar_Planner_Node::TPS_Astar_Planner_Node() : rclcpp::Node(NODE_NAME)
{
	const auto qos = rclcpp::SystemDefaultsQoS();
	// See: REP-2003: https://ros.org/reps/rep-2003.html
	const auto mapQoS =
		rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

	read_parameters();

	// Create the tf2 buffer and listener
	// ----------------------------------------

	tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	// Init ROS subs:
	// -----------------------
	sub_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
		topic_goal_sub_, qos,
		[this](const geometry_msgs::msg::PoseStamped& msg)
		{ this->callback_goal(msg); });

	// parse lists:
	std::vector<std::string> vecStaticTopics;
	mrpt::system::tokenize(topic_static_maps_, ", \t\r\n", vecStaticTopics);
	// more convenient as a set:
	std::set<std::string> staticTopics;
	for (const auto& s : vecStaticTopics) staticTopics.insert(s);

	std::vector<std::string> lstGridTopics;
	mrpt::system::tokenize(topic_gridmap_sub_, ", \t\r\n", lstGridTopics);

	// Do not assert for non-empty lists of obstacles.
	// It might be that someone wants to plan paths without obstacles?
	auto lck = mrpt::lockHelper(obstacles_cs_);
	for (const auto& topic : lstGridTopics)
	{
		auto& e = gridmaps_.emplace_back();
		const bool isStatic = staticTopics.count(topic) != 0;

		e.sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
			topic, isStatic ? mapQoS : qos,
			[this, &e](const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
			{ this->callback_map(msg, e); });
	}

	std::vector<std::string> lstPointTopics;
	mrpt::system::tokenize(
		topic_obstacle_points_sub_, ", \t\r\n", lstPointTopics);
	for (const auto& topic : lstPointTopics)
	{
		auto& e = obstacle_points_.emplace_back();
		const bool isStatic = staticTopics.count(topic) != 0;

		e.sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
			topic, isStatic ? mapQoS : qos,
			[this, &e](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
			{ this->callback_obstacles(msg, e); });
	}

	sub_replan_ = this->create_subscription<
		geometry_msgs::msg::PoseWithCovarianceStamped>(
		topic_replan_sub_, qos,
		[this](
			const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
		{ this->callback_replan(msg); });

	// Init ROS publishers:
	// -----------------------

	pub_wp_seq_ = this->create_publisher<mrpt_msgs::msg::WaypointSequence>(
		topic_wp_seq_pub_, qos);

	initialize_planner();
}

bool TPS_Astar_Planner_Node::wait_for_transform(
	mrpt::poses::CPose3D& des, const std::string& target_frame,
	const std::string& source_frame, const int timeout_milliseconds)

{
	const rclcpp::Duration timeout(0, 1000 * timeout_milliseconds);
	try
	{
		geometry_msgs::msg::TransformStamped src_to_trg_frame =
			tf_buffer_->lookupTransform(
				source_frame, target_frame, tf2::TimePointZero,
				tf2::durationFromSec(timeout.seconds()));

		tf2::Transform tf;
		tf2::fromMsg(src_to_trg_frame.transform, tf);
		des = mrpt::ros2bridge::fromROS(tf);

		RCLCPP_DEBUG(
			get_logger(), "[wait_for_transform] Found pose %s -> %s: %s",
			source_frame.c_str(), target_frame.c_str(), des.asString().c_str());

		return true;
	}
	catch (const tf2::TransformException& ex)
	{
		RCLCPP_ERROR(get_logger(), "[wait_for_transform] %s", ex.what());
		return false;
	}
}

void TPS_Astar_Planner_Node::read_parameters()
{
	this->declare_parameter<bool>("show_gui", false);
	this->get_parameter("show_gui", gui_mrpt_);
	RCLCPP_INFO(
		this->get_logger(), "MRPT GUI Enabled: %s",
		gui_mrpt_ ? "true" : "false");

	this->declare_parameter<std::string>("topic_goal_sub", topic_goal_sub_);
	this->get_parameter("topic_goal_sub", topic_goal_sub_);
	RCLCPP_INFO(
		this->get_logger(), "topic_goal_sub %s", topic_goal_sub_.c_str());

	this->declare_parameter<std::string>(
		"topic_obstacles_gridmap_sub", topic_gridmap_sub_);
	this->get_parameter("topic_obstacles_gridmap_sub", topic_gridmap_sub_);
	RCLCPP_INFO(
		this->get_logger(), "topic_obstacles_gridmap_sub: %s",
		topic_gridmap_sub_.c_str());

	this->declare_parameter<std::string>(
		"topic_obstacles_sub", topic_obstacle_points_sub_);
	this->get_parameter("topic_obstacles_sub", topic_obstacle_points_sub_);
	RCLCPP_INFO(
		this->get_logger(), "topic_obstacles_sub: %s",
		topic_obstacle_points_sub_.c_str());

	this->declare_parameter<std::string>(
		"topic_static_maps", topic_static_maps_);
	this->get_parameter("topic_static_maps", topic_static_maps_);
	RCLCPP_INFO(
		this->get_logger(), "topic_static_maps: %s",
		topic_static_maps_.c_str());

	this->declare_parameter<std::string>("topic_replan_sub", "/replan");
	this->get_parameter("topic_replan_sub", topic_replan_sub_);
	RCLCPP_INFO(
		this->get_logger(), "topic_replan_sub %s", topic_replan_sub_.c_str());

	this->declare_parameter<std::string>("topic_wp_seq_pub", "/waypoints");
	this->get_parameter("topic_wp_seq_pub", topic_wp_seq_pub_);
	RCLCPP_INFO(
		this->get_logger(), "topic_wp_seq_pub%s", topic_wp_seq_pub_.c_str());

	this->declare_parameter<std::string>("ptg_ini", ptg_ini_file_);
	this->get_parameter("ptg_ini", ptg_ini_file_);
	RCLCPP_INFO(this->get_logger(), "ptg_ini_file %s", ptg_ini_file_.c_str());

	ASSERT_FILE_EXISTS_(ptg_ini_file_);

	this->declare_parameter<std::string>(
		"global_costmap_parameters", costmap_params_file_);
	this->get_parameter("global_costmap_parameters", costmap_params_file_);
	RCLCPP_INFO(
		this->get_logger(), "global_costmap_params_file %s",
		costmap_params_file_.c_str());

	ASSERT_FILE_EXISTS_(costmap_params_file_);

	this->declare_parameter<std::string>(
		"prefer_waypoints_parameters", wp_params_file_);
	this->get_parameter("prefer_waypoints_parameters", wp_params_file_);
	RCLCPP_INFO(
		this->get_logger(), "prefer_waypoints_parameters_file %s",
		wp_params_file_.c_str());

	ASSERT_FILE_EXISTS_(wp_params_file_);

	this->declare_parameter<std::string>(
		"planner_parameters", planner_params_file_);
	this->get_parameter("planner_parameters", planner_params_file_);
	RCLCPP_INFO(
		this->get_logger(), "planner_parameters_file %s",
		planner_params_file_.c_str());

	ASSERT_FILE_EXISTS_(planner_params_file_);

	this->declare_parameter<double>(
		"mid_waypoints_allowed_distance", mid_waypoints_allowed_distance_);
	this->get_parameter(
		"mid_waypoints_allowed_distance", mid_waypoints_allowed_distance_);

	this->declare_parameter<double>(
		"final_waypoint_allowed_distance", final_waypoint_allowed_distance_);
	this->get_parameter(
		"final_waypoint_allowed_distance", final_waypoint_allowed_distance_);
}

void TPS_Astar_Planner_Node::initialize_planner()
{
	planner_ = mpp::TPS_Astar::Create();

	// Enable time profiler:
	planner_->profiler_().enable(true);

	const auto c = mrpt::containers::yaml::FromFile(planner_params_file_);
	planner_->params_from_yaml(c);
	RCLCPP_INFO_STREAM(
		this->get_logger(),
		"Loaded these planner params:" << planner_->params_as_yaml());

	mrpt::config::CConfigFile cfg(ptg_ini_file_);
	planner_input_.ptgs.initFromConfigFile(cfg, "SelfDriving");

	costMapParams_ = mpp::CostEvaluatorCostMap::Parameters::FromYAML(
		mrpt::containers::yaml::FromFile(costmap_params_file_));
}

void TPS_Astar_Planner_Node::callback_goal(
	const geometry_msgs::msg::PoseStamped& _goal)
{
	try
	{
		const auto p = mrpt::ros2bridge::fromROS(_goal.pose);
		nav_goal_ = mrpt::math::TPose2D(p.asTPose());

		mrpt::poses::CPose3D robot_pose;
		const bool robot_pose_ok =
			wait_for_transform(robot_pose, "base_link", "map");

		ASSERT_(robot_pose_ok);

		start_pose_.x = robot_pose.x();
		start_pose_.y = robot_pose.y();
		start_pose_.phi = robot_pose.yaw();

		path_plan_done_ = do_path_plan(start_pose_, nav_goal_);
	}
	catch (const std::exception& e)
	{
		RCLCPP_ERROR(
			this->get_logger(), "Exception in goal callback : %s", e.what());
	}
}

void TPS_Astar_Planner_Node::callback_map(
	const nav_msgs::msg::OccupancyGrid::SharedPtr& grid,
	TPS_Astar_Planner_Node::InfoPerGridMapSource& e)
{
	RCLCPP_INFO_STREAM(
		this->get_logger(),
		"Received gridmap from topic: " << e.sub->get_topic_name());

	update_map(grid, e);
}

// Add current obstacles to make better plan
void TPS_Astar_Planner_Node::callback_replan(
	const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& msg)
{
	try
	{
		if (!msg)
		{
			RCLCPP_ERROR(
				this->get_logger(),
				"Received null ptr as pose in replan callback");
			return;
		}

		tf2::Quaternion quat(
			msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
			msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
		tf2::Matrix3x3 mat(quat);
		double roll, pitch, yaw;
		mat.getRPY(roll, pitch, yaw);

		start_pose_.x = msg->pose.pose.position.x;
		start_pose_.y = msg->pose.pose.position.y;
		start_pose_.phi = yaw;

		path_plan_done_ = do_path_plan(start_pose_, nav_goal_);
	}
	catch (std::exception& e)
	{
		RCLCPP_ERROR(
			this->get_logger(), "Exception in replan callback : %s", e.what());
	}
}

void TPS_Astar_Planner_Node::callback_obstacles(
	const sensor_msgs::msg::PointCloud2::SharedPtr& pc,
	InfoPerPointMapSource& e)
{
	RCLCPP_INFO_STREAM(
		this->get_logger(),
		"Received obstacle points from topic: " << e.sub->get_topic_name());

	update_obstacles(pc, e);
}

void TPS_Astar_Planner_Node::update_obstacles(
	const sensor_msgs::msg::PointCloud2::SharedPtr& pcMsg,
	InfoPerPointMapSource& e)
{
	auto lck = mrpt::lockHelper(obstacles_cs_);

	auto pc = mrpt::maps::CSimplePointsMap::Create();
	if (!mrpt::ros2bridge::fromROS(*pcMsg, *pc))
	{
		RCLCPP_ERROR(
			this->get_logger(),
			"Failed to convert Point Cloud to MRPT Points Map");
	}
	e.obstacle_points = pc;
}

void TPS_Astar_Planner_Node::publish_waypoint_sequence(
	const mrpt_msgs::msg::WaypointSequence& wps)
{
	pub_wp_seq_->publish(wps);
}

void TPS_Astar_Planner_Node::init_3d_debug()
{
	if (win_3d_) return;

	win_3d_ = mrpt::gui::CDisplayWindow3D::Create(
		"Pathplanning-TPS-AStar", 1000, 600);
	win_3d_->setCameraZoom(20);
	win_3d_->setCameraAzimuthDeg(-45);

	auto scene = win_3d_->get3DSceneAndLock();

	auto lck = mrpt::lockHelper(obstacles_cs_);

	for (const auto& e : gridmaps_) scene->insert(e.grid->getVisualization());

	for (const auto& e : obstacle_points_)
		scene->insert(e.obstacle_points->getVisualization());

	lck.unlock();

	scene->enableFollowCamera(true);

	win_3d_->unlockAccess3DScene();
}

void TPS_Astar_Planner_Node::update_map(
	const nav_msgs::msg::OccupancyGrid::SharedPtr& msg, InfoPerGridMapSource& e)
{
	auto lck = mrpt::lockHelper(obstacles_cs_);

	e.grid = mrpt::maps::COccupancyGridMap2D::Create();
	mrpt::ros2bridge::fromROS(*msg, *e.grid);

	e.grid_obstacles = mrpt::maps::CSimplePointsMap::Create();
	e.grid->getAsPointCloud(*e.grid_obstacles);
}

bool TPS_Astar_Planner_Node::do_path_plan(
	mrpt::math::TPose2D& start, mrpt::math::TPose2D& goal)
{
	RCLCPP_INFO_STREAM(this->get_logger(), "Do path planning");

	auto bbox = mrpt::math::TBoundingBoxf::PlusMinusInfinity();

	// Start => Goal
	// --------------------------------------------------------------
	planner_input_.stateStart.pose = start;
	planner_input_.stateStart.vel = start_vel_;
	planner_input_.stateGoal.state = goal;

	bbox.updateWithPoint(mrpt::math::TPoint3D(start.translation()));
	bbox.updateWithPoint(mrpt::math::TPoint3D(goal.translation()));

	// Create obstacle sources, and find out bounding box:
	// --------------------------------------------------------------
	auto lckObs = mrpt::lockHelper(obstacles_cs_);

	planner_->costEvaluators_.clear();
	planner_input_.obstacles.clear();

	size_t obstacleSources = 0, totalObstaclePoints = 0;
	// gridmaps:
	for (const auto& e : gridmaps_)
	{
		auto obs = mpp::ObstacleSource::FromStaticPointcloud(e.grid_obstacles);
		planner_input_.obstacles.emplace_back(obs);

		auto bb = e.grid_obstacles->boundingBox();
		bbox = bbox.unionWith(bb);

		obstacleSources++;
		totalObstaclePoints += e.grid_obstacles->size();

		auto costmap = mpp::CostEvaluatorCostMap::FromStaticPointObstacles(
			*e.grid_obstacles, costMapParams_, planner_input_.stateStart.pose);
		planner_->costEvaluators_.push_back(costmap);
	}
	// points:
	for (const auto& e : obstacle_points_)
	{
		auto obs = mpp::ObstacleSource::FromStaticPointcloud(e.obstacle_points);
		planner_input_.obstacles.emplace_back(obs);

		auto bb = obs->obstacles()->boundingBox();
		bbox = bbox.unionWith(bb);

		obstacleSources++;
		totalObstaclePoints += e.obstacle_points->size();

		auto costmap = mpp::CostEvaluatorCostMap::FromStaticPointObstacles(
			*e.obstacle_points, costMapParams_, planner_input_.stateStart.pose);
		planner_->costEvaluators_.push_back(costmap);
	}

	lckObs.unlock();

	{
		const auto bboxMargin = mrpt::math::TPoint3Df(2.0, 2.0, .0);
		const auto ptStart = mrpt::math::TPoint3Df(
			planner_input_.stateStart.pose.x, planner_input_.stateStart.pose.y,
			0);
		const auto ptGoal = mrpt::math::TPoint3Df(
			planner_input_.stateGoal.asSE2KinState().pose.x,
			planner_input_.stateGoal.asSE2KinState().pose.y, 0);
		bbox.updateWithPoint(ptStart - bboxMargin);
		bbox.updateWithPoint(ptStart + bboxMargin);
		bbox.updateWithPoint(ptGoal - bboxMargin);
		bbox.updateWithPoint(ptGoal + bboxMargin);
	}

	planner_input_.worldBboxMax = {bbox.max.x, bbox.max.y, M_PI};
	planner_input_.worldBboxMin = {bbox.min.x, bbox.min.y, -M_PI};

	RCLCPP_INFO_STREAM(
		this->get_logger(),
		"Start state: " << planner_input_.stateStart.asString()
						<< "\n Goal state: "
						<< planner_input_.stateGoal.asString()
						<< "\n Obstacle sources: " << obstacleSources
						<< "\n Total obstacle points: " << totalObstaclePoints
						<< "\n  World bbox : "
						<< planner_input_.worldBboxMin.asString() << "-"
						<< planner_input_.worldBboxMax.asString());

	// Insert custom progress callback:
	planner_->progressCallback_ = [](const mpp::ProgressCallbackData& pcd)
	{
		std::cout << "[progressCallback] bestCostFromStart: "
				  << pcd.bestCostFromStart
				  << " bestCostToGoal: " << pcd.bestCostToGoal
				  << " bestPathLength: " << pcd.bestPath.size() << std::endl;
	};

	const mpp::PlannerOutput plan = planner_->plan(planner_input_);

	std::cout << "\nDone.\n";
	std::cout << "Success: " << (plan.success ? "YES" : "NO") << "\n";
	std::cout << "Plan has " << plan.motionTree.edges_to_children.size()
			  << " overall edges, " << plan.motionTree.nodes().size()
			  << " nodes\n";

	if (!plan.bestNodeId.has_value())
	{
		std::cerr << "No bestNodeId in plan output.\n";
		return false;
	}

	// if (plan.success) { activePlanOutput_ = plan; }

	if (!plan.success)
	{
		planner_->costEvaluators_.clear();
	}

	// backtrack:
	auto [plannedPath, pathEdges] =
		plan.motionTree.backtrack_path(*plan.bestNodeId);

#if 0  // JLBC: disabled to check if this is causing troubles
mpp::refine_trajectory(plannedPath, pathEdges, planner_input.ptgs);
#endif

	// Show plan in a GUI for debugging
	if (plan.success && gui_mrpt_)
	{
		mpp::VisualizationOptions vizOpts;

		vizOpts.renderOptions.highlight_path_to_node_id = plan.bestNodeId;
		vizOpts.renderOptions.color_normal_edge = {0xb0b0b0, 0x20};	 // RGBA
		vizOpts.renderOptions.width_normal_edge =
			0;	// hide all edges except best path
		vizOpts.gui_modal = false;	// leave GUI open in a background thread

		mpp::viz_nav_plan(plan, vizOpts, planner_->costEvaluators_);
	}

	// Interpolate so we have many waypoints:
	mpp::trajectory_t interpPath;
	if (plan.success)
	{
		const double interpPeriod = 0.25;  // [s]

		interpPath = mpp::plan_to_trajectory(
			pathEdges, planner_input_.ptgs, interpPeriod);

		// Note: trajectory is in local frame of reference
		// of plan.originalInput.stateStart.pose
		// so, correct that relative pose so we keep everything in global
		// frame:
		const auto& startPose = plan.originalInput.stateStart.pose;
		for (auto& kv : interpPath)
			kv.second.state.pose = startPose + kv.second.state.pose;
	}

	wps_msg_ = mrpt_msgs::msg::WaypointSequence();
	if (plan.success)
	{
		for (auto& kv : interpPath)
		{
			const auto& goal_state = kv.second.state;
			std::cout << "Waypoint: x = " << goal_state.pose.x
					  << ", y= " << goal_state.pose.y << std::endl;
			auto wp_msg = mrpt_msgs::msg::Waypoint();
			wp_msg.target = mrpt::ros2bridge::toROS_Pose(goal_state.pose);

			wp_msg.allowed_distance = mid_waypoints_allowed_distance_;
			wp_msg.allow_skip = true;

			wps_msg_.waypoints.push_back(wp_msg);
		}

		auto wp_msg = mrpt_msgs::msg::Waypoint();
		wp_msg.target = mrpt::ros2bridge::toROS_Pose(nav_goal_);

		wp_msg.allowed_distance = final_waypoint_allowed_distance_;
		wp_msg.allow_skip = false;

		wps_msg_.waypoints.push_back(wp_msg);
	}

	publish_waypoint_sequence(wps_msg_);

	return plan.success;
}

// ------------------------------------
int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<TPS_Astar_Planner_Node>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
