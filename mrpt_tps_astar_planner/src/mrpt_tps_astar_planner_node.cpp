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
#include <mrpt/ros2bridge/map.h>
#include <mrpt/ros2bridge/point_cloud2.h>
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/time.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
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

	/// Flag for performing initialization once
	std::once_flag init_flag_;

	/// Flag for receiving map once
	std::once_flag map_received_flag_;

	/// Obstacle points, in global "map" frame.
	/// (default: no obstacles)
	mrpt::maps::CPointsMap::Ptr obstacles_ =
		mrpt::maps::CSimplePointsMap::Create();

	/// Message for waypoint sequence
	mrpt_msgs::msg::WaypointSequence wps_msg_;

	/// Navigation goal position
	mrpt::math::TPose2D nav_goal_{0, 0, 0};

	/// Navigation start position
	mrpt::math::TPose2D start_pose_{0, 0, 0};

	/// Robot velocity at start
	mrpt::math::TTwist2D start_vel_{0, 0, 0};

	/// Pointer to obstacle map
	mrpt::maps::CPointsMap::Ptr obstacle_src_;

	/// Subscriber to Goal position
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;

	/// Subscriber to map
	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;

	/// Subscriber to obstacle map info
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
		sub_obstacles_;

	/// Subscriber to topic from rnav that tells to replan
	/// TODO(JL): Switch into a service!
	rclcpp::Subscription<
		geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_replan_;

	/// Publisher for waypoint sequence
	rclcpp::Publisher<mrpt_msgs::msg::WaypointSequence>::SharedPtr pub_wp_seq_;

	/// Mutex for obstacle data
	std::mutex obstacles_cs_;

	/// Flag for MRPT GUI
	bool gui_mrpt_ = false;

	/// goal topic subscriber name
	std::string topic_goal_sub_ = "tps_astar_nav_goal";

	/// map topic subscriber name
	std::string topic_obstacles_gridmap_sub_;

	/// obstacles topic subscriber name
	std::string topic_obstacles_sub_;

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

	/// MRPT OpenGL scene
	mrpt::opengl::COpenGLScene scene_;

	/// Path planner algorithm
	mpp::Planner::Ptr planner_;

	/// Path planner input
	mpp::PlannerInput planner_input_;

	/// Parameters for the cost evaluator
	mpp::CostEvaluatorCostMap::Parameters costMapParams_;

	/// Path plan cost evaluators
	std::vector<mpp::CostEvaluator::Ptr> costEvaluators_;

	/// bool indicating path plan is done
	bool path_plan_done_ = false;

   private:
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
	void callback_goal(const geometry_msgs::msg::PoseStamped& _goal);

	/**
	 * @brief Callback function when a new map is received
	 * @param _map is an occupancy grid object pointer
	 */
	void callback_map(const nav_msgs::msg::OccupancyGrid::SharedPtr& _map);

	/**
	 * @brief Callback function to update the obstacles around the Robot in case
	 * of replan
	 * @param _pc pointcloud object pointer from sensors
	 */
	void callback_obstacles(
		const sensor_msgs::msg::PointCloud2::SharedPtr& _pc);

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
	void update_map(const nav_msgs::msg::OccupancyGrid::SharedPtr& _msg);

	/**
	 * @brief Mutex locked method to update local obstacle map
	 * @param _pc PointCloud2 object
	 */
	void update_obstacles(const sensor_msgs::msg::PointCloud2::SharedPtr& _pc);

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
	 * @brief Method to check if a given pose is within map bounds
	 * @param pose mrpt pose2D
	 * @return true if given pose is within map bounds
	 */
	bool is_pose_within_map_bounds(const mrpt::math::TPose2D& pose);

	/**
	 * @brief Publisher method to publish waypoint sequence
	 * @param wps Waypoint sequence object
	 */
	void publish_waypoint_sequence(const mrpt_msgs::msg::WaypointSequence& wps);
};

TPS_Astar_Planner_Node::TPS_Astar_Planner_Node() : rclcpp::Node(NODE_NAME)
{
	const auto qos = rclcpp::SystemDefaultsQoS();

	read_parameters();

	// Init ROS subs:
	// -----------------------

	sub_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
		topic_goal_sub_, qos,
		[this](const geometry_msgs::msg::PoseStamped& msg) {
			this->callback_goal(msg);
		});

	sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
		topic_obstacles_gridmap_sub_, qos,
		[this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
			this->callback_map(msg);
		});

	sub_replan_ = this->create_subscription<
		geometry_msgs::msg::PoseWithCovarianceStamped>(
		topic_replan_sub_, qos,
		[this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
				   msg) { this->callback_replan(msg); });

	sub_obstacles_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
		topic_obstacles_sub_, qos,
		[this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
			this->callback_obstacles(msg);
		});

	// Init ROS publishers:
	// -----------------------

	pub_wp_seq_ = this->create_publisher<mrpt_msgs::msg::WaypointSequence>(
		topic_wp_seq_pub_, qos);

	initialize_planner();
}

void TPS_Astar_Planner_Node::read_parameters()
{
	std::vector<double> nav_goal;
	std::vector<double> start_pose;
	std::vector<double> start_vel;

	this->declare_parameter<std::vector<double>>("nav_goal", {0.0, 0.0, 0.0});
	this->get_parameter("nav_goal", nav_goal);
	if (nav_goal.size() != 3)
	{
		RCLCPP_ERROR(this->get_logger(), "Invalid nav_goal parameter");
		return;
	}
	nav_goal_ = mrpt::math::TPose2D(nav_goal[0], nav_goal[1], nav_goal[2]);

	RCLCPP_INFO_STREAM(
		this->get_logger(),
		"[TPS_Astar_Planner_Node] nav goal =" << nav_goal_.asString());

	this->declare_parameter<std::vector<double>>("start_pose", {0.0, 0.0, 0.0});
	this->get_parameter("start_pose", start_pose);
	if (start_pose.size() != 3)
	{
		RCLCPP_ERROR(this->get_logger(), "Invalid start pose parameter");
		return;
	}
	start_pose_ =
		mrpt::math::TPose2D(start_pose[0], start_pose[1], start_pose[2]);
	RCLCPP_INFO_STREAM(
		this->get_logger(),
		"[TPS_Astar_Planner_Node] start pose =" << start_pose_.asString());

	this->declare_parameter<std::vector<double>>("start_vel", {0.0, 0.0, 0.0});
	this->get_parameter("start_vel", start_vel);
	if (start_vel.size() != 3)
	{
		RCLCPP_ERROR(this->get_logger(), "Invalid start velocity parameter");
		return;
	}
	start_vel_ = mrpt::math::TTwist2D(start_vel[0], start_vel[1], start_vel[2]);
	RCLCPP_INFO_STREAM(
		this->get_logger(), "[TPS_Astar_Planner_Node] starting velocity ="
								<< start_vel_.asString());

	this->declare_parameter<bool>("show_gui", false);
	this->get_parameter("show_gui", gui_mrpt_);
	RCLCPP_INFO(
		this->get_logger(), "MRPT GUI Enabled: %s",
		gui_mrpt_ ? "true" : "false");

	this->declare_parameter<std::string>("topic_goal_sub", topic_goal_sub_);
	this->get_parameter("topic_goal_sub", topic_goal_sub_);
	RCLCPP_INFO(
		this->get_logger(), "topic_goal_sub %s", topic_goal_sub_.c_str());

	this->declare_parameter<std::string>("topic_obstacles_gridmap_sub", "map");
	this->get_parameter(
		"topic_obstacles_gridmap_sub", topic_obstacles_gridmap_sub_);
	RCLCPP_INFO(
		this->get_logger(), "topic_obstacles_gridmap_sub %s",
		topic_obstacles_gridmap_sub_.c_str());

	this->declare_parameter<std::string>(
		"topic_obstacles_sub", "/map_pointcloud");
	this->get_parameter("topic_obstacles_sub", topic_obstacles_sub_);
	RCLCPP_INFO(
		this->get_logger(), "topic_obstacles_sub %s",
		topic_obstacles_sub_.c_str());

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

#if 0  // JLBC: does this make sense with pointcloud obstacles?
		if (!is_pose_within_map_bounds(nav_goal_))
		{
			RCLCPP_WARN(
				this->get_logger(), "Received goal is outside of the map");
			return;
		}
#endif

		path_plan_done_ = do_path_plan(start_pose_, nav_goal_);
	}
	catch (const std::exception& e)
	{
		RCLCPP_ERROR(
			this->get_logger(), "Exception in goal callback : %s", e.what());
	}
}

void TPS_Astar_Planner_Node::callback_map(
	const nav_msgs::msg::OccupancyGrid::SharedPtr& _map)
{
	RCLCPP_DEBUG_STREAM(
		this->get_logger(), "Navigator Map received for planning");
	std::call_once(
		map_received_flag_, [this, &_map]() { this->update_map(_map); });
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
	const sensor_msgs::msg::PointCloud2::SharedPtr& _pc)
{
	update_obstacles(_pc);
}

void TPS_Astar_Planner_Node::update_obstacles(
	const sensor_msgs::msg::PointCloud2::SharedPtr& _pc)
{
	mrpt::maps::CSimplePointsMap point_cloud;
	if (!mrpt::ros2bridge::fromROS(*_pc, point_cloud))
	{
		RCLCPP_ERROR(
			this->get_logger(),
			"Failed to convert Point Cloud to MRPT Points Map");
	}
	std::lock_guard<std::mutex> csl(obstacles_cs_);
	obstacle_src_ = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(
		std::make_shared<mrpt::maps::CSimplePointsMap>(point_cloud));

	RCLCPP_DEBUG_STREAM(this->get_logger(), "Obstacles update complete");
}

void TPS_Astar_Planner_Node::publish_waypoint_sequence(
	const mrpt_msgs::msg::WaypointSequence& wps)
{
	pub_wp_seq_->publish(wps);
}

bool TPS_Astar_Planner_Node::is_pose_within_map_bounds(
	const mrpt::math::TPose2D& pose)
{
	if (!obstacles_)
	{
		RCLCPP_ERROR(this->get_logger(), "No map to check");
		return false;
	}

	auto map_bbox = obstacles_->boundingBox();
	if (pose.x >= map_bbox.min.x && pose.x <= map_bbox.max.x &&
		pose.y >= map_bbox.min.y && pose.y <= map_bbox.max.y)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void TPS_Astar_Planner_Node::init_3d_debug()
{
	if (!win_3d_)
	{
		win_3d_ = mrpt::gui::CDisplayWindow3D::Create(
			"Pathplanning-TPS-AStar", 1000, 600);
		win_3d_->setCameraZoom(20);
		win_3d_->setCameraAzimuthDeg(-45);

		auto plane = obstacles_->getVisualization();
		scene_.insert(plane);

		{
			mrpt::opengl::COpenGLScene::Ptr ptr_scene =
				win_3d_->get3DSceneAndLock();

			ptr_scene->insert(plane);

			ptr_scene->enableFollowCamera(true);

			win_3d_->unlockAccess3DScene();
		}
	}  // Show 3D?
}

void TPS_Astar_Planner_Node::update_map(
	const nav_msgs::msg::OccupancyGrid::SharedPtr& msg)
{
	mrpt::maps::COccupancyGridMap2D grid;
	// ASSERT_(grid.countMapsByClass<mrpt::maps::COccupancyGridMap2D>());
	mrpt::ros2bridge::fromROS(*msg, grid);
	auto obsPts = mrpt::maps::CSimplePointsMap::Create();
	grid.getAsPointCloud(*obsPts);
	RCLCPP_INFO_STREAM(this->get_logger(), "Setting gridmap for planning");
	obstacles_ = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(obsPts);

	// path_plan_done_ = do_path_plan(start_pose_, nav_goal_);
}

bool TPS_Astar_Planner_Node::do_path_plan(
	mrpt::math::TPose2D& start, mrpt::math::TPose2D& goal)
{
	RCLCPP_INFO_STREAM(this->get_logger(), "Do path planning");
	auto obs = mpp::ObstacleSource::FromStaticPointcloud(obstacles_);

	planner_input_.stateStart.pose = start;
	planner_input_.stateStart.vel = start_vel_;
	planner_input_.stateGoal.state = goal;
	planner_input_.obstacles.emplace_back(obs);
	auto bbox = obs->obstacles()->boundingBox();

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
						<< "\n Obstacles : " << obs->obstacles()->size()
						<< "points \n  World bbox : "
						<< planner_input_.worldBboxMin.asString() << "-"
						<< planner_input_.worldBboxMax.asString());

	auto costmap = mpp::CostEvaluatorCostMap::FromStaticPointObstacles(
		*obstacles_, costMapParams_, planner_input_.stateStart.pose);

	planner_->costEvaluators_.push_back(costmap);

	// Insert custom progress callback:
	planner_->progressCallback_ = [](const mpp::ProgressCallbackData& pcd) {
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

	if (plan.success)
	{
		// activePlanOutput_ = plan;
		costEvaluators_ = planner_->costEvaluators_;
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
