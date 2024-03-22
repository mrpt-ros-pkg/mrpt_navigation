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
#include <mrpt/ros2bridge/point_cloud.h>
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/time.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <functional>
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
#include <sstream>
#include <std_msgs/msg/bool.hpp>
#include <string>

// for debugging
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/stock_objects.h>

const char* NODE_NAME = "mrpt_tps_astar_node";

class TPS_Astar_Nav_Node;

/**
 * @brief RobotInterface is the robot interface for NavEngine.
 * NavEngine algorithms callback into the interface functions to execute
 * robot autonomous navigation
 */
class RobotInterface : public mpp::VehicleMotionInterface, mpp::ObstacleSource
{
   public:
	/* Ctor*/
	RobotInterface(TPS_Astar_Nav_Node& parent);

	/**
	 * @brief Provides access to the vehicle localization data.
	 *
	 * In case of a hardware/communication error, leave `valid=false` in the
	 * return structure.
	 */
	mpp::VehicleLocalizationState get_localization() override;

	/** @brief Access method to the vehicle odometry data.
	 *
	 *
	 * In case of a hardware/communication error, leave `valid=false` in the
	 * return structure.
	 */
	mpp::VehicleOdometryState get_odometry() override;

	/**
	 * @brief Refer Parent class for docs
	 *
	 */
	bool motion_execute(
		const std::optional<mrpt::kinematics::CVehicleVelCmd::Ptr>& immediate,
		const std::optional<mpp::EnqueuedMotionCmd>& next) override;

	bool supports_enqeued_motions() const override;

	bool enqeued_motion_pending() const override;

	bool enqeued_motion_timed_out() const override;

	std::optional<mpp::VehicleOdometryState>
		enqued_motion_last_odom_when_triggered() const override;

	bool changeSpeeds(const mrpt::kinematics::CVehicleVelCmd& vel_cmd);

	void stop(const mpp::STOP_TYPE stopType) override;

	mrpt::kinematics::CVehicleVelCmd::Ptr getStopCmd();

	mrpt::kinematics::CVehicleVelCmd::Ptr getEmergencyStopCmd();

	void stop_watchdog() override;

	void start_watchdog(const size_t periodMilliseconds) override;

	void on_nav_end_due_to_error() override;

	void on_nav_start() override;

	void on_nav_end() override;

	void on_path_seems_blocked() override;

	void on_apparent_collision() override;

	mrpt::maps::CPointsMap::Ptr obstacles(
		[[maybe_unused]] mrpt::system::TTimeStamp t =
			mrpt::system::TTimeStamp()) override;

	// JL: Needed?
	// CObject* clone() const override { return nullptr; }

   private:
	//!< Top level parent class object
	TPS_Astar_Nav_Node& parent_;

	/// Indicator that enqueued command is pending and needs to be executed
	bool enqueued_motion_pending_;

	/// indicator that enqueued command timedout
	bool enqueued_motion_timeout_;
	mpp::VehicleOdometryState enqueued_motion_trigger_odom_;

	std::mutex enqueued_motion_mutex_;	//!< mutex for enqueued commands
};

/**
 * The main ROS2 node class.
 */
class TPS_Astar_Nav_Node : public rclcpp::Node
{
	friend class RobotInterface;

   public:
	TPS_Astar_Nav_Node();
	virtual ~TPS_Astar_Nav_Node() = default;

   private:
	/// CTimeLogger instance for profiling
	mrpt::system::CTimeLogger profiler_;

	/// Flag for performing initialization once
	std::once_flag init_flag_;

	/// Flag for receiving map once
	std::once_flag map_received_flag_;

	/// Pointer to a grid map
	mrpt::maps::CPointsMap::Ptr grid_map_;

	/// Message for waypoint sequence
	mrpt_msgs::msg::WaypointSequence wps_msg_;

	/// Navigation goal position
	mrpt::math::TPose2D nav_goal_{0, 0, 0};

	/// Navigation start position
	mrpt::math::TPose2D start_pose_{0, 0, 0};

	/// Robot velocity at start
	mrpt::math::TTwist2D start_vel_{0, 0, 0};

	/// Vehicle localization state
	mpp::VehicleLocalizationState localization_pose_;

	/// Vehicle odometry state
	mpp::VehicleOdometryState odometry_;

	/// Pointer to obstacle map
	mrpt::maps::CPointsMap::Ptr obstacle_src_;

	/// Flag for initializing navigation engine once
	std::once_flag init_nav_flag_;

	/// Flag indicating navigation engine is initialized
	bool nav_engine_init_ = false;

	/// Timer for running navigator periodically
	rclcpp::TimerBase::SharedPtr timer_run_nav_;

	/// Timer for enqueued command status checker
	rclcpp::TimerBase::SharedPtr timer_enqueue_;

	/// Time period [s] of navigation step
	double nav_period_ = 0.25;

	/// Time period [s] of enqueued command status checker
	double enq_cmd_check_time_ = 0.020;

	/// Subscriber to map
	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;

	/// Subscriber to localization info
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>
		sub_localization_pose_;

	/// Subscriber to odometry info from robot
	rclcpp::Subscription<nav_msgs::msg::Odometry> sub_odometry_;

	/// Subscriber to obstacle map info
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2> sub_obstacles_;

	/// Subscriber to topic from rnav that tells to replan
	/// TODO(JL): Switch into a service!
	rclcpp::Subscription<std_msgs::msg::Bool> sub_replan_;

	/// Publisher for velocity commands for the robot
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;

	/// Publisher for waypoint sequence
	rclcpp::Publisher<mrpt_msgs::msg::WaypointSequence>::SharedPtr pub_wp_seq_;

	/// Mutex for obstacle data
	std::mutex obstacles_cs_;

	/// Mutex for localization data
	std::mutex localization_cs_;

	/// Mutex for odometry data
	std::mutex odometry_cs_;

	/// Mutex for enqueued command
	std::mutex next_cmd_cs_;

	/// Debug flag
	bool debug_ = true;

	/// Flag for MRPT GUI
	bool gui_mrpt_ = false;

	/// Pointer to MRPT 3D display window
	mrpt::gui::CDisplayWindow3D::Ptr win_3d_;

	/// MRPT OpenGL scene
	mrpt::opengl::COpenGLScene scene_;

	/// Callback for jackal robot when enqueued command fired
	std::function<void()> on_enqueued_motion_fired;

	/// Callback for jackal robot when enqueued command timed out
	std::function<void()> on_enqueued_motion_timeout;

	/// Pose at which enqueued command should be executed
	mrpt::math::TPose2D motion_trigger_pose_;

	/// Pose tolerance for enqueued command
	mrpt::math::TPose2D motion_trigger_tolerance_;

	/// Timeout period within which the enqueued command should execute
	double motion_trigger_timeout_ = 0;

	/// Pointer to enqueued motion command
	mrpt::kinematics::CVehicleVelCmd::Ptr next_cmd_;

	/// Pointer to NO-op command, continue with previous command
	mrpt::kinematics::CVehicleVelCmd::Ptr NOP_cmd_;

	/// Timer that checks if enqueued command timed out
	double enq_motion_timer_ = 0;

	std::shared_ptr<RobotInterface> robot_;

	mpp::PlannerOutput activePlanOutput_;
	std::vector<mpp::CostEvaluator::Ptr> costEvaluators_;
	bool path_plan_done_ = false;

	std::shared_ptr<mpp::NavEngine> nav_engine_;
	mpp::WaypointSequence waypts_;	//<! DS for waypoints
	mpp::WaypointStatusSequence wayptsStatus_;	//<! DS for waypoint status

   private:
	template <typename T>
	std::vector<T> processStringParam(const std::string& parastr_);

	/* Define callbacks*/
	void callbackMap(const nav_msgs::msg::OccupancyGrid& msg);
	void callbackLocalization(
		const geometry_msgs::msg::PoseWithCovarianceStamped& _pose);
	void callbackOdometry(const nav_msgs::msg::Odometry& _odom);
	void callbackObstacles(const sensor_msgs::msg::PointCloud2& _pc);
	void callbackReplan(const std_msgs::msg::Bool& _flag);

	/* update methods*/
	void updateMap(const nav_msgs::msg::OccupancyGrid& _msg);
	void updateLocalization(
		const geometry_msgs::msg::PoseWithCovarianceStamped& _pose);
	void updateOdom(const nav_msgs::msg::Odometry& _odom);
	void updateObstacles(const sensor_msgs::msg::PointCloud2& _pc);

	bool do_path_plan(mrpt::math::TPose2D& start, mrpt::math::TPose2D& goal);
	void init3DDebug();

	/*Getter functions for Robot interface*/
	mpp::VehicleLocalizationState get_localization_state();
	mpp::VehicleOdometryState get_odometry_state();
	mrpt::maps::CPointsMap::Ptr get_current_obstacles();

	void publish_cmd_vel(const geometry_msgs::msg::Twist& cmd_vel);
	void publish_waypoint_sequence(const mrpt_msgs::msg::WaypointSequence& wps);

	/* Navigator methods*/
	void initializeNavigator();
	void navigateTo(const mrpt::math::TPose2D& target);
	void onDoNavigation(const ros::TimerEvent&);
	void checkEnqueuedMotionCmds(const ros::TimerEvent&);
};

TPS_Astar_Nav_Node::TPS_Astar_Nav_Node() : rclcpp::Node(NODE_NAME)
{
	{
		const auto qos = rclcpp::SystemDefaultsQoS();

		std::string nav_goal_str = "[0.0, 0.0, 0.0]";
		std::string start_pose_str = "[0.0, 0.0, 0.0]";
		std::string vel_str = "2.0";

		this->declare_parameter<std::string>("nav_goal", nav_goal_str);
		nav_goal_str = this->get_parameter("nav_goal").as_string();

		localn_.param("nav_goal", nav_goal_str, nav_goal_str);
		std::vector<double> goal_pose =
			processStringParam<double>(nav_goal_str);
		if (goal_pose.size() != 3)
		{
			ROS_ERROR("Invalid nav_goal parameter.");
			return;
		}
		nav_goal_ =
			mrpt::math::TPose2D(goal_pose[0], goal_pose[1], goal_pose[2]);
		RCLCPP_INFO_STREAM(
			this->get_logger(),
			"[TPS_Astar_Nav_Node]nav goal received =" << nav_goal_.asString());

		localn_.param("start_pose", start_pose_str, start_pose_str);
		std::vector<double> start_pose =
			processStringParam<double>(start_pose_str);
		if (start_pose.size() != 3)
		{
			ROS_ERROR("Invalid start pose parameter.");
			return;
		}
		start_pose_ =
			mrpt::math::TPose2D(start_pose[0], start_pose[1], start_pose[2]);
		RCLCPP_INFO_STREAM(
			this->get_logger(), "[TPS_Astar_Nav_Node] start pose received ="
									<< start_pose_.asString());

		localn_.param("start_vel", vel_str, vel_str);
		start_vel_ = mrpt::math::TTwist2D(std::stod(vel_str), 0.0, 0.0);
		RCLCPP_INFO_STREAM(
			this->get_logger(),
			"[TPS_Astar_Nav_Node]starting velocity =" << start_vel_.asString());

		localn_.param("mrpt_gui", gui_mrpt_str_, gui_mrpt_str_);
		if (!gui_mrpt_str_.empty())
			gui_mrpt_ = stringToBool(gui_mrpt_str_);
		else
			gui_mrpt_ = false;

		localn_.param("topic_map_sub", sub_map_str_, sub_map_str_);
		sub_map_ = nh_.subscribe(
			sub_map_str_, 1, &TPS_Astar_Nav_Node::callbackMap, this);

		localn_.param(
			"topic_localization_sub", sub_localization_str_,
			sub_localization_str_);
		sub_localization_pose_ = nh_.subscribe(
			sub_localization_str_, 1,
			[this](const geometry_msgs::msg::PoseWithCovarianceStamped& pose) {
				updateLocalization(pose);
			});

		localn_.param(
			"topic_odometry_sub", sub_odometry_str_, sub_odometry_str_);
		sub_odometry_ = this->create_subscription<>(
			sub_odometry_str_, 1, [this]() { updateOdom(odom); });

		localn_.param(
			"topic_obstacles_sub", sub_obstacles_str_, sub_obstacles_str_);
		sub_obstacles_ = nh_.subscribe(
			sub_obstacles_str_, 1, &TPS_Astar_Nav_Node::callbackObstacles,
			this);

		localn_.param("topic_replan_sub", sub_replan_str_, sub_replan_str_);
		sub_replan_ = nh_.subscribe(
			sub_replan_str_, 1, &TPS_Astar_Nav_Node::callbackReplan, this);

		localn_.param("topic_cmd_vel_pub", pub_cmd_vel_str_, pub_cmd_vel_str_);
		pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>(pub_cmd_vel_str_, 1);

		localn_.param("topic_wp_seq_pub", pub_wp_seq_str_, pub_wp_seq_str_);
		pub_wp_seq_ =
			nh_.advertise<mrpt_msgs::WaypointSequence>(pub_wp_seq_str_, 1);

		// Init timers:
		timer_run_nav_ = this->create_wall_timer(
			std::chrono::microseconds(mrpt::round(1e6 * nav_period_)),
			std::bind(&TPS_Astar_Nav_Node::onDoNavigation, this));

		// Odometry publisher runs at 50Hz, so this functions runs at the same
		// periodicity
		timer_enqueue_ = this->create_wall_timer(
			std::chrono::microseconds(mrpt::round(1e6 * enq_cmd_check_time_)),
			std::bind(&TPS_Astar_Nav_Node::timerCallback, this));

		if (nav_engine_)
		{
			RCLCPP_INFO_STREAM(
				this->get_logger(),
				"TPS Astart Navigator already initialized, resetting nav "
				"engine");
			nav_engine_.reset();
		}

		nav_engine_ = std::make_shared<mpp::TPS_Navigator>();

		robot_ = std::make_shared<RobotInterface>(*this);
	}

	bool TPS_Astar_Nav_Node::stringToBool(const std::string& str)
	{
		if (str == "true" || str == "True" || str == "TRUE" || str == "1")
			return true;
		else
			return false;
		//    if(str == "false" || str == "False" || str == "FALSE" || str ==
		//    "0") return false;
	}
	template <typename T>
	std::vector<T> TPS_Astar_Nav_Node::processStringParam(
		const std::string& parastr_)
	{
		std::string str = parastr_;
		std::replace(str.begin(), str.end(), '[', ' ');
		std::replace(str.begin(), str.end(), ']', ' ');

		std::vector<T> result;
		std::istringstream iss(str);
		T value;

		while (iss >> value)
		{
			result.push_back(value);
			if (iss.peek() == ',')
			{
				iss.ignore();
			}
		}
		return result;
	}

	void TPS_Astar_Nav_Node::callbackMap(const nav_msgs::OccupancyGrid& _map)
	{
		// RCLCPP_INFO_STREAM(this->get_logger(), "Navigator Map received for
		// planning");
		std::call_once(
			map_received_flag_, [this, _map]() { this->updateMap(_map); });
	}

	void TPS_Astar_Nav_Node::callbackOdometry(const nav_msgs::Odometry& _odom)
	{
	}

	void TPS_Astar_Nav_Node::callbackObstacles(
		const sensor_msgs::msg::PointCloud2& _pc)
	{
		updateObstacles(_pc);
	}

	void TPS_Astar_Nav_Node::callbackReplan(const std_msgs::Bool& _flag)
	{
		auto& pose = localization_pose_;

		if (pose.valid && _flag.data)
		{
			path_plan_done_ = do_path_plan(pose.pose, nav_goal_);
		}
	}

	void TPS_Astar_Nav_Node::publish_cmd_vel(
		const geometry_msgs::Twist& cmd_vel)
	{
		RCLCPP_INFO_STREAM(
			this->get_logger(), "Publishing velocity command" << cmd_vel);
		pub_cmd_vel_.publish(cmd_vel);
	}

	void TPS_Astar_Nav_Node::publish_waypoint_sequence(
		const mrpt_msgs::WaypointSequence& wps)
	{
		pub_wp_seq_.publish(wps);
	}

	void TPS_Astar_Nav_Node::init3DDebug()
	{
		ROS_INFO("init3DDebug");

		if (!win_3d_)
		{
			win_3d_ = mrpt::gui::CDisplayWindow3D::Create(
				"Pathplanning-TPS-AStar", 1000, 600);
			win_3d_->setCameraZoom(20);
			win_3d_->setCameraAzimuthDeg(-45);

			auto plane = grid_map_->getVisualization();
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

	void TPS_Astar_Nav_Node::updateLocalization(
		const geometry_msgs::PoseWithCovarianceStamped& msg)
	{
		tf2::Quaternion quat(
			msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
			msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
		tf2::Matrix3x3 mat(quat);
		double roll, pitch, yaw;
		mat.getRPY(roll, pitch, yaw);
		std::lock_guard<std::mutex> csl(localization_cs_);
		localization_pose_.frame_id = msg.header.frame_id;
		localization_pose_.valid = true;
		localization_pose_.pose.x = msg.pose.pose.position.x;
		localization_pose_.pose.y = msg.pose.pose.position.y;
		localization_pose_.pose.phi = yaw;
		localization_pose_.timestamp =
			mrpt::ros1bridge::fromROS(msg.header.stamp);
		// RCLCPP_INFO_STREAM(this->get_logger(), "Localization update
		// complete");
	}

	void TPS_Astar_Nav_Node::updateOdom(const nav_msgs::Odometry& msg)
	{
		tf2::Quaternion quat(
			msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
			msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
		tf2::Matrix3x3 mat(quat);
		double roll, pitch, yaw;
		mat.getRPY(roll, pitch, yaw);
		std::lock_guard<std::mutex> csl(odometry_cs_);
		odometry_.odometry.x = msg.pose.pose.position.x;
		odometry_.odometry.y = msg.pose.pose.position.y;
		odometry_.odometry.phi = yaw;

		odometry_.odometryVelocityLocal.vx = msg.twist.twist.linear.x;
		odometry_.odometryVelocityLocal.vy = msg.twist.twist.linear.y;
		odometry_.odometryVelocityLocal.omega = msg.twist.twist.angular.z;

		odometry_.valid = true;
		odometry_.timestamp = mrpt::system::now();
		/*TODO*/
		odometry_.pendedActionExists = robot_->enqeued_motion_pending();
		// RCLCPP_INFO_STREAM(this->get_logger(), "Odometry update complete");
	}

	void TPS_Astar_Nav_Node::updateObstacles(const sensor_msgs::PointCloud& _pc)
	{
		mrpt::maps::CSimplePointsMap point_cloud;
		if (!mrpt::ros1bridge::fromROS(_pc, point_cloud))
		{
			ROS_ERROR("Failed to convert Point Cloud to MRPT Points Map");
		}
		std::lock_guard<std::mutex> csl(obstacles_cs_);
		obstacle_src_ = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(
			std::make_shared<mrpt::maps::CSimplePointsMap>(point_cloud));

		// RCLCPP_INFO_STREAM(this->get_logger(), "Obstacles update complete");
	}

	void TPS_Astar_Nav_Node::updateMap(const nav_msgs::OccupancyGrid& msg)
	{
		mrpt::maps::COccupancyGridMap2D grid;
		// ASSERT_(grid.countMapsByClass<mrpt::maps::COccupancyGridMap2D>());
		mrpt::ros1bridge::fromROS(msg, grid);
		auto obsPts = mrpt::maps::CSimplePointsMap::Create();
		grid.getAsPointCloud(*obsPts);
		RCLCPP_INFO_STREAM(this->get_logger(), "Setting gridmap for planning");
		grid_map_ = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(obsPts);

		path_plan_done_ = do_path_plan(start_pose_, nav_goal_);
	}

	void TPS_Astar_Nav_Node::initializeNavigator()
	{
		if (!nav_engine_)
		{
			ROS_ERROR("TPS_AStar Not created!");
			return;
		}

		nav_engine_->setMinLoggingLevel(
			mrpt::system::VerbosityLevel::LVL_DEBUG);
		nav_engine_->config_.vehicleMotionInterface =
			std::dynamic_pointer_cast<mpp::VehicleMotionInterface>(
				/*std::make_shared<RobotInterface>*/ (robot_));
		nav_engine_->config_.vehicleMotionInterface->setMinLoggingLevel(
			mrpt::system::VerbosityLevel::LVL_DEBUG);
		nav_engine_->config_.globalMapObstacleSource =
			mpp::ObstacleSource::FromStaticPointcloud(grid_map_);
		nav_engine_->config_.localSensedObstacleSource =
			mpp::ObstacleSource::FromStaticPointcloud(obstacle_src_);

		{
			std::string ptg_ini_file;
			localn_.param("ptg_ini", ptg_ini_file, ptg_ini_file);

			ROS_ASSERT_MSG(
				mrpt::system::fileExists(ptg_ini_file),
				"PTG ini file not found: '%s'", ptg_ini_file.c_str());
			mrpt::config::CConfigFile cfg(ptg_ini_file);
			nav_engine_->config_.ptgs.initFromConfigFile(cfg, "SelfDriving");
		}

		{
			// cost map:
			std::string costmap_parafile_;

			localn_.param(
				"global_costmap_parameters", costmap_parafile_,
				costmap_parafile_);

			ROS_ASSERT_MSG(
				mrpt::system::fileExists(costmap_parafile_),
				"costmap params file not found: '%s'",
				costmap_parafile_.c_str());

			nav_engine_->config_.globalCostParameters =
				mpp::CostEvaluatorCostMap::Parameters::FromYAML(
					mrpt::containers::yaml::FromFile(costmap_parafile_));

			nav_engine_->config_.localCostParameters =
				mpp::CostEvaluatorCostMap::Parameters::FromYAML(
					mrpt::containers::yaml::FromFile(costmap_parafile_));
		}

		// Preferred waypoints:
		{
			std::string wp_params_file;
			localn_.param(
				"prefer_waypoints_parameters", wp_params_file, wp_params_file);

			ROS_ASSERT_MSG(
				mrpt::system::fileExists(wp_params_file),
				"Prefer waypoints params file not found: '%s'",
				wp_params_file.c_str());
			nav_engine_->config_.preferWaypointsParameters =
				mpp::CostEvaluatorPreferredWaypoint::Parameters::FromYAML(
					mrpt::containers::yaml::FromFile(wp_params_file));
		}

		{
			std::string planner_parameters_file;
			localn_.param(
				"planner_parameters", planner_parameters_file,
				planner_parameters_file);

			ROS_ASSERT_MSG(
				mrpt::system::fileExists(planner_parameters_file),
				"Planner params file not found: '%s'",
				planner_parameters_file.c_str());

			nav_engine_->config_.plannerParams =
				mpp::TPS_Astar_Parameters::FromYAML(
					mrpt::containers::yaml::FromFile(planner_parameters_file));
		}

		{
			std::string nav_engine_parameters_file;
			localn_.param(
				"nav_engine_parameters", nav_engine_parameters_file,
				nav_engine_parameters_file);

			ROS_ASSERT_MSG(
				mrpt::system::fileExists(nav_engine_parameters_file),
				"Planner params file not found: '%s'",
				nav_engine_parameters_file.c_str());

			nav_engine_->config_.loadFrom(
				mrpt::containers::yaml::FromFile(nav_engine_parameters_file));
		}

		nav_engine_->initialize();

		RCLCPP_INFO_STREAM(
			this->get_logger(), "TPS_Astar Navigator intialized");

		if (path_plan_done_)
		{
			navigateTo(nav_goal_);
		}
		nav_engine_init_ = true;
	}

	void TPS_Astar_Nav_Node::navigateTo(const mrpt::math::TPose2D& target)
	{
		// mpp::Waypoint waypoint(target.x, target.y, 1.0, false, 0.0, 0.0);

		// RCLCPP_INFO_STREAM(this->get_logger(), "[TPS_Astar_Nav_Node]
		// navigateTo"<<waypoint.getAsText());

		// waypts_.waypoints.push_back(waypoint);

		nav_engine_->request_navigation(waypts_);
	}

	bool TPS_Astar_Nav_Node::do_path_plan(
		mrpt::math::TPose2D & start, mrpt::math::TPose2D & goal)
	{
		RCLCPP_INFO_STREAM(this->get_logger(), "Do path planning");
		auto obs = mpp::ObstacleSource::FromStaticPointcloud(grid_map_);
		mpp::PlannerInput planner_input;

		planner_input.stateStart.pose = start;
		planner_input.stateStart.vel = start_vel_;
		planner_input.stateGoal.state = goal;
		planner_input.obstacles.emplace_back(obs);
		auto bbox = obs->obstacles()->boundingBox();

		{
			const auto bboxMargin = mrpt::math::TPoint3Df(2.0, 2.0, .0);
			const auto ptStart = mrpt::math::TPoint3Df(
				planner_input.stateStart.pose.x,
				planner_input.stateStart.pose.y, 0);
			const auto ptGoal = mrpt::math::TPoint3Df(
				planner_input.stateGoal.asSE2KinState().pose.x,
				planner_input.stateGoal.asSE2KinState().pose.y, 0);
			bbox.updateWithPoint(ptStart - bboxMargin);
			bbox.updateWithPoint(ptStart + bboxMargin);
			bbox.updateWithPoint(ptGoal - bboxMargin);
			bbox.updateWithPoint(ptGoal + bboxMargin);
		}

		planner_input.worldBboxMax = {bbox.max.x, bbox.max.y, PI_};
		planner_input.worldBboxMin = {bbox.min.x, bbox.min.y, -PI_};

		std::cout << "Start state: " << planner_input.stateStart.asString()
				  << "\n";
		std::cout << "Goal state : " << planner_input.stateGoal.asString()
				  << "\n";
		std::cout << "Obstacles  : " << obs->obstacles()->size() << " points\n";
		std::cout << "World bbox : " << planner_input.worldBboxMin.asString()
				  << " - " << planner_input.worldBboxMax.asString() << "\n";

		mpp::Planner::Ptr planner = mpp::TPS_Astar::Create();

		// Enable time profiler:
		planner->profiler_().enable(true);

		{
			// cost map:
			std::string costmap_parafile_;

			localn_.param(
				"global_costmap_parameters", costmap_parafile_,
				costmap_parafile_);

			ROS_ASSERT_MSG(
				mrpt::system::fileExists(costmap_parafile_),
				"costmap params file not found: '%s'",
				costmap_parafile_.c_str());

			const auto costMapParams =
				mpp::CostEvaluatorCostMap::Parameters::FromYAML(
					mrpt::containers::yaml::FromFile(costmap_parafile_));

			auto costmap = mpp::CostEvaluatorCostMap::FromStaticPointObstacles(
				*grid_map_, costMapParams, planner_input.stateStart.pose);

			RCLCPP_INFO_STREAM(
				this->get_logger(),
				"******************************* Costmap file read");

			planner->costEvaluators_.push_back(costmap);
		}

		{
			std::string planner_parameters_file;
			localn_.param(
				"planner_parameters", planner_parameters_file,
				planner_parameters_file);

			ROS_ASSERT_MSG(
				mrpt::system::fileExists(planner_parameters_file),
				"Planner params file not found: '%s'",
				planner_parameters_file.c_str());

			const auto c =
				mrpt::containers::yaml::FromFile(planner_parameters_file);
			planner->params_froyaml_(c);
			std::cout << "Loaded these planner params:\n";
			planner->params_as_yaml().printAsYAML();
		}

		// Insert custom progress callback:
		planner->progressCallback_ = [](const mpp::ProgressCallbackData& pcd) {
			std::cout << "[progressCallback] bestCostFromStart: "
					  << pcd.bestCostFromStart
					  << " bestCostToGoal: " << pcd.bestCostToGoal
					  << " bestPathLength: " << pcd.bestPath.size()
					  << std::endl;
		};

		{
			std::string ptg_ini_file;
			localn_.param("ptg_ini", ptg_ini_file, ptg_ini_file);

			ROS_ASSERT_MSG(
				mrpt::system::fileExists(ptg_ini_file),
				"PTG ini file not found: '%s'", ptg_ini_file.c_str());
			mrpt::config::CConfigFile cfg(ptg_ini_file);
			planner_input.ptgs.initFromConfigFile(cfg, "SelfDriving");

			RCLCPP_INFO_STREAM(this->get_logger(), "PTG ini");
		}

		const mpp::PlannerOutput plan = planner->plan(planner_input);

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
			activePlanOutput_ = plan;
			costEvaluators_ = planner->costEvaluators_;
		}

		// backtrack:
		auto [plannedPath, pathEdges] =
			plan.motionTree.backtrack_path(*plan.bestNodeId);

#if 0  // JLBC: disabled to check if this is causing troubles
    mpp::refine_trajectory(plannedPath, pathEdges, planner_input.ptgs);
#endif

		// Show plan in a GUI for debugging
		if (plan.success)  // && gui_mrpt_
		{
			mpp::VisualizationOptions vizOpts;

			vizOpts.renderOptions.highlight_path_to_node_id = plan.bestNodeId;
			vizOpts.renderOptions.color_normal_edge = {0xb0b0b0, 0x20};	 // RGBA
			vizOpts.renderOptions.width_normal_edge =
				0;	// hide all edges except best path
			vizOpts.gui_modal = false;	// leave GUI open in a background thread

			mpp::viz_nav_plan(plan, vizOpts, planner->costEvaluators_);
		}

		// Interpolate so we have many waypoints:
		mpp::trajectory_t interpPath;
		if (plan.success)
		{
			const double interpPeriod = 0.25;  // [s]

			interpPath = mpp::plan_to_trajectory(
				pathEdges, planner_input.ptgs, interpPeriod);

			// Note: trajectory is in local frame of reference
			// of plan.originalInput.stateStart.pose
			// so, correct that relative pose so we keep everything in global
			// frame:
			const auto& startPose = plan.originalInput.stateStart.pose;
			for (auto& kv : interpPath)
				kv.second.state.pose = startPose + kv.second.state.pose;
		}

		wps_msg_ = mrpt_msgs::WaypointSequence();
		if (plan.success)
		{
#if 0
        size_t N = pathEdges.size();
        for(const auto& edge: pathEdges)
        {
            const auto& goal_state = edge->stateTo;
            std::cout<< "Waypoint: x = "<<goal_state.pose.x<<", y= "<<goal_state.pose.y<<std::endl;
            auto wp_msg = mrpt_msgs::Waypoint();
            wp_msg.target.position.x = goal_state.pose.x;
            wp_msg.target.position.y = goal_state.pose.y;
            wp_msg.target.position.z = 0.0;
            wp_msg.target.orientation.x = 0.0;
            wp_msg.target.orientation.y = 0.0;
            wp_msg.target.orientation.z = 0.0;
            wp_msg.target.orientation.w = 0.0;
            wp_msg.allow_skip = true;

            wps_msg_.waypoints.push_back(wp_msg);
        }
#else
			for (auto& kv : interpPath)
			{
				const auto& goal_state = kv.second.state;
				std::cout << "Waypoint: x = " << goal_state.pose.x
						  << ", y= " << goal_state.pose.y << std::endl;
				auto wp_msg = mrpt_msgs::Waypoint();
				wp_msg.target.position.x = goal_state.pose.x;
				wp_msg.target.position.y = goal_state.pose.y;
				wp_msg.target.position.z = 0.0;
				wp_msg.target.orientation.x = 0.0;
				wp_msg.target.orientation.y = 0.0;
				wp_msg.target.orientation.z = 0.0;
				wp_msg.target.orientation.w = 0.0;

				wp_msg.allowed_distance = 1.5;	// TODO: Make a param
				wp_msg.allow_skip = true;

				wps_msg_.waypoints.push_back(wp_msg);
			}
#endif

			auto wp_msg = mrpt_msgs::Waypoint();
			wp_msg.target.position.x = nav_goal_.x;
			wp_msg.target.position.y = nav_goal_.y;
			wp_msg.target.position.z = 0.0;
			tf2::Quaternion quaternion;
			quaternion.setRPY(0.0, 0.0, nav_goal_.phi);
			wp_msg.target.orientation.x = quaternion.x();
			wp_msg.target.orientation.y = quaternion.y();
			wp_msg.target.orientation.z = quaternion.z();
			wp_msg.target.orientation.w = quaternion.w();

			wp_msg.allowed_distance = 0.4;	// TODO: Make a param
			wp_msg.allow_skip = false;

			wps_msg_.waypoints.push_back(wp_msg);
		}
		return plan.success;
	}

	mpp::VehicleLocalizationState TPS_Astar_Nav_Node::get_localization_state()
	{
		std::lock_guard<std::mutex> csl(localization_cs_);
		return localization_pose_;
	}
	mpp::VehicleOdometryState TPS_Astar_Nav_Node::get_odometry_state()
	{
		std::lock_guard<std::mutex> csl(odometry_cs_);
		return odometry_;
	}
	mrpt::maps::CPointsMap::Ptr TPS_Astar_Nav_Node::get_current_obstacles()
	{
		std::lock_guard<std::mutex> csl(obstacles_cs_);
		return obstacle_src_;
	}

	void TPS_Astar_Nav_Node::onDoNavigation(const ros::TimerEvent&)
	{
		if (path_plan_done_)
		{
			publish_waypoint_sequence(wps_msg_);
		}

		// if(obstacle_src_ && localization_pose_.valid)
		// {
		//     std::call_once(init_nav_flag_,[this]()
		//     {this->initializeNavigator();});
		// }

		// if(nav_engine_init_)
		// {
		//     try
		//     {
		// 	    nav_engine_->navigation_step();
		//     }
		//     catch (const std::exception& e)
		//     {
		// 	    std::cerr << "[TPS_Astar_Nav_Node] Exception:" << e.what() <<
		// std::endl; 	    return;
		//     }

		// }
	}

	void TPS_Astar_Nav_Node::checkEnqueuedMotionCmds(const ros::TimerEvent&)
	{
		// if(odometry_.valid) // && jackal_robot_->enqeued_motion_pending())
		// {
		//     auto& odo = odometry_.odometry;
		//     RCLCPP_INFO_STREAM(this->get_logger(), "Odo: "<< odo.asString());
		//     auto& pose = localization_pose_.pose;
		//     RCLCPP_INFO_STREAM(this->get_logger(), "Pose: "<<
		//     pose.asString()); RCLCPP_INFO_STREAM(this->get_logger(), "Trigger
		//     Pose "<< motion_trigger_pose_.asString()); if(std::abs(odo.x -
		//     motion_trigger_pose_.x) < motion_trigger_tolerance_.x &&
		//        std::abs(odo.y - motion_trigger_pose_.y) <
		//        motion_trigger_tolerance_.y && std::abs(odo.phi -
		//        motion_trigger_pose_.phi) < motion_trigger_tolerance_.phi)
		//     {
		//         std::lock_guard<std::mutex>
		//         csl(jackal_robot_->enqueued_motion_mutex_);
		//         //RCLCPP_INFO_STREAM(this->get_logger(), "Enqueued motion
		//         fired"); on_enqueued_motion_fired();
		//         jackal_robot_->enqueued_motion_trigger_odom_ = odometry_;
		//         RCLCPP_INFO_STREAM(this->get_logger(), "calling change speeds
		//         upon pend action fired");
		//         //jackal_robot_->changeSpeeds(*next_cmd_);
		//         RCLCPP_INFO_STREAM(this->get_logger(), "Change speeds
		//         complete"); NOP_cmd_ = next_cmd_;
		//         RCLCPP_INFO_STREAM(this->get_logger(), "Next NOP command
		//         set");
		//     }
		//     else
		//     {
		//         RCLCPP_INFO_STREAM(this->get_logger(), "[TPS_Astar_Nav_Node]
		//         Enqueued motion timer =
		//         "<<enq_motion_timer_); enq_motion_timer_ +=
		//         enq_cmd_check_time_;
		//         if(jackal_robot_->enqeued_motion_pending()
		//         &&
		//           enq_motion_timer_ > motion_trigger_timeout_)
		//         {
		//             std::lock_guard<std::mutex>
		//             csl(jackal_robot_->enqueued_motion_mutex_);
		//             on_enqueued_motion_timeout();
		//             enq_motion_timer_ = 0.0;
		//         }
		//     }
		// }
	}

	// --------------------------------------

	RobotInterface::RobotInterface(TPS_Astar_Nav_Node & parent)
		: parent_(parent),
		  enqueued_motion_pending_(false),
		  enqueued_motion_timeout_(false)
	{
		parent_.on_enqueued_motion_fired = [this]() {
			RCLCPP_INFO_STREAM(
				this->get_logger(), "[Jackal Robot] Enqueud motion fired");
			changeSpeeds(*parent_.next_cmd_);
			RCLCPP_INFO_STREAM(
				this->get_logger(), "[Jackal Robot] change speeds");
			{
				// std::lock_guard<std::mutex> csl(enqueued_motion_mutex_);
				enqueued_motion_pending_ = false;
				enqueued_motion_timeout_ = false;
			}
			RCLCPP_INFO_STREAM(this->get_logger(), "Mutex Release");
			return;
		};

		parent_.on_enqueued_motion_timeout = [this]() {
			RCLCPP_INFO_STREAM(
				this->get_logger(), "[Jackal Robot] Enqueud motion timedout");
			// std::lock_guard<std::mutex> csl(enqueued_motion_mutex_);
			enqueued_motion_pending_ = false;
			enqueued_motion_timeout_ = true;
			return;
		};
	}

	mpp::VehicleLocalizationState RobotInterface::get_localization()
	{
		return parent_.get_localization_state();
	}

	mpp::VehicleOdometryState RobotInterface::get_odometry()
	{
		return parent_.get_odometry_state();
	}

	bool RobotInterface::motion_execute(
		const std::optional<mrpt::kinematics::CVehicleVelCmd::Ptr>& immediate,
		const std::optional<mpp::EnqueuedMotionCmd>& next)
	{
		if (immediate.has_value())
		{
			std::lock_guard<std::mutex> csl(enqueued_motion_mutex_);
			RCLCPP_INFO_STREAM(
				this->get_logger(), "[Jackal_Robot] Motion command received");
			mrpt::kinematics::CVehicleVelCmd& vel_cmd = *(immediate.value());
			changeSpeeds(vel_cmd);
			parent_.NOP_cmd_ = immediate.value();
			enqueued_motion_pending_ = false;
			enqueued_motion_timeout_ = false;
		}

		if (next.has_value())
		{
			// RCLCPP_INFO_STREAM(this->get_logger(), "[Jackal_Robot] Enqueued
			// command received");
			std::lock_guard<std::mutex> csl(enqueued_motion_mutex_);
			auto& enq_cmd = next.value();
			enqueued_motion_pending_ = true;
			enqueued_motion_timeout_ = false;
			parent_.motion_trigger_pose_ = enq_cmd.nextCondition.position;
			parent_.motion_trigger_tolerance_ = enq_cmd.nextCondition.tolerance;
			parent_.motion_trigger_timeout_ = enq_cmd.nextCondition.timeout;
			parent_.next_cmd_ = next->nextCmd;
			parent_.enq_motion_timer_ = 0.0;
			RCLCPP_INFO_STREAM(
				this->get_logger(),
				"[Jackal_Robot] Next cmd timeout = "
					<< parent_.motion_trigger_timeout_
					<< " Enq_Motion_timer = " << parent_.enq_motion_timer_);
		}

		if (!immediate.has_value() && !next.has_value())
		{
			std::lock_guard<std::mutex> csl(enqueued_motion_mutex_);
			RCLCPP_INFO_STREAM(
				this->get_logger(), "[Jackal_Robot] NOP command received");
			if (parent_.NOP_cmd_)
			{
				changeSpeeds(*parent_.NOP_cmd_);
				return true;
			}
		}

		return true;
	}

	bool RobotInterface::supports_enqeued_motions() const { return true; }

	bool RobotInterface::enqeued_motion_pending() const
	{
		auto lck = mrpt::lockHelper(enqueued_motion_mutex_);
		// RCLCPP_INFO_STREAM(this->get_logger(), "Enqueued motion command
		// pending
		// ?"<< 				(enqueued_motion_pending_?"True":"False"));
		return enqueued_motion_pending_;
	}

	bool RobotInterface::enqeued_motion_timed_out() const
	{
		auto lck = mrpt::lockHelper(enqueued_motion_mutex_);
		// RCLCPP_INFO_STREAM(this->get_logger(), "Enqueued motion command timed
		// out
		// ?"<< 				(enqueued_motion_timeout_?"True":"False"));
		return enqueued_motion_timeout_;
	}

	std::optional<mpp::VehicleOdometryState>
		RobotInterface::enqued_motion_last_odowhen_triggered_() const
	{
		auto lck = mrpt::lockHelper(enqueued_motion_mutex_);
		return enqueued_motion_trigger_odom_;
	}

	bool RobotInterface::changeSpeeds(
		const mrpt::kinematics::CVehicleVelCmd& vel_cmd)
	{
		// RCLCPP_INFO_STREAM(this->get_logger(), "[Jackal_Robot] change
		// speeds");
		using namespace mrpt::kinematics;
		const CVehicleVelCmd_DiffDriven* vel_cmd_diff_driven =
			dynamic_cast<const CVehicleVelCmd_DiffDriven*>(&vel_cmd);
		ASSERT_(vel_cmd_diff_driven);

		const double v = vel_cmd_diff_driven->lin_vel;
		const double w = vel_cmd_diff_driven->ang_vel;
		RCLCPP_INFO_STREAM(
			this->get_logger(),
			"changeSpeeds: v= " << v << "m/s and w=" << w * 180.0f / PI_
								<< "deg/s");
		geometry_msgs::Twist cmd;
		cmd.linear.x = v;
		cmd.angular.z = w;
		parent_.publish_cmd_vel(cmd);
		return true;
	}

	void RobotInterface::stop(const mpp::STOP_TYPE stopType)
	{
		if (stopType == mpp::STOP_TYPE::EMERGENCY)
		{
			const auto cmd = getEmergencyStopCmd();
			enqueued_motion_pending_ = false;
			enqueued_motion_timeout_ = false;
			changeSpeeds(*cmd);
		}
		else
		{
			const auto cmd = getStopCmd();
			enqueued_motion_pending_ = false;
			enqueued_motion_timeout_ = false;
			changeSpeeds(*cmd);
		}
	}

	mrpt::kinematics::CVehicleVelCmd::Ptr RobotInterface::getStopCmd()
	{
		mrpt::kinematics::CVehicleVelCmd::Ptr vel =
			mrpt::kinematics::CVehicleVelCmd::Ptr(
				new mrpt::kinematics::CVehicleVelCmd_DiffDriven);
		vel->setToStop();
		return vel;
	}

	mrpt::kinematics::CVehicleVelCmd::Ptr RobotInterface::getEmergencyStopCmd()
	{
		return getStopCmd();
	}

	void RobotInterface::stop_watchdog()
	{
		RCLCPP_INFO_STREAM(
			parent_.get_logger(), "TPS_Astar_Navigator watchdog timer stopped");
	}

	void RobotInterface::start_watchdog(const size_t periodMilliseconds)
	{
		RCLCPP_INFO_STREAM(
			parent_.get_logger(), "TPS_Astar_Navigator start watchdog timer");
	}

	void RobotInterface::on_nav_end_due_to_error()
	{
		RCLCPP_INFO_STREAM(
			parent_.get_logger(), "[TPS_Astar_Navigator] Nav End due to error");
		stop(mpp::STOP_TYPE::EMERGENCY);
	}

	void RobotInterface::on_nav_start()
	{
		RCLCPP_INFO_STREAM(
			parent_.get_logger(), "[TPS_Astar_Navigator] Nav starting to Goal ="
									  << parent_.nav_goal_.asString());
	}

	void RobotInterface::on_nav_end()
	{
		RCLCPP_INFO_STREAM(
			parent_.get_logger(), "[TPS_Astar_Navigator] Nav End");
		stop(mpp::STOP_TYPE::REGULAR);
	}

	void RobotInterface::on_path_seems_blocked()
	{
		RCLCPP_INFO_STREAM(
			parent_.get_logger(), "[TPS_Astar_Navigator] Path Seems blocked");
		stop(mpp::STOP_TYPE::REGULAR);
	}

	void RobotInterface::on_apparent_collision()
	{
		RCLCPP_INFO_STREAM(
			parent_.get_logger(), "[TPS_Astar_Navigator] Apparent collision");
		// stop(mpp::STOP_TYPE::REGULAR);
	}

	mrpt::maps::CPointsMap::Ptr RobotInterface::obstacles(
		mrpt::system::TTimeStamp t)
	{
		return parent_.get_current_obstacles();
	}

	// ------------------------------------
	int main(int argc, char** argv)
	{
		rclcpp::init(argc, argv);
		auto node = std::make_shared<TPS_Astar_Nav_Node>();
		rclcpp::spin(node);
		rclcpp::shutdown();
		return 0;
	}
