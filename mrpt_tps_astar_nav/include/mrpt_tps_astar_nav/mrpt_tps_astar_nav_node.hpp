/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#pragma once
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

const char* NODE_NAME = "mrpt_tps_astar_nav_node";

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

	// JL: Needed? //Causes build errors due to presence in base class
	CObject* clone() const override { return nullptr; }

   private:
	//!< Top level parent class object
	TPS_Astar_Nav_Node& parent_;

	/// Indicator that enqueued command is pending and needs to be executed
	bool enqueued_motion_pending_;

	/// indicator that enqueued command timedout
	bool enqueued_motion_timeout_;

    public: 
    /// odometry when enqueued motion is triggered
    mpp::VehicleOdometryState enqueued_motion_trigger_odom_;
    /// mutex for enqueued commands : how to make it private ?
	std::mutex enqueued_motion_mutex_;	
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

	/// Subscriber to localization info
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
		sub_localization_pose_;

	/// Subscriber to odometry info from robot
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;

	/// Subscriber to obstacle map info
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_obstacles_;

    /// Subscribe to waypoint sequence from planner
	rclcpp::Subscription<mrpt_msgs::msg::WaypointSequence>::SharedPtr sub_wp_seq_;

	/// Subscriber to topic from rnav that tells to replan
	/// TODO(JL): Switch into a service!
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_replan_;

	/// Publisher for velocity commands for the robot
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;

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

    /// Localization subscriber topic name
	std::string topic_localization_sub_;

    /// Odometry subscriber topic name
	std::string topic_odometry_sub_;

    /// Obstacles subscriber topic name
	std::string topic_obstacles_sub_;

    /// Waypoint sequence subscriber topic name
    std::string topic_wp_seq_sub_;

    /// Velocity Command publisher topic name
	std::string topic_cmd_vel_pub_;

	/// Feedback odometry topic name  : SRai22 -> JL required ?
	std::string topic_feedback_odom_pub_;

    /// Parameter file for PTGs
    std::string ptg_ini_file_;

    /// Parameters file for Costmap evaluator
    std::string costmap_params_file_;

    /// Parameters file for waypoints preferences
    std::string wp_params_file_;

    /// Parameters file for planner
    std::string planner_params_file_;

    /// Parameters file for A-star navigation engine
    std::string nav_engine_params_file_;

	/// Pointer to MRPT 3D display window
	mrpt::gui::CDisplayWindow3D::Ptr win_3d_;

	/// MRPT OpenGL scene
	mrpt::opengl::COpenGLScene scene_;

	/// Callback for robot when enqueued command fired
	std::function<void()> on_enqueued_motion_fired;

	/// Callback for robot when enqueued command timed out
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

    /// Robot object that acts as Interface between the engine and robot
	std::shared_ptr<RobotInterface> robot_;

    /// TPS A-star Navigation Engine
	std::shared_ptr<mpp::NavEngine> nav_engine_;

	/// To be used
	mpp::WaypointSequence waypts_;	//<! DS for waypoints
	mpp::WaypointStatusSequence wayptsStatus_;	//<! DS for waypoint status

   private:
    /**
     * @brief Reads a parameter from the node's parameter server.
     *
     * This function attempts to retrieve parameters and assign it to class member vars.
    */
	void read_parameters();

    /**
     * @brief Callback when a waypoint sequence is received from planner
     * @param _wps Waypoint sequence pointer
    */
    void callback_waypoint_seq
        (const mrpt_msgs::msg::WaypointSequence::SharedPtr& _wps);

    /**
     * @brief Callback when received obstacles map around the robot
     * @param _pc PointCloud pointer
    */
	void callback_obstacles(const sensor_msgs::msg::PointCloud2::SharedPtr& _pc);

	/** 
     * @brief Mutex locked method to update localization info from sensor 
     * @param _pose PoseWithCovarianceStamped
    */
	void update_localization(
		const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& _pose);

    /**
     * @brief Mutex locked method to update odometry info from sensor 
     * @param _odom Odometry object
    */
	void update_odometry(const nav_msgs::msg::Odometry::SharedPtr& _odom);

    /**
     * @brief Mutex locked method to update local obstacle map
     * @param _pc PointCloud2 object
    */
	void update_obstacles(const sensor_msgs::msg::PointCloud2::SharedPtr& _pc);

	// bool do_path_plan(mrpt::math::TPose2D& start, mrpt::math::TPose2D& goal);
	// void init_3d_debug();

	/**
     * @brief Get localization state called by Robot interface method
     * @return Vehicle Localization state
    */
	mpp::VehicleLocalizationState get_localization_state();

    /**
     * @brief Get odometry state called by Robot interface method
     * @return Vehicle odometry state
    */
	mpp::VehicleOdometryState get_odometry_state();

    /**
     * @brief Get current bstacles around robot called by Robot interface method
     * @return mrpt points map
    */
	mrpt::maps::CPointsMap::Ptr get_current_obstacles();


    /**
     * @brief Method to publish velocity command to the robot 
     * @param cmd_vel Twist command
    */
	void publish_cmd_vel(const geometry_msgs::msg::Twist& cmd_vel);


	//void publish_waypoint_sequence(const mrpt_msgs::msg::WaypointSequence& wps);

	/* Navigator methods*/
    /**
     * @brief Initialize the Navigation Engine with required parameters
    */
	void initialize_navigator();

    /**
     * @brief Method to move the robot to the given target 
     * @param target Pose2D object
    */
	void navigate_to(const mrpt::math::TPose2D& target);

    /**
     * @brief Timer based navigation step method
    */
	void on_do_navigation();

    /**
     * @brief Check if there are motion commands queued up
    */
	void check_enqueued_motion_cmds();
};
