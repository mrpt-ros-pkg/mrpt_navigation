/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/config/CConfigFile.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/kinematics/CVehicleVelCmd_DiffDriven.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/nav/reactive/CReactiveNavigationSystem.h>
#include <mrpt/nav/reactive/TWaypoint.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/ros2bridge/point_cloud2.h>
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/time.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mrpt_msgs/msg/waypoint.hpp>
#include <mrpt_msgs/msg/waypoint_sequence.hpp>
#include <mrpt_nav_interfaces/action/navigate_goal.hpp>
#include <mrpt_nav_interfaces/action/navigate_waypoints.hpp>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace mrpt_reactivenav2d
{
// The ROS node
class ReactiveNav2DNode : public rclcpp::Node
{
   public:
	/* Ctor*/
	explicit ReactiveNav2DNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
	/* Dtor*/
	~ReactiveNav2DNode() {}

   private:
	// methods
	void read_parameters();
	void navigate_to(const mrpt::math::TPose2D& target);
	void on_do_navigation();
	void on_goal_received(const geometry_msgs::msg::PoseStamped::SharedPtr& trg_ptr);
	void on_local_obstacles(const sensor_msgs::msg::PointCloud2::SharedPtr& obs);
	void on_set_robot_shape(const geometry_msgs::msg::Polygon::SharedPtr& newShape);
	void on_odometry_received(const nav_msgs::msg::Odometry::SharedPtr& odom);
	void update_waypoint_sequence(const mrpt_msgs::msg::WaypointSequence& wp);
	void on_waypoint_seq_received(const mrpt_msgs::msg::WaypointSequence::SharedPtr& wps);

   private:
	/** @name ROS pubs/subs
	 *  @{ */
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry_;
	rclcpp::Subscription<mrpt_msgs::msg::WaypointSequence>::SharedPtr subWpSeq_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subNavGoal_;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLocalObs_;
	rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr subRobotShape_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubCmdVel_;

	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubSelectedPtg_;

	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubNavEvents_;

	std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
	std::shared_ptr<tf2_ros::TransformListener> tfListener_;
	/** @} */

	mrpt::system::CTimeLogger profiler_;
	bool initialized_ = false;	//!< Reactive initialization done?

	std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;

	// Used for single goal commands. For waypoints, the node will use
	// the distances in the waypoints msg:
	double targetAllowedDistance_ = 0.40;
	double navPeriod_ = 0.10;

	std::string subTopicNavGoal_ = "reactive_nav_goal";
	std::string subTopicLocalObstacles_ = "local_map_pointcloud";
	std::string subTopicRobotShape_;
	std::string subTopicWpSeq_ = "reactive_nav_waypoints";
	std::string subTopicOdometry_ = "/odom";

	std::string pubTopicCmdVel_ = "/cmd_vel";
	std::string pubTopicSelectedPtg_ = "reactivenav_selected_ptg";
	std::string pubTopicEvents_ = "reactivenav_events";

	std::string frameidReference_ = "map";
	std::string frameidRobot_ = "base_link";

	/// If enabled, no obstacle avoidance will be attempted (!)
	bool pure_pursuit_mode_ = false;
	std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_pure_pursuit_mode_;

	std::string pluginFile_ = {};
	std::string cfgFileReactive_ = "reactive2d_config.ini";

	bool saveNavLog_ = false;

	rclcpp::TimerBase::SharedPtr timerRunNav_;

	mrpt::obs::CObservationOdometry odometry_;
	std::mutex odometryMtx_;

	mrpt::maps::CSimplePointsMap lastObstacles_;
	std::mutex lastObstaclesMtx_;

	bool waitForTransform(
		mrpt::poses::CPose3D& des, const std::string& target_frame, const std::string& source_frame,
		const int timeoutMilliseconds = 50);

	void publish_last_log_record_to_ros(const mrpt::nav::CLogFileRecord& lr);

	visualization_msgs::msg::MarkerArray log_to_margers(const mrpt::nav::CLogFileRecord& lr);

	void publish_event_message(const std::string& text);

	struct MyReactiveInterface : public mrpt::nav::CRobot2NavInterface
	{
		ReactiveNav2DNode& parent_;

		MyReactiveInterface(ReactiveNav2DNode& parent) : parent_(parent) {}

		/** Get the current pose and speeds of the robot.
		 *   \param curPose Current robot pose.
		 *   \param curV Current linear speed, in meters per second.
		 *	 \param curW Current angular speed, in radians per second.
		 * \return false on any error.
		 */
		bool getCurrentPoseAndSpeeds(
			mrpt::math::TPose2D& curPose, mrpt::math::TTwist2D& curVel,
			mrpt::system::TTimeStamp& timestamp, mrpt::math::TPose2D& curOdometry,
			std::string& frame_id) override;

		/** Change the instantaneous speeds of robot.
		 *   \param v Linear speed, in meters per second.
		 *	 \param w Angular speed, in radians per second.
		 * \return false on any error.
		 */
		bool changeSpeeds(const mrpt::kinematics::CVehicleVelCmd& vel_cmd) override;

		bool stop(bool isEmergency) override;

		/** Start the watchdog timer of the robot platform, if any.
		 * \param T_ms Period, in ms.
		 * \return false on any error. */
		bool startWatchdog(float T_ms) override { return true; }

		/** Stop the watchdog timer.
		 * \return false on any error. */
		bool stopWatchdog() override { return true; }

		/** Return the current set of obstacle points.
		 * \return false on any error. */
		bool senseObstacles(
			mrpt::maps::CSimplePointsMap& obstacles, mrpt::system::TTimeStamp& timestamp) override;

		mrpt::kinematics::CVehicleVelCmd::Ptr getEmergencyStopCmd() override
		{
			return getStopCmd();
		}

		mrpt::kinematics::CVehicleVelCmd::Ptr getStopCmd() override
		{
			auto ret = mrpt::kinematics::CVehicleVelCmd_DiffDriven::Create();
			ret->setToStop();
			return ret;
		}

		/** Callback: Start of navigation command */
		void sendNavigationStartEvent() override;

		/** Callback: End of navigation command (reach of single goal, or final
		 * waypoint of waypoint list) */
		void sendNavigationEndEvent() override;

		/** Callback: Reached an intermediary waypoint in waypoint list
		 * navigation. reached_nSkipped will be `true` if the waypoint was
		 * physically reached; `false` if it was actually "skipped".
		 */
		void sendWaypointReachedEvent(int waypoint_index, bool reached_nSkipped) override;

		/** Callback: Heading towards a new intermediary/final waypoint in
		 * waypoint list navigation */
		void sendNewWaypointTargetEvent(int waypoint_index) override;

		/** Callback: Error asking sensory data from robot or sending motor
		 * commands. */
		void sendNavigationEndDueToErrorEvent() override;

		/** Callback: No progression made towards target for a predefined period
		 * of time. */
		void sendWaySeemsBlockedEvent() override;

		/** Callback: Apparent collision event (i.e. there is at least one
		 * obstacle point inside the robot shape) */
		void sendApparentCollisionEvent() override;

		/** Callback: Target seems to be blocked by an obstacle. */
		void sendCannotGetCloserToBlockedTargetEvent() override;
	};

	MyReactiveInterface reactiveInterface_{*this};
	mrpt::nav::CReactiveNavigationSystem rnavEngine_{reactiveInterface_};
	std::mutex rnavEngineMtx_;

	// ACTION INTERFACE: NavigateGoal
	// --------------------------------------
	using NavigateGoal = mrpt_nav_interfaces::action::NavigateGoal;
	using HandleNavigateGoal = rclcpp_action::ServerGoalHandle<NavigateGoal>;

	rclcpp_action::Server<NavigateGoal>::SharedPtr action_server_goal_;

	rclcpp_action::GoalResponse handle_goal(
		const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const NavigateGoal::Goal> goal);

	rclcpp_action::CancelResponse handle_cancel(
		const std::shared_ptr<HandleNavigateGoal> goal_handle);

	void handle_accepted(const std::shared_ptr<HandleNavigateGoal> goal_handle);
	void execute_action_goal(const std::shared_ptr<HandleNavigateGoal> goal_handle);

	std::optional<bool> currentNavEndedSuccessfully_;
	std::mutex currentNavEndedSuccessfullyMtx_;

	// ACTION INTERFACE: NavigateWaypoints
	// --------------------------------------
	using NavigateWaypoints = mrpt_nav_interfaces::action::NavigateWaypoints;
	using HandleNavigateWaypoints = rclcpp_action::ServerGoalHandle<NavigateWaypoints>;

	rclcpp_action::Server<NavigateWaypoints>::SharedPtr action_server_waypoints_;

	rclcpp_action::GoalResponse handle_goal_wp(
		const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const NavigateWaypoints::Goal> goal);

	rclcpp_action::CancelResponse handle_cancel_wp(
		const std::shared_ptr<HandleNavigateWaypoints> goal_handle);

	void handle_accepted_wp(const std::shared_ptr<HandleNavigateWaypoints> goal_handle);
	void execute_action_wp(const std::shared_ptr<HandleNavigateWaypoints> goal_handle);
};

}  // namespace mrpt_reactivenav2d
