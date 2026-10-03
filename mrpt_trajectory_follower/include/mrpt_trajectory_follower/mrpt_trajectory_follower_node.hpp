/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2026, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#pragma once

// This node is built on mpp::TrajectoryFollower, whose header was added in a
// newer mrpt_path_planning. Guard the whole translation unit on its presence so
// the package still builds against older mpp versions (as a stub that errors at
// runtime) instead of failing the whole workspace build.
#if __has_include(<mpp/algos/TrajectoryFollower.h>)

#include <mpp/algos/TrajectoryFollower.h>
#include <mpp/data/TrajectoriesAndRobotShape.h>
#include <mpp/data/robot_shape_sampling.h>
#include <mpp/interfaces/TrajectoryVehicleInterface.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/ros2bridge/point_cloud2.h>
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/time.h>
#include <mrpt/system/filesystem.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// The collision guard is only available in newer mrpt_path_planning versions:
#if __has_include(<mpp/algos/CollisionGuard.h>)
#include <mpp/algos/CollisionGuard.h>
#define HAVE_MPP_COLLISION_GUARD 1
#else
#define HAVE_MPP_COLLISION_GUARD 0
#endif

#include <atomic>
#include <chrono>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mrpt_nav_interfaces/srv/make_plan_to.hpp>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sstream>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mrpt_trajectory_follower
{
/** ROS 2 node wrapping mpp::TrajectoryFollower.
 *
 * Runs the follower's outer control loop at its configured control_period:
 * pulls the map-frame localization (from TF) and odometry (from a /odom topic),
 * calls step(), and drives the platform via the TrajectoryVehicleInterface it
 * implements (feedforward cmd_vel of the immediate chunk sample, with a
 * watchdog that zeroes cmd_vel if the loop stalls). Obstacles and the reference
 * path arrive on topics.
 *
 * Safety layers:
 * - The follower's own predictive safety (map frame, along the path).
 * - A last-resort mpp::CollisionGuard on every published command, which only
 *   uses the latest sensed obstacles in the robot frame, so it does not
 *   depend on localization or the path.
 * - If the robot leaves the path, or stays blocked, it is stopped and the
 *   path dropped; optionally, a new plan to the same goal is requested.
 */
class TrajectoryFollowerNode : public rclcpp::Node, public mpp::TrajectoryVehicleInterface
{
   public:
	explicit TrajectoryFollowerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
	~TrajectoryFollowerNode() override = default;

	// --- TrajectoryVehicleInterface ---
	mpp::VehicleLocalizationState get_localization() override;
	mpp::VehicleOdometryState get_odometry() override;
	void follow(const mpp::SampledTrajectory& ref) override;
	void stop(mpp::StopKind kind) override;
	void start_watchdog(std::chrono::milliseconds timeout) override;

   private:
	mpp::TrajectoryFollower follower_;
	std::mutex follower_cs_;  //!< guards follower_ (step vs. set*)

#if HAVE_MPP_COLLISION_GUARD
	mpp::CollisionGuard guard_;
	std::mutex guard_cs_;
#endif
	bool guard_enabled_ = false;  //!< guard active (needs obstacle data)

	/// Since when the guard holds the robot stopped (to detect blockages)
	std::optional<rclcpp::Time> guard_stopped_since_;

	// tf2:
	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	// Subs / pubs:
	rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_obstacles_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_speed_limit_;
	rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr sub_robot_shape_;
	rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_chunk_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_ref_path_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;

	rclcpp::Client<mrpt_nav_interfaces::srv::MakePlanTo>::SharedPtr plan_client_;

	rclcpp::TimerBase::SharedPtr control_timer_;
	rclcpp::TimerBase::SharedPtr watchdog_timer_;

	// Latest odometry:
	std::mutex odom_cs_;
	nav_msgs::msg::Odometry last_odom_;
	bool have_odom_ = false;

	// Watchdog:
	std::mutex wd_cs_;
	rclcpp::Time last_cmd_time_;
	std::chrono::milliseconds wd_timeout_{0};

	// True only while actively emitting a driving command (status Running). Used
	// so the node never spams zero cmd_vel when idle or after ReachedGoal/Blocked
	// -- otherwise a downstream twist_mux would treat this node as a permanently
	// active input and fight other cmd_vel sources (e.g. teleop). We publish one
	// clean zero on the transition to a stopped state, then stay silent until a
	// new reference path arrives.
	std::atomic<bool> actively_driving_{false};

	// Params:
	std::string frame_id_map_ = "map";
	std::string frame_id_robot_ = "base_link";
	std::string topic_path_sub_ = "/waypoints_path";
	std::string topic_obstacles_sub_ = "";
	std::string topic_odom_sub_ = "/odom";
	std::string topic_cmd_vel_pub_ = "/cmd_vel";
	std::string ptg_ini_file_ = "";
	std::string follower_params_file_ = "";
	std::string follower_params_overrides_file_ = "";

	// Run-time speed limit [m/s] (<=0: none). The effective max speed is the
	// minimum of this and the platform max_speed from the parameters file.
	double speed_limit_ = 0.0;
	double platform_max_speed_ = 0.0;
	void apply_speed_limit(double limit);

	double robot_radius_ = 0.0;	 //!< footprint fallback when no ptg_ini given

	// Obstacle cloud height band (relative to the robot base frame). Points
	// outside are dropped before the 2D safety checks, so a raw 3D lidar can
	// be used as an obstacle source without its ground/overhead returns
	// causing false stops. Defaults are permissive (effectively disabled); set
	// them for a 3D lidar.
	double obstacle_z_min_ = -1e6;
	double obstacle_z_max_ = 1e6;

	// Drop obstacle returns within this radius [m] of the robot base (removes
	// the robot's own body seen by a 3D lidar). <=0 disables the self-filter.
	double self_filter_radius_ = 0.0;

	// Raises follower_'s COutputLogger verbosity to LVL_DEBUG so its internal
	// per-cycle trace (lookahead, curvature, speed-cap breakdown, safety
	// scale, ...) is printed to stdout. Off by default (noisy).
	bool follower_debug_trace_ = false;

	// Replanning on failure (off-path, blocked):
	bool replan_on_failure_ = false;
	int max_replan_attempts_ = 3;
	std::string planner_service_ = "/mrpt_tps_astar_planner_node/make_plan_to";

	// Footprint consistency check against the planner's one:
	std::string topic_robot_shape_sub_ = "/mrpt_tps_astar_planner_node/robot_shape";
	mrpt::math::TPolygon2D own_shape_;
	bool shape_mismatch_ = false;
	void callback_robot_shape(const geometry_msgs::msg::PolygonStamped& msg);
	int replan_attempts_ = 0;  //!< since the last externally given path
	bool replan_pending_ = false;

	mpp::TrajectoriesAndRobotShape ptgs_;

	mpp::Trajectory last_trajectory_;  //!< to ignore identical re-published paths
	bool have_trajectory_ = false;

	std::string status_;  //!< last published status
	rclcpp::Time last_idle_status_pub_;

	void read_parameters();
	void control_tick();
	void callback_path(const nav_msgs::msg::Path& msg);
	void callback_obstacles(const sensor_msgs::msg::PointCloud2::SharedPtr& pc);
	void callback_odom(const nav_msgs::msg::Odometry::SharedPtr& msg);

	/// Starts following a new reference path (caller must hold follower_cs_)
	void set_reference_path(const mpp::Trajectory& tr);

	/// Stops the robot and drops the current path (caller must hold
	/// follower_cs_)
	void abort_navigation(mpp::StopKind kind);

	/// Called on unrecoverable tracking failures: stops, drops the path, and
	/// requests a new plan if enabled. Caller must hold follower_cs_.
	void on_navigation_failure(const std::string& reason);

	void publish_status(const std::string& s);

	[[nodiscard]] bool wait_for_transform(
		mrpt::poses::CPose3D& des, const std::string& target_frame, const std::string& source_frame,
		int timeout_milliseconds = 50);

	void publish_cmd(double vx, double omega);
	void publish_chunk(const mpp::SampledTrajectory& ref);
};

}  // namespace mrpt_trajectory_follower

#endif	// __has_include(<mpp/algos/TrajectoryFollower.h>)
