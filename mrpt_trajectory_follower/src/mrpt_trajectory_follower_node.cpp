/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2026, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

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

namespace
{
const char* NODE_NAME = "mrpt_trajectory_follower_node";

const char* to_string(mpp::FollowerStatus s)
{
	switch (s)
	{
		case mpp::FollowerStatus::Idle:
			return "Idle";
		case mpp::FollowerStatus::Running:
			return "Running";
		case mpp::FollowerStatus::ReachedGoal:
			return "ReachedGoal";
		case mpp::FollowerStatus::Blocked:
			return "Blocked";
		case mpp::FollowerStatus::OffPathExceeded:
			return "OffPathExceeded";
		case mpp::FollowerStatus::MissedGoal:
			return "MissedGoal";
	}
	return "?";
}

std::string to_string(const mrpt::math::TPolygon2D& poly)
{
	std::stringstream ss;
	ss << "[";
	for (const auto& p : poly)
	{
		ss << " (" << p.x << "," << p.y << ")";
	}
	ss << " ]";
	return ss.str();
}

// Status strings published by this node, in addition to the follower ones:
const char* STATUS_CANCELED = "Canceled";
const char* STATUS_REPLANNING = "Replanning";
const char* STATUS_FAILED = "Failed";
}  // namespace

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
	TrajectoryFollowerNode();
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

TrajectoryFollowerNode::TrajectoryFollowerNode()
	: rclcpp::Node(NODE_NAME), last_cmd_time_(this->now()), last_idle_status_pub_(this->now())
{
	read_parameters();

	follower_.setMinLoggingLevel(
		follower_debug_trace_ ? mrpt::system::LVL_DEBUG : mrpt::system::LVL_INFO);

	tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	// Load follower params (optional), then robot-specific overrides (only
	// the keys present in the overrides file are changed):
	// collision_guard sections, applied in order (later keys override):
	std::vector<mrpt::containers::yaml> guardParams;
	auto loadParams = [&](const std::string& file)
	{
		ASSERT_FILE_EXISTS_(file);
		const auto y = mrpt::containers::yaml::FromFile(file);
		follower_.params.load_from_yaml(y);
		if (y.has("collision_guard"))
		{
			guardParams.emplace_back(y["collision_guard"].node());
		}
	};
	if (!follower_params_file_.empty())
	{
		loadParams(follower_params_file_);
	}
	if (!follower_params_overrides_file_.empty())
	{
		loadParams(follower_params_overrides_file_);
	}
	platform_max_speed_ = follower_.params.max_speed;
	apply_speed_limit(speed_limit_);
	RCLCPP_INFO_STREAM(get_logger(), "Follower params:\n" << follower_.params.as_yaml());

	// Load robot footprint from the PTG ini (optional but recommended so the
	// safety layers use the real robot shape):
	mpp::RobotShape robotShape;
	if (!ptg_ini_file_.empty())
	{
		ASSERT_FILE_EXISTS_(ptg_ini_file_);
		mrpt::config::CConfigFile cfg(ptg_ini_file_);
		// Only the robot description is needed (no PTG collision grids):
		ptgs_.initFromConfigFile(cfg, "SelfDriving", false /*initializePTGs*/);
		robotShape = ptgs_.robotShape;
		RCLCPP_INFO(get_logger(), "Loaded robot shape from '%s'.", ptg_ini_file_.c_str());

		// The vehicle min turning radius is part of the robot description:
		if (ptgs_.minTurningRadius > 0)
		{
			if (follower_.params.min_turn_radius > 0 &&
				std::abs(follower_.params.min_turn_radius - ptgs_.minTurningRadius) > 1e-3)
			{
				RCLCPP_WARN(
					get_logger(),
					"Ignoring follower min_turn_radius=%.3f: using "
					"RobotModel_min_turning_radius=%.3f from '%s'.",
					follower_.params.min_turn_radius, ptgs_.minTurningRadius,
					ptg_ini_file_.c_str());
			}
			follower_.params.min_turn_radius = ptgs_.minTurningRadius;
			RCLCPP_INFO(
				get_logger(), "min_turn_radius: %.3f m (from robot description)",
				follower_.params.min_turn_radius);
		}
	}
	else if (robot_radius_ > 0)
	{
		robotShape = mpp::robot_radius_t{robot_radius_};
		RCLCPP_INFO(get_logger(), "Using circular footprint, radius %.3f m.", robot_radius_);
	}
	else
	{
		robotShape = std::monostate();
		RCLCPP_WARN(
			get_logger(),
			"No 'ptg_ini' or 'robot_radius' given: safety checks will use "
			"the reference point only (no footprint).");
	}
	follower_.setRobotShape(robotShape);
	own_shape_ = mpp::robotShapeAsPolygon(robotShape);

	// Collision guard:
	if (guard_enabled_ && topic_obstacles_sub_.empty())
	{
		RCLCPP_WARN(
			get_logger(),
			"collision_guard is enabled, but no 'topic_obstacles_sub' is "
			"given: disabling it. Commands will NOT be checked against sensed "
			"obstacles.");
		guard_enabled_ = false;
	}
#if HAVE_MPP_COLLISION_GUARD
	if (guard_enabled_)
	{
		for (const auto& gp : guardParams)
		{
			guard_.params.load_from_yaml(gp);
		}
		guard_.setRobotShape(robotShape);
		RCLCPP_INFO_STREAM(get_logger(), "Collision guard params:\n" << guard_.params.as_yaml());
	}
#else
	if (guard_enabled_)
	{
		RCLCPP_WARN(
			get_logger(),
			"collision_guard requested, but this build of mrpt_path_planning "
			"lacks mpp::CollisionGuard: disabling it.");
		guard_enabled_ = false;
	}
#endif

	const auto qos = rclcpp::SystemDefaultsQoS();
	const auto latchedQoS = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

	sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
		topic_path_sub_, qos, [this](const nav_msgs::msg::Path& msg) { this->callback_path(msg); });

	sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
		topic_odom_sub_, qos,
		[this](const nav_msgs::msg::Odometry::SharedPtr msg) { this->callback_odom(msg); });

	if (!topic_obstacles_sub_.empty())
	{
		sub_obstacles_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
			topic_obstacles_sub_, qos,
			[this](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
			{ this->callback_obstacles(msg); });
	}

	sub_speed_limit_ = this->create_subscription<std_msgs::msg::Float64>(
		"~/speed_limit", qos,
		[this](const std_msgs::msg::Float64& msg)
		{
			auto lck = std::lock_guard(follower_cs_);
			apply_speed_limit(msg.data);
		});

	// The speed limit can also be changed with "ros2 param set":
	param_cb_handle_ = this->add_on_set_parameters_callback(
		[this](const std::vector<rclcpp::Parameter>& params)
		{
			rcl_interfaces::msg::SetParametersResult result;
			result.successful = true;
			for (const auto& p : params)
			{
				if (p.get_name() == "speed_limit")
				{
					auto lck = std::lock_guard(follower_cs_);
					apply_speed_limit(p.as_double());
				}
			}
			return result;
		});

	if (!topic_robot_shape_sub_.empty())
	{
		sub_robot_shape_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
			topic_robot_shape_sub_, latchedQoS,
			[this](const geometry_msgs::msg::PolygonStamped& msg)
			{ this->callback_robot_shape(msg); });
	}

	pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_cmd_vel_pub_, qos);
	pub_chunk_ = this->create_publisher<nav_msgs::msg::Path>("~/predicted_trajectory", qos);
	pub_ref_path_ = this->create_publisher<nav_msgs::msg::Path>("~/reference_path", latchedQoS);
	pub_status_ = this->create_publisher<std_msgs::msg::String>("~/status", latchedQoS);

	if (replan_on_failure_)
	{
		plan_client_ = this->create_client<mrpt_nav_interfaces::srv::MakePlanTo>(planner_service_);
	}

	// Control loop at the follower's control period:
	const auto period = std::chrono::duration<double>(follower_.params.control_period);
	control_timer_ = this->create_wall_timer(
		std::chrono::duration_cast<std::chrono::nanoseconds>(period),
		[this]() { this->control_tick(); });

	// Watchdog: a few control periods without a fresh command -> zero cmd_vel.
	start_watchdog(std::chrono::milliseconds(
		std::max<int>(200, static_cast<int>(5000 * follower_.params.control_period))));

	publish_status(to_string(mpp::FollowerStatus::Idle));

	RCLCPP_INFO(get_logger(), "%s initialized.", NODE_NAME);
}

void TrajectoryFollowerNode::read_parameters()
{
	auto p = [&](const std::string& name, std::string& var)
	{
		this->declare_parameter<std::string>(name, var);
		this->get_parameter(name, var);
		RCLCPP_INFO(get_logger(), "%s: %s", name.c_str(), var.c_str());
	};
	p("frame_id_map", frame_id_map_);
	p("frame_id_robot", frame_id_robot_);
	p("topic_path_sub", topic_path_sub_);
	p("topic_obstacles_sub", topic_obstacles_sub_);
	p("topic_odom_sub", topic_odom_sub_);
	p("topic_cmd_vel_pub", topic_cmd_vel_pub_);
	p("ptg_ini", ptg_ini_file_);
	p("follower_parameters", follower_params_file_);
	p("follower_parameters_overrides", follower_params_overrides_file_);
	p("planner_service", planner_service_);
	p("topic_robot_shape_sub", topic_robot_shape_sub_);

	this->declare_parameter<double>("speed_limit", speed_limit_);
	this->get_parameter("speed_limit", speed_limit_);
	RCLCPP_INFO(get_logger(), "speed_limit: %.3f m/s", speed_limit_);

	this->declare_parameter<double>("robot_radius", robot_radius_);
	this->get_parameter("robot_radius", robot_radius_);
	RCLCPP_INFO(get_logger(), "robot_radius: %.3f", robot_radius_);

	this->declare_parameter<double>("obstacle_z_min", obstacle_z_min_);
	this->get_parameter("obstacle_z_min", obstacle_z_min_);
	this->declare_parameter<double>("obstacle_z_max", obstacle_z_max_);
	this->get_parameter("obstacle_z_max", obstacle_z_max_);
	RCLCPP_INFO(
		get_logger(), "obstacle_z band (robot frame): [%.2f, %.2f] m", obstacle_z_min_,
		obstacle_z_max_);

	this->declare_parameter<double>("self_filter_radius", self_filter_radius_);
	this->get_parameter("self_filter_radius", self_filter_radius_);
	RCLCPP_INFO(get_logger(), "self_filter_radius: %.3f m", self_filter_radius_);

	this->declare_parameter<bool>("follower_debug_trace", follower_debug_trace_);
	this->get_parameter("follower_debug_trace", follower_debug_trace_);
	RCLCPP_INFO(get_logger(), "follower_debug_trace: %s", follower_debug_trace_ ? "true" : "false");

	this->declare_parameter<bool>("collision_guard", true);
	this->get_parameter("collision_guard", guard_enabled_);
	RCLCPP_INFO(get_logger(), "collision_guard: %s", guard_enabled_ ? "true" : "false");

	this->declare_parameter<bool>("replan_on_failure", replan_on_failure_);
	this->get_parameter("replan_on_failure", replan_on_failure_);
	this->declare_parameter<int>("max_replan_attempts", max_replan_attempts_);
	this->get_parameter("max_replan_attempts", max_replan_attempts_);
	RCLCPP_INFO(
		get_logger(), "replan_on_failure: %s (max attempts: %d)",
		replan_on_failure_ ? "true" : "false", max_replan_attempts_);
}

void TrajectoryFollowerNode::apply_speed_limit(double limit)
{
	speed_limit_ = limit;
	const double prev = follower_.params.max_speed;
	follower_.params.max_speed =
		limit > 0 ? std::min(limit, platform_max_speed_) : platform_max_speed_;
	if (prev != follower_.params.max_speed)
	{
		RCLCPP_INFO(
			get_logger(), "Effective max speed: %.2f m/s (limit=%.2f, platform max=%.2f)",
			follower_.params.max_speed, limit, platform_max_speed_);
	}
}

bool TrajectoryFollowerNode::wait_for_transform(
	mrpt::poses::CPose3D& des, const std::string& target_frame, const std::string& source_frame,
	int timeout_milliseconds)
{
	const rclcpp::Duration timeout(0, 1000000LL * timeout_milliseconds);
	try
	{
		geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
			source_frame, target_frame, tf2::TimePointZero,
			tf2::durationFromSec(timeout.seconds()));
		tf2::Transform t;
		tf2::fromMsg(tf.transform, t);
		des = mrpt::ros2bridge::fromROS(t);
		return true;
	}
	catch (const tf2::TransformException& ex)
	{
		RCLCPP_ERROR_THROTTLE(
			get_logger(), *get_clock(), 5000, "[wait_for_transform] %s", ex.what());
		return false;
	}
}

// --------------------------- TrajectoryVehicleInterface ---------------------
mpp::VehicleLocalizationState TrajectoryFollowerNode::get_localization()
{
	mpp::VehicleLocalizationState st;
	mrpt::poses::CPose3D p;
	if (wait_for_transform(p, frame_id_robot_, frame_id_map_))
	{
		st.valid = true;
		st.pose = mrpt::poses::CPose2D(p).asTPose();
		st.frame_id = frame_id_map_;
		st.timestamp = mrpt::Clock::now();
	}
	return st;
}

mpp::VehicleOdometryState TrajectoryFollowerNode::get_odometry()
{
	mpp::VehicleOdometryState st;
	auto lck = std::lock_guard(odom_cs_);
	if (!have_odom_)
	{
		return st;
	}

	st.valid = true;
	st.odometry = mrpt::poses::CPose2D(mrpt::ros2bridge::fromROS(last_odom_.pose.pose)).asTPose();
	st.odometryVelocityLocal = mrpt::math::TTwist2D(
		last_odom_.twist.twist.linear.x, last_odom_.twist.twist.linear.y,
		last_odom_.twist.twist.angular.z);
	st.timestamp = mrpt::ros2bridge::fromROS(last_odom_.header.stamp);
	return st;
}

void TrajectoryFollowerNode::follow(const mpp::SampledTrajectory& ref)
{
	double vx = 0;
	double omega = 0;
	if (!ref.empty())
	{
		const auto& tw = ref.points.front().twist;
		vx = tw.vx;
		omega = tw.omega;
	}

#if HAVE_MPP_COLLISION_GUARD
	if (guard_enabled_)
	{
		std::optional<mrpt::math::TTwist2D> curVel;
		{
			auto lck = std::lock_guard(odom_cs_);
			if (have_odom_)
			{
				curVel = mrpt::math::TTwist2D(
					last_odom_.twist.twist.linear.x, 0, last_odom_.twist.twist.angular.z);
			}
		}
		mpp::CollisionGuard::Result r;
		{
			auto lck = std::lock_guard(guard_cs_);
			r = guard_.filter(vx, omega, mrpt::ros2bridge::fromROS(this->now()), curVel);
		}
		if (r.limited)
		{
			RCLCPP_WARN_THROTTLE(
				get_logger(), *get_clock(), 1000,
				"Collision guard: (v,w)=(%.2f,%.2f) -> (%.2f,%.2f) free_travel=%.2f "
				"limiting_point=%s%s%s%s",
				vx, omega, r.v, r.omega, r.free_travel,
				r.limiting_point ? r.limiting_point->asString().c_str() : "none",
				r.current_motion_unsafe ? " [CURRENT MOTION UNSAFE]" : "",
				r.stale ? " [STALE OBSTACLE DATA]" : "", r.in_contact ? " [IN CONTACT]" : "");
		}
		// Held by the guard: stopped, or only allowed to crawl (it slows down
		// smoothly toward obstacles), which would never end the navigation:
		constexpr double kCrawlSpeed = 0.05;  // [m/s]
		constexpr double kCrawlOmega = 0.05;  // [rad/s]
		const bool stoppedByGuard =
			r.limited && std::abs(r.v) < kCrawlSpeed && std::abs(r.omega) < kCrawlOmega;
		if (stoppedByGuard)
		{
			if (!guard_stopped_since_)
			{
				guard_stopped_since_ = this->now();
			}
		}
		else
		{
			guard_stopped_since_.reset();
		}
		vx = r.v;
		omega = r.omega;
	}
#endif

	publish_cmd(vx, omega);
	publish_chunk(ref);
}

void TrajectoryFollowerNode::stop(mpp::StopKind /*kind*/) { publish_cmd(0, 0); }

void TrajectoryFollowerNode::start_watchdog(std::chrono::milliseconds timeout)
{
	{
		auto lck = std::lock_guard(wd_cs_);
		wd_timeout_ = timeout;
		last_cmd_time_ = this->now();
	}
	if (watchdog_timer_)
	{
		return;
	}

	// Check at a fraction of the timeout so a stall is caught promptly.
	const auto checkPeriod = std::max<int64_t>(50, timeout.count() / 4);
	watchdog_timer_ = this->create_wall_timer(
		std::chrono::milliseconds(checkPeriod),
		[this]()
		{
			// Only guard an actively driving loop: when idle or already stopped,
			// staying silent lets a downstream mux time this input out instead of
			// us fighting other cmd_vel sources with a stream of zeros.
			if (!actively_driving_.load())
			{
				return;
			}
			auto lck = std::lock_guard(wd_cs_);
			if (wd_timeout_.count() <= 0)
			{
				return;
			}
			if ((this->now() - last_cmd_time_) > rclcpp::Duration(wd_timeout_))
			{
				// Stalled: fail safe. Publish directly (avoid re-entering the
				// watchdog's own bookkeeping).
				geometry_msgs::msg::Twist zero;
				pub_cmd_vel_->publish(zero);
			}
		});
}

// --------------------------------- helpers ----------------------------------
void TrajectoryFollowerNode::publish_cmd(double vx, double omega)
{
	geometry_msgs::msg::Twist cmd;
	cmd.linear.x = vx;
	cmd.angular.z = omega;
	pub_cmd_vel_->publish(cmd);

	auto lck = std::lock_guard(wd_cs_);
	last_cmd_time_ = this->now();
}

void TrajectoryFollowerNode::publish_chunk(const mpp::SampledTrajectory& ref)
{
	if (pub_chunk_->get_subscription_count() == 0)
	{
		return;
	}
	nav_msgs::msg::Path path;
	path.header.frame_id = ref.frame_id.empty() ? frame_id_map_ : ref.frame_id;
	path.header.stamp = this->now();
	for (const auto& s : ref.points)
	{
		auto& q = path.poses.emplace_back();
		q.header = path.header;
		q.pose = mrpt::ros2bridge::toROS_Pose(s.pose);
	}
	pub_chunk_->publish(path);
}

void TrajectoryFollowerNode::publish_status(const std::string& s)
{
	if (s != status_)
	{
		if (s == to_string(mpp::FollowerStatus::OffPathExceeded) ||
			s == to_string(mpp::FollowerStatus::MissedGoal) ||
			s == to_string(mpp::FollowerStatus::Blocked) || s == STATUS_FAILED)
		{
			RCLCPP_WARN(get_logger(), "Status: %s -> %s", status_.c_str(), s.c_str());
		}
		else
		{
			RCLCPP_INFO(get_logger(), "Status: %s -> %s", status_.c_str(), s.c_str());
		}
		status_ = s;
	}
	std_msgs::msg::String sm;
	sm.data = s;
	pub_status_->publish(sm);
}

void TrajectoryFollowerNode::set_reference_path(const mpp::Trajectory& tr)
{
	follower_.setTrajectory(tr);
	last_trajectory_ = tr;
	have_trajectory_ = true;
	guard_stopped_since_.reset();

	nav_msgs::msg::Path p;
	p.header.frame_id = frame_id_map_;
	p.header.stamp = this->now();
	for (const auto& pt : tr)
	{
		auto& q = p.poses.emplace_back();
		q.header = p.header;
		q.pose = mrpt::ros2bridge::toROS_Pose(pt.pose);
	}
	pub_ref_path_->publish(p);
}

void TrajectoryFollowerNode::abort_navigation(mpp::StopKind kind)
{
	if (actively_driving_.exchange(false))
	{
		stop(kind);
	}
	follower_.reset();
	have_trajectory_ = false;
	guard_stopped_since_.reset();
}

void TrajectoryFollowerNode::on_navigation_failure(const std::string& reason)
{
	// Keep the goal before dropping the path:
	const auto goal =
		last_trajectory_.empty() ? mrpt::math::TPose2D() : last_trajectory_.back().pose;
	const bool haveGoal = !last_trajectory_.empty();

	abort_navigation(mpp::StopKind::EMERGENCY);
	publish_status(reason);

	if (!replan_on_failure_ || !haveGoal)
	{
		return;
	}
	if (replan_attempts_ >= max_replan_attempts_)
	{
		RCLCPP_ERROR(
			get_logger(), "Giving up navigation after %d replan attempts.", replan_attempts_);
		publish_status(STATUS_FAILED);
		return;
	}
	if (!plan_client_ || !plan_client_->service_is_ready())
	{
		RCLCPP_ERROR(
			get_logger(), "Cannot replan: service '%s' not available.", planner_service_.c_str());
		publish_status(STATUS_FAILED);
		return;
	}

	replan_attempts_++;
	replan_pending_ = true;
	RCLCPP_WARN(
		get_logger(), "Requesting a new plan to (%.2f, %.2f, %.1f deg), attempt %d/%d", goal.x,
		goal.y, mrpt::RAD2DEG(goal.phi), replan_attempts_, max_replan_attempts_);
	publish_status(STATUS_REPLANNING);

	auto req = std::make_shared<mrpt_nav_interfaces::srv::MakePlanTo::Request>();
	req->target.header.frame_id = frame_id_map_;
	req->target.header.stamp = this->now();
	req->target.pose = mrpt::ros2bridge::toROS_Pose(goal);

	plan_client_->async_send_request(
		req,
		[this](rclcpp::Client<mrpt_nav_interfaces::srv::MakePlanTo>::SharedFuture future)
		{
			auto lck = std::lock_guard(follower_cs_);
			if (!replan_pending_)
			{
				return;	 // superseded by a new external path
			}
			replan_pending_ = false;
			const auto resp = future.get();
			mpp::Trajectory tr;
			for (const auto& wp : resp->waypoints.waypoints)
			{
				tr.emplace_back(
					mrpt::poses::CPose2D(mrpt::ros2bridge::fromROS(wp.target)).asTPose(), 0.0);
			}
			if (!resp->valid_path_found || tr.size() < 2)
			{
				RCLCPP_ERROR(get_logger(), "Replanning failed: no valid path found.");
				publish_status(STATUS_FAILED);
				return;
			}
			RCLCPP_INFO(get_logger(), "Replanned path with %zu poses.", tr.size());
			set_reference_path(tr);
		});
}

// -------------------------------- callbacks ---------------------------------
void TrajectoryFollowerNode::callback_path(const nav_msgs::msg::Path& msg)
{
	if (!msg.header.frame_id.empty() && msg.header.frame_id != frame_id_map_)
	{
		RCLCPP_WARN_THROTTLE(
			get_logger(), *get_clock(), 5000,
			"Reference path frame '%s' != map frame '%s'; assuming map coordinates.",
			msg.header.frame_id.c_str(), frame_id_map_.c_str());
	}

	mpp::Trajectory tr;
	tr.reserve(msg.poses.size());
	for (const auto& ps : msg.poses)
	{
		const auto pose = mrpt::poses::CPose2D(mrpt::ros2bridge::fromROS(ps.pose)).asTPose();
		tr.emplace_back(pose, 0.0 /* <=0 => use follower max_speed */);
	}

	auto lck = std::lock_guard(follower_cs_);

	if (shape_mismatch_)
	{
		RCLCPP_ERROR(
			get_logger(),
			"Ignoring reference path: robot footprint differs from the planner one "
			"(see '%s').",
			topic_robot_shape_sub_.c_str());
		publish_status(STATUS_FAILED);
		return;
	}

	if (tr.size() < 2)
	{
		// An empty path cancels the current navigation (e.g. the planner
		// could not find a path to a new goal): never keep executing a stale
		// one.
		RCLCPP_WARN(
			get_logger(), "Received a path with %zu poses: canceling navigation.", tr.size());
		abort_navigation(mpp::StopKind::REGULAR);
		replan_pending_ = false;
		last_trajectory_.clear();
		publish_status(STATUS_CANCELED);
		return;
	}

	// Ignore a re-published identical path (e.g. from a transient_local/latched
	// publisher): resetting the follower would restart its speed profile from
	// zero and prevent it from ever accelerating.
	if (have_trajectory_ && tr.size() == last_trajectory_.size())
	{
		bool same = true;
		for (std::size_t i = 0; i < tr.size(); i++)
		{
			const auto& a = tr[i].pose;
			const auto& b = last_trajectory_[i].pose;
			if (std::abs(a.x - b.x) > 1e-3 || std::abs(a.y - b.y) > 1e-3 ||
				std::abs(mrpt::math::wrapToPi(a.phi - b.phi)) > 1e-3)
			{
				same = false;
				break;
			}
		}
		if (same)
		{
			return;
		}
	}

	// A new path from outside: reset the replanning state.
	replan_attempts_ = 0;
	replan_pending_ = false;

	set_reference_path(tr);
	RCLCPP_INFO(get_logger(), "New reference path with %zu poses.", tr.size());
}

void TrajectoryFollowerNode::callback_obstacles(
	const sensor_msgs::msg::PointCloud2::SharedPtr& pcMsg)
{
	auto pc = mrpt::maps::CSimplePointsMap::Create();
	if (!mrpt::ros2bridge::fromROS(*pcMsg, *pc))
	{
		RCLCPP_ERROR(get_logger(), "Failed to convert PointCloud2 to MRPT points map.");
		return;
	}

	// Express the points in the robot frame (do the possibly-blocking TF
	// lookups before taking the locks). Obstacles are often already published
	// in the robot frame:
	if (pcMsg->header.frame_id != frame_id_robot_)
	{
		mrpt::poses::CPose3D sensorPoseInRobot;
		if (!wait_for_transform(sensorPoseInRobot, pcMsg->header.frame_id, frame_id_robot_))
		{
			return;
		}
		pc->changeCoordinatesReference(sensorPoseInRobot);
	}

	// Drop points outside the collision height band (removes ground and
	// overhead returns from a raw 3D lidar) and those on the robot itself.
	const double selfR2 = self_filter_radius_ * self_filter_radius_;
	const auto& xs = pc->getPointsBufferRef_x();
	const auto& ys = pc->getPointsBufferRef_y();
	const auto& zs = pc->getPointsBufferRef_z();
	auto filtered = mrpt::maps::CSimplePointsMap::Create();
	filtered->reserve(xs.size());
	for (std::size_t i = 0; i < xs.size(); i++)
	{
		if (zs[i] < obstacle_z_min_ || zs[i] > obstacle_z_max_)
		{
			continue;
		}
		if (self_filter_radius_ > 0 && xs[i] * xs[i] + ys[i] * ys[i] < selfR2)
		{
			continue;
		}
		filtered->insertPointFast(xs[i], ys[i], zs[i]);
	}
	filtered->mark_as_modified();

#if HAVE_MPP_COLLISION_GUARD
	if (guard_enabled_)
	{
		auto lck = std::lock_guard(guard_cs_);
		guard_.setObstacles(*filtered, mrpt::ros2bridge::fromROS(pcMsg->header.stamp));
	}
#endif

	// The follower predictive safety works in the map frame:
	mrpt::poses::CPose3D robotPoseInMap;
	if (!wait_for_transform(robotPoseInMap, frame_id_robot_, frame_id_map_))
	{
		return;
	}
	filtered->changeCoordinatesReference(robotPoseInMap);

	auto lck = std::lock_guard(follower_cs_);
	follower_.setObstacles(*filtered);
}

void TrajectoryFollowerNode::callback_robot_shape(const geometry_msgs::msg::PolygonStamped& msg)
{
	mrpt::math::TPolygon2D other;
	for (const auto& p : msg.polygon.points)
	{
		other.emplace_back(p.x, p.y);
	}

	auto lck = std::lock_guard(follower_cs_);
	const bool same = mpp::sameRobotShape(own_shape_, other);
	if (same)
	{
		RCLCPP_INFO(
			get_logger(), "Robot footprint is consistent with '%s'.",
			topic_robot_shape_sub_.c_str());
		shape_mismatch_ = false;
		return;
	}
	shape_mismatch_ = true;
	std::stringstream ss;
	ss << "Robot footprint MISMATCH: this node uses " << to_string(own_shape_) << " but '"
	   << topic_robot_shape_sub_ << "' publishes " << to_string(other)
	   << ". Use the same robot description (ptg_ini) in all nodes. Refusing to drive.";
	RCLCPP_ERROR_STREAM(get_logger(), ss.str());
	abort_navigation(mpp::StopKind::EMERGENCY);
	publish_status(STATUS_FAILED);
}

void TrajectoryFollowerNode::callback_odom(const nav_msgs::msg::Odometry::SharedPtr& msg)
{
	auto lck = std::lock_guard(odom_cs_);
	last_odom_ = *msg;
	have_odom_ = true;
}

// ------------------------------- control loop -------------------------------
void TrajectoryFollowerNode::control_tick()
{
	{
		auto lck = std::lock_guard(follower_cs_);
		if (!follower_.hasTrajectory())
		{
			// Nothing to do; robot commanded elsewhere / already idle. Keep
			// reporting the last status at a low rate.
			if ((this->now() - last_idle_status_pub_).seconds() > 1.0)
			{
				last_idle_status_pub_ = this->now();
				publish_status(status_);
			}
			return;
		}
	}

	// Read localization and odometry (possibly-blocking TF lookups) outside the
	// follower lock so they do not stall the path/obstacle callbacks.
	const auto loc = get_localization();
	if (!loc.valid)
	{
		// Emit a single stop only if we were driving; then stay silent.
		if (actively_driving_.exchange(false))
		{
			stop(mpp::StopKind::EMERGENCY);
		}
		return;
	}
	const auto odo = get_odometry();

	auto lck = std::lock_guard(follower_cs_);
	if (!follower_.hasTrajectory())
	{
		return;	 // canceled meanwhile
	}
	const mpp::TrajectoryFollower::Output out = follower_.step(loc, odo);

	switch (out.status)
	{
		case mpp::FollowerStatus::ReachedGoal:
			// Publish one clean zero on the transition to stopped, then stay
			// silent (don't re-publish every tick) so a downstream mux can time
			// this input out.
			if (actively_driving_.exchange(false))
			{
				stop(mpp::StopKind::REGULAR);
				RCLCPP_INFO(
					get_logger(), "Goal reached: final heading error %.1f deg.",
					mrpt::RAD2DEG(out.heading_err));
			}
			publish_status(to_string(out.status));
			break;

		case mpp::FollowerStatus::Blocked:
			RCLCPP_WARN_THROTTLE(
				get_logger(), *get_clock(), 2000,
				"Blocked by predicted contacts: forecast at %.2f m, reference path at %.2f m "
				"(s=%.2f/%.2f m)",
				out.contact_dist_forecast, out.contact_dist_reference, out.arc_length_s,
				follower_.totalLength());
			if (replan_on_failure_)
			{
				on_navigation_failure(to_string(out.status));
			}
			else
			{
				// Wait (stopped) for the way to clear:
				if (actively_driving_.exchange(false))
				{
					stop(mpp::StopKind::REGULAR);
				}
				publish_status(to_string(out.status));
			}
			break;

		case mpp::FollowerStatus::OffPathExceeded:
			// The robot is no longer on the reference path: never keep
			// driving on it.
			RCLCPP_WARN(
				get_logger(), "Off path: cross_track=%.2f m, s=%.2f/%.2f m", out.cross_track_err,
				out.arc_length_s, follower_.totalLength());
			on_navigation_failure(to_string(out.status));
			break;

		case mpp::FollowerStatus::MissedGoal:
			// Stopped next to the goal, but too far from its heading: a new
			// plan from here can include the needed maneuver.
			RCLCPP_WARN(
				get_logger(), "Missed the goal: final heading error %.1f deg.",
				mrpt::RAD2DEG(out.heading_err));
			on_navigation_failure(to_string(out.status));
			break;

		default:
		{
			actively_driving_ = true;
			follow(out.command);

			// Held stopped by the collision guard for too long:
			const bool guardBlocked =
				guard_stopped_since_ &&
				(this->now() - *guard_stopped_since_).seconds() > follower_.params.block_timeout;
			if (!guardBlocked)
			{
				publish_status(to_string(out.status));
				break;
			}
			const std::string blocked = to_string(mpp::FollowerStatus::Blocked);
			if (status_ != blocked)
			{
				RCLCPP_WARN(get_logger(), "Blocked by the collision guard.");
			}
			if (replan_on_failure_)
			{
				on_navigation_failure(blocked);
			}
			else
			{
				// Keep reporting it while the guard holds the robot:
				publish_status(blocked);
			}
			break;
		}
	}

	RCLCPP_DEBUG_THROTTLE(
		get_logger(), *get_clock(), 1000, "status=%s safety_scale=%.2f v=%.2f",
		to_string(out.status), out.safety_scale, out.target_speed);
}

// ------------------------------------
int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TrajectoryFollowerNode>());
	rclcpp::shutdown();
	return 0;
}

#else  // mpp::TrajectoryFollower not available in the linked mrpt_path_planning

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	RCLCPP_FATAL(
		rclcpp::get_logger("mrpt_trajectory_follower"),
		"This node requires a newer mrpt_path_planning providing "
		"mpp::TrajectoryFollower (mpp/algos/TrajectoryFollower.h). "
		"Update mrpt_path_planning and rebuild.");
	rclcpp::shutdown();
	return 1;
}

#endif	// __has_include(<mpp/algos/TrajectoryFollower.h>)
