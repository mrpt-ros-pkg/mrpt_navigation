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

#include <atomic>
#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
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
	}
	return "?";
}
}  // namespace

/** ROS 2 node wrapping mpp::TrajectoryFollower.
 *
 * Runs the follower's outer control loop at its configured control_period:
 * pulls the map-frame localization (from TF) and odometry (from a /odom topic),
 * calls step(), and drives the platform via the TrajectoryVehicleInterface it
 * implements (feedforward cmd_vel of the immediate chunk sample, with a
 * watchdog that zeroes cmd_vel if the loop stalls). Obstacles and the reference
 * path arrive on topics.
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

	// tf2:
	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	// Subs / pubs:
	rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_obstacles_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_chunk_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;

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
	double robot_radius_ = 0.0;	 //!< footprint fallback when no ptg_ini given

	// Obstacle cloud height band (in the map frame). Points outside are dropped
	// before the 2D predictive-safety check, so a raw 3D lidar can be used as an
	// obstacle source without its ground/overhead returns causing false stops.
	// Defaults are permissive (effectively disabled); set them for a 3D lidar.
	double obstacle_z_min_ = -1e6;
	double obstacle_z_max_ = 1e6;

	// Drop obstacle returns within this radius [m] of the robot base (removes
	// the robot's own body seen by a 3D lidar). <=0 disables the self-filter.
	double self_filter_radius_ = 0.0;

	// Raises follower_'s COutputLogger verbosity to LVL_DEBUG so its internal
	// per-cycle trace (lookahead, curvature, speed-cap breakdown, safety
	// scale, ...) is printed to stdout. Off by default (noisy).
	bool follower_debug_trace_ = false;

	mpp::TrajectoriesAndRobotShape ptgs_;

	mpp::Trajectory last_trajectory_;  //!< to ignore identical re-published paths
	bool have_trajectory_ = false;

	void read_parameters();
	void control_tick();
	void callback_path(const nav_msgs::msg::Path& msg);
	void callback_obstacles(const sensor_msgs::msg::PointCloud2::SharedPtr& pc);
	void callback_odom(const nav_msgs::msg::Odometry::SharedPtr& msg);

	[[nodiscard]] bool wait_for_transform(
		mrpt::poses::CPose3D& des, const std::string& target_frame, const std::string& source_frame,
		int timeout_milliseconds = 50);

	void publish_cmd(double vx, double omega);
	void publish_chunk(const mpp::SampledTrajectory& ref);
};

TrajectoryFollowerNode::TrajectoryFollowerNode()
	: rclcpp::Node(NODE_NAME), last_cmd_time_(this->now())
{
	read_parameters();

	follower_.setMinLoggingLevel(
		follower_debug_trace_ ? mrpt::system::LVL_DEBUG : mrpt::system::LVL_INFO);

	tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	// Load follower params (optional):
	if (!follower_params_file_.empty())
	{
		ASSERT_FILE_EXISTS_(follower_params_file_);
		follower_.params = mpp::TrajectoryFollower::Parameters::FromYAML(
			mrpt::containers::yaml::FromFile(follower_params_file_));
		RCLCPP_INFO_STREAM(get_logger(), "Loaded follower params:\n" << follower_.params.as_yaml());
	}

	// Load robot footprint from the PTG ini (optional but recommended so the
	// predictive safety sweep uses the real robot shape):
	if (!ptg_ini_file_.empty())
	{
		ASSERT_FILE_EXISTS_(ptg_ini_file_);
		mrpt::config::CConfigFile cfg(ptg_ini_file_);
		ptgs_.initFromConfigFile(cfg, "SelfDriving");
		follower_.setRobotShape(ptgs_.robotShape);
		RCLCPP_INFO(get_logger(), "Loaded robot shape from '%s'.", ptg_ini_file_.c_str());
	}
	else if (robot_radius_ > 0)
	{
		follower_.setRobotShape(mpp::robot_radius_t{robot_radius_});
		RCLCPP_INFO(get_logger(), "Using circular footprint, radius %.3f m.", robot_radius_);
	}
	else
	{
		RCLCPP_WARN(
			get_logger(),
			"No 'ptg_ini' or 'robot_radius' given: predictive safety will "
			"sample the reference point only (no footprint).");
	}

	const auto qos = rclcpp::SystemDefaultsQoS();

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

	pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_cmd_vel_pub_, qos);
	pub_chunk_ = this->create_publisher<nav_msgs::msg::Path>("~/predicted_trajectory", qos);
	pub_status_ = this->create_publisher<std_msgs::msg::String>("~/status", qos);

	// Control loop at the follower's control period:
	const auto period = std::chrono::duration<double>(follower_.params.control_period);
	control_timer_ = this->create_wall_timer(
		std::chrono::duration_cast<std::chrono::nanoseconds>(period),
		[this]() { this->control_tick(); });

	// Watchdog: a few control periods without a fresh command -> zero cmd_vel.
	start_watchdog(std::chrono::milliseconds(
		std::max<int>(200, static_cast<int>(5000 * follower_.params.control_period))));

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

	this->declare_parameter<double>("robot_radius", robot_radius_);
	this->get_parameter("robot_radius", robot_radius_);
	RCLCPP_INFO(get_logger(), "robot_radius: %.3f", robot_radius_);

	this->declare_parameter<double>("obstacle_z_min", obstacle_z_min_);
	this->get_parameter("obstacle_z_min", obstacle_z_min_);
	this->declare_parameter<double>("obstacle_z_max", obstacle_z_max_);
	this->get_parameter("obstacle_z_max", obstacle_z_max_);
	RCLCPP_INFO(
		get_logger(), "obstacle_z band (map frame): [%.2f, %.2f] m", obstacle_z_min_,
		obstacle_z_max_);

	this->declare_parameter<double>("self_filter_radius", self_filter_radius_);
	this->get_parameter("self_filter_radius", self_filter_radius_);
	RCLCPP_INFO(get_logger(), "self_filter_radius: %.3f m", self_filter_radius_);

	this->declare_parameter<bool>("follower_debug_trace", follower_debug_trace_);
	this->get_parameter("follower_debug_trace", follower_debug_trace_);
	RCLCPP_INFO(get_logger(), "follower_debug_trace: %s", follower_debug_trace_ ? "true" : "false");
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
	if (ref.empty())
	{
		publish_cmd(0, 0);
	}
	else
	{
		const auto& tw = ref.points.front().twist;
		publish_cmd(tw.vx, tw.omega);
	}
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
	if (tr.size() < 2)
	{
		RCLCPP_WARN(get_logger(), "Ignoring reference path with < 2 poses.");
		return;
	}

	auto lck = std::lock_guard(follower_cs_);

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

	follower_.setTrajectory(tr);
	last_trajectory_ = tr;
	have_trajectory_ = true;
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

	// Transform to the map frame (do the possibly-blocking TF lookup before
	// taking the follower lock).
	mrpt::poses::CPose3D sensorPoseInMap;
	if (!wait_for_transform(sensorPoseInMap, pcMsg->header.frame_id, frame_id_map_))
	{
		return;
	}
	pc->changeCoordinatesReference(sensorPoseInMap);

	// Robot base in map, for the self-filter (points on the robot's own body).
	double robotX = 0;
	double robotY = 0;
	bool haveRobot = false;
	if (self_filter_radius_ > 0)
	{
		mrpt::poses::CPose3D robotInMap;
		if (wait_for_transform(robotInMap, frame_id_robot_, frame_id_map_))
		{
			robotX = robotInMap.x();
			robotY = robotInMap.y();
			haveRobot = true;
		}
	}
	const double selfR2 = self_filter_radius_ * self_filter_radius_;

	// Drop points outside the collision height band (removes ground and
	// overhead returns from a raw 3D lidar) and those on the robot itself.
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
		if (haveRobot)
		{
			const double dx = xs[i] - robotX;
			const double dy = ys[i] - robotY;
			if (dx * dx + dy * dy < selfR2)
			{
				continue;
			}
		}
		filtered->insertPointFast(xs[i], ys[i], zs[i]);
	}
	filtered->mark_as_modified();

	auto lck = std::lock_guard(follower_cs_);
	follower_.setObstacles(*filtered);
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
			return;	 // nothing to do; robot commanded elsewhere / already idle
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

	mpp::TrajectoryFollower::Output out;
	{
		auto lck = std::lock_guard(follower_cs_);
		out = follower_.step(loc, odo);
	}

	switch (out.status)
	{
		case mpp::FollowerStatus::ReachedGoal:
		case mpp::FollowerStatus::Blocked:
			// Publish one clean zero on the transition to stopped, then stay
			// silent (don't re-publish every tick) so a downstream mux can time
			// this input out. Status keeps being published below regardless.
			if (actively_driving_.exchange(false))
			{
				stop(mpp::StopKind::REGULAR);
			}
			break;
		default:
			actively_driving_ = true;
			follow(out.command);
			break;
	}

	RCLCPP_DEBUG_THROTTLE(
		get_logger(), *get_clock(), 1000, "status=%s safety_scale=%.2f v=%.2f",
		to_string(out.status), out.safety_scale, out.target_speed);

	std_msgs::msg::String sm;
	sm.data = to_string(out.status);
	pub_status_->publish(sm);
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
