/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2023, Individual contributors, see commit authors   |
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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mrpt_msgs/msg/waypoint.hpp>
#include <mrpt_msgs/msg/waypoint_sequence.hpp>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace mrpt::nav;
using mrpt::maps::CSimplePointsMap;
using namespace mrpt::system;
using namespace mrpt::config;

// The ROS node
class ReactiveNav2DNode : public rclcpp::Node
{
   public:
	/* Ctor*/
	explicit ReactiveNav2DNode(
		const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
	/* Dtor*/
	~ReactiveNav2DNode() {}

   private:
	// methods
	void read_parameters();
	void navigate_to(const mrpt::math::TPose2D& target);
	void on_do_navigation();
	void on_goal_received(
		const geometry_msgs::msg::PoseStamped::SharedPtr& trg_ptr);
	void on_local_obstacles(
		const sensor_msgs::msg::PointCloud2::SharedPtr& obs);
	void on_set_robot_shape(
		const geometry_msgs::msg::Polygon::SharedPtr& newShape);
	void on_odometry_received(const nav_msgs::msg::Odometry::SharedPtr& odom);
	void update_waypoint_sequence(
		const mrpt_msgs::msg::WaypointSequence::SharedPtr& wp);
	void on_waypoint_seq_received(
		const mrpt_msgs::msg::WaypointSequence::SharedPtr& wps);

   private:
	/** @name ROS pubs/subs
	 *  @{ */
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_sub_odometry;
	rclcpp::Subscription<mrpt_msgs::msg::WaypointSequence>::SharedPtr
		m_sub_wp_seq;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
		m_sub_nav_goal;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
		m_sub_local_obs;
	rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr
		m_sub_robot_shape;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_pub_cmd_vel;

	std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
	std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
	/** @} */

	CTimeLogger m_profiler;
	bool m_1st_time_init;  //!< Reactive initialization done?
	double m_target_allowed_distance;
	double m_nav_period;

	std::string m_sub_topic_reactive_nav_goal = "reactive_nav_goal";
	std::string m_sub_topic_local_obstacles = "local_map_pointcloud";
	std::string m_sub_topic_robot_shape{};
	std::string m_sub_topic_wp_seq = "reactive_nav_waypoints";
	std::string m_sub_topic_odometry = "odom";

	std::string m_pub_topic_cmd_vel = "cmd_vel";

	std::string m_frameid_reference = "map";
	std::string m_frameid_robot = "base_link";

	std::string m_plugin_file;
	std::string m_cfg_file_reactive;

	bool m_save_nav_log;

	rclcpp::TimerBase::SharedPtr m_timer_run_nav;

	mrpt::obs::CObservationOdometry m_odometry;
	CSimplePointsMap m_last_obstacles;
	std::mutex m_last_obstacles_cs;
	std::mutex m_odometry_cs;

	struct MyReactiveInterface : public mrpt::nav::CRobot2NavInterface
	{
		ReactiveNav2DNode& m_parent;

		MyReactiveInterface(ReactiveNav2DNode& parent) : m_parent(parent) {}

		/** Get the current pose and speeds of the robot.
		 *   \param curPose Current robot pose.
		 *   \param curV Current linear speed, in meters per second.
		 *	 \param curW Current angular speed, in radians per second.
		 * \return false on any error.
		 */
		bool getCurrentPoseAndSpeeds(
			mrpt::math::TPose2D& curPose, mrpt::math::TTwist2D& curVel,
			mrpt::system::TTimeStamp& timestamp,
			mrpt::math::TPose2D& curOdometry, std::string& frame_id) override
		{
			double curV, curW;

			CTimeLoggerEntry tle(
				m_parent.m_profiler, "getCurrentPoseAndSpeeds");

			// rclcpp::Duration timeout(0.1);
			rclcpp::Duration timeout(std::chrono::milliseconds(100));

			geometry_msgs::msg::TransformStamped tfGeom;
			try
			{
				CTimeLoggerEntry tle2(
					m_parent.m_profiler,
					"getCurrentPoseAndSpeeds.lookupTransform_sensor");

				tfGeom = m_parent.m_tf_buffer->lookupTransform(
					m_parent.m_frameid_reference, m_parent.m_frameid_robot,
					tf2::TimePointZero,
					tf2::durationFromSec(timeout.seconds()));
			}
			catch (const tf2::TransformException& ex)
			{
				RCLCPP_ERROR(m_parent.get_logger(), "%s", ex.what());
				return false;
			}

			tf2::Transform txRobotPose;
			tf2::fromMsg(tfGeom.transform, txRobotPose);

			const mrpt::poses::CPose3D curRobotPose =
				mrpt::ros2bridge::fromROS(txRobotPose);

			timestamp = mrpt::ros2bridge::fromROS(tfGeom.header.stamp);

			// Explicit 3d->2d to confirm we know we're losing information
			curPose = mrpt::poses::CPose2D(curRobotPose).asTPose();
			curOdometry = curPose;

			curV = curW = 0;
			MRPT_TODO("Retrieve current speeds from /odom topic?");
			RCLCPP_DEBUG(
				m_parent.get_logger(),
				"[getCurrentPoseAndSpeeds] Latest pose: %s",
				curPose.asString().c_str());

			// From local to global:
			curVel = mrpt::math::TTwist2D(curV, .0, curW).rotated(curPose.phi);

			return true;
		}

		/** Change the instantaneous speeds of robot.
		 *   \param v Linear speed, in meters per second.
		 *	 \param w Angular speed, in radians per second.
		 * \return false on any error.
		 */
		bool changeSpeeds(
			const mrpt::kinematics::CVehicleVelCmd& vel_cmd) override
		{
			using namespace mrpt::kinematics;
			const CVehicleVelCmd_DiffDriven* vel_cmd_diff_driven =
				dynamic_cast<const CVehicleVelCmd_DiffDriven*>(&vel_cmd);
			ASSERT_(vel_cmd_diff_driven);

			const double v = vel_cmd_diff_driven->lin_vel;
			const double w = vel_cmd_diff_driven->ang_vel;
			RCLCPP_DEBUG(
				m_parent.get_logger(),
				"changeSpeeds: v=%7.4f m/s  w=%8.3f deg/s", v,
				w * 180.0f / M_PI);
			geometry_msgs::msg::Twist cmd;
			cmd.linear.x = v;
			cmd.angular.z = w;
			m_parent.m_pub_cmd_vel->publish(cmd);
			return true;
		}

		bool stop(bool isEmergency) override
		{
			mrpt::kinematics::CVehicleVelCmd_DiffDriven vel_cmd;
			vel_cmd.lin_vel = 0;
			vel_cmd.ang_vel = 0;
			return changeSpeeds(vel_cmd);
		}

		/** Start the watchdog timer of the robot platform, if any.
		 * \param T_ms Period, in ms.
		 * \return false on any error. */
		virtual bool startWatchdog(float T_ms) override { return true; }
		/** Stop the watchdog timer.
		 * \return false on any error. */
		virtual bool stopWatchdog() override { return true; }
		/** Return the current set of obstacle points.
		 * \return false on any error. */
		bool senseObstacles(
			CSimplePointsMap& obstacles,
			mrpt::system::TTimeStamp& timestamp) override
		{
			timestamp = mrpt::system::now();
			std::lock_guard<std::mutex> csl(m_parent.m_last_obstacles_cs);
			obstacles = m_parent.m_last_obstacles;

			MRPT_TODO("TODO: Check age of obstacles!");
			return true;
		}

		mrpt::kinematics::CVehicleVelCmd::Ptr getEmergencyStopCmd() override
		{
			return getStopCmd();
		}
		mrpt::kinematics::CVehicleVelCmd::Ptr getStopCmd() override
		{
			mrpt::kinematics::CVehicleVelCmd::Ptr ret =
				mrpt::kinematics::CVehicleVelCmd::Ptr(
					new mrpt::kinematics::CVehicleVelCmd_DiffDriven);
			ret->setToStop();
			return ret;
		}

		void sendNavigationStartEvent() override {}
		void sendNavigationEndEvent() override {}
		void sendNavigationEndDueToErrorEvent() override {}
		void sendWaySeemsBlockedEvent() override {}
	};

	MyReactiveInterface m_reactive_if;

	CReactiveNavigationSystem m_reactive_nav_engine;
	std::mutex m_reactive_nav_engine_cs;
};