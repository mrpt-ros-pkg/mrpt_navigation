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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry_;
	rclcpp::Subscription<mrpt_msgs::msg::WaypointSequence>::SharedPtr subWpSeq_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
		subNavGoal_;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLocalObs_;
	rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr subRobotShape_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubCmdVel_;

	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
		pubSelectedPtg_;

	std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
	std::shared_ptr<tf2_ros::TransformListener> tfListener_;
	/** @} */

	CTimeLogger profiler_;
	bool initialized_ = false;	//!< Reactive initialization done?
	double targetAllowedDistance_ = 0.40;
	double navPeriod_ = 0.10;

	std::string subTopicNavGoal_ = "reactive_nav_goal";
	std::string subTopicLocalObstacles_ = "local_map_pointcloud";
	std::string subTopicRobotShape_;
	std::string subTopicWpSeq_ = "reactive_nav_waypoints";
	std::string subTopicOdometry_ = "/odom";

	std::string pubTopicCmdVel_ = "/cmd_vel";
	std::string pubTopicSelectedPtg_ = "reactivenav_selected_ptg";

	std::string frameidReference_ = "map";
	std::string frameidRobot_ = "base_link";

	std::string pluginFile_;
	std::string cfgFileReactive_;

	bool saveNavLog_ = false;

	rclcpp::TimerBase::SharedPtr timerRunNav_;

	mrpt::obs::CObservationOdometry odometry_;
	std::mutex odometryMtx_;

	CSimplePointsMap lastObstacles_;
	std::mutex lastObstaclesMtx_;

	bool waitForTransform(
		mrpt::poses::CPose3D& des, const std::string& target_frame,
		const std::string& source_frame, const int timeoutMilliseconds = 50);

	void publish_last_log_record_to_ros(const mrpt::nav::CLogFileRecord& lr);

	visualization_msgs::msg::MarkerArray log_to_margers(
		const mrpt::nav::CLogFileRecord& lr);

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
			mrpt::system::TTimeStamp& timestamp,
			mrpt::math::TPose2D& curOdometry, std::string& frame_id) override
		{
			double curV, curW;

			CTimeLoggerEntry tle(parent_.profiler_, "getCurrentPoseAndSpeeds");

			// rclcpp::Duration timeout(0.1);
			rclcpp::Duration timeout(std::chrono::milliseconds(100));

			geometry_msgs::msg::TransformStamped tfGeom;
			try
			{
				CTimeLoggerEntry tle2(
					parent_.profiler_,
					"getCurrentPoseAndSpeeds.lookupTransform_sensor");

				tfGeom = parent_.tfBuffer_->lookupTransform(
					parent_.frameidReference_, parent_.frameidRobot_,
					tf2::TimePointZero,
					tf2::durationFromSec(timeout.seconds()));
			}
			catch (const tf2::TransformException& ex)
			{
				RCLCPP_ERROR(parent_.get_logger(), "%s", ex.what());
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
				parent_.get_logger(),
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
				parent_.get_logger(),
				"changeSpeeds: v=%7.4f m/s  w=%8.3f deg/s", v,
				w * 180.0f / M_PI);
			geometry_msgs::msg::Twist cmd;
			cmd.linear.x = v;
			cmd.angular.z = w;
			parent_.pubCmdVel_->publish(cmd);
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
			std::lock_guard<std::mutex> csl(parent_.lastObstaclesMtx_);
			obstacles = parent_.lastObstacles_;

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

	MyReactiveInterface reactiveInterface_{*this};
	CReactiveNavigationSystem rnavEngine_{reactiveInterface_};
	std::mutex rnavEngineMtx_;
};
