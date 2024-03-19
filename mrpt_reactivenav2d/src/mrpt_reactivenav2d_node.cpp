/***********************************************************************************
 * Revised BSD License *
 * Copyright (c) 2014-2023, Jose-Luis Blanco <jlblanco@ual.es> *
 * All rights reserved. *
 *                                                                                 *
 * Redistribution and use in source and binary forms, with or without *
 * modification, are permitted provided that the following conditions are met: *
 *     * Redistributions of source code must retain the above copyright *
 *       notice, this list of conditions and the following disclaimer. *
 *     * Redistributions in binary form must reproduce the above copyright *
 *       notice, this list of conditions and the following disclaimer in the *
 *       documentation and/or other materials provided with the distribution. *
 *     * Neither the name of the Vienna University of Technology nor the *
 *       names of its contributors may be used to endorse or promote products *
 *       derived from this software without specific prior written permission. *
 *                                                                                 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND *
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 **
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE *
 * DISCLAIMED. IN NO EVENT SHALL Markus Bader BE LIABLE FOR ANY *
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES *
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 **
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND *
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 **
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. *
 ***********************************************************************************/

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseStamped.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/kinematics/CVehicleVelCmd_DiffDriven.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/nav/reactive/CReactiveNavigationSystem.h>
#include <mrpt/nav/reactive/TWaypoint.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/ros1bridge/point_cloud2.h>
#include <mrpt/ros1bridge/pose.h>
#include <mrpt/ros1bridge/time.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt_msgs/Waypoint.h>
#include <mrpt_msgs/WaypointSequence.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <mutex>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace mrpt::nav;
using mrpt::maps::CSimplePointsMap;
using namespace mrpt::system;
using namespace mrpt::config;

// The ROS node
class ReactiveNav2DNode
{
   private:
	struct TAuxInitializer
	{
		TAuxInitializer(int argc, char** argv)
		{
			ros::init(argc, argv, "mrpt_reactivenav2d");
		}
	};

	CTimeLogger m_profiler;
	TAuxInitializer m_auxinit;	//!< Just to make sure ROS is init first
	ros::NodeHandle m_nh{};	 //!< The node handle
	ros::NodeHandle m_localn{"~"};	//!< "~"

	/** @name ROS pubs/subs
	 *  @{ */
	ros::Subscriber m_sub_odometry;
	ros::Subscriber m_sub_wp_seq;
	ros::Subscriber m_sub_nav_goal;
	ros::Subscriber m_sub_local_obs;
	ros::Subscriber m_sub_robot_shape;
	ros::Publisher m_pub_cmd_vel;

	tf2_ros::Buffer m_tf_buffer;
	tf2_ros::TransformListener m_tf_listener{m_tf_buffer};
	/** @} */

	bool m_1st_time_init = false;  //!< Reactive initialization done?
	double m_target_allowed_distance = 0.40f;  //!< for single-target commands
	double m_nav_period = 0.1;	//!< [s]

	std::string m_sub_topic_reactive_nav_goal = "reactive_nav_goal";
	std::string m_sub_topic_local_obstacles = "local_map_pointcloud";
	std::string m_sub_topic_robot_shape{};

	std::string m_pub_topic_cmd_vel = "cmd_vel";
	std::string m_sub_topic_wp_seq = "reactive_nav_waypoints";
	std::string m_sub_topic_odometry = "odom";

	std::string m_frameid_reference = "map";
	std::string m_frameid_robot = "base_link";

	std::string m_plugin_file;

	bool m_save_nav_log = false;

	ros::Timer m_timer_run_nav;

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

			ros::Duration timeout(0.1);

			geometry_msgs::TransformStamped tfGeom;
			try
			{
				CTimeLoggerEntry tle2(
					m_parent.m_profiler,
					"getCurrentPoseAndSpeeds.lookupTransform_sensor");

				tfGeom = m_parent.m_tf_buffer.lookupTransform(
					m_parent.m_frameid_reference, m_parent.m_frameid_robot,
					ros::Time(0), timeout);
			}
			catch (const tf2::TransformException& ex)
			{
				ROS_ERROR("%s", ex.what());
				return false;
			}

			tf2::Transform txRobotPose;
			tf2::fromMsg(tfGeom.transform, txRobotPose);

			const mrpt::poses::CPose3D curRobotPose =
				mrpt::ros1bridge::fromROS(txRobotPose);

			timestamp = mrpt::ros1bridge::fromROS(tfGeom.header.stamp);

			// Explicit 3d->2d to confirm we know we're losing information
			curPose = mrpt::poses::CPose2D(curRobotPose).asTPose();
			curOdometry = curPose;

			curV = curW = 0;
			MRPT_TODO("Retrieve current speeds from /odom topic?");
			ROS_DEBUG(
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
			ROS_DEBUG(
				"changeSpeeds: v=%7.4f m/s  w=%8.3f deg/s", v,
				w * 180.0f / M_PI);
			geometry_msgs::Twist cmd;
			cmd.linear.x = v;
			cmd.angular.z = w;
			m_parent.m_pub_cmd_vel.publish(cmd);
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
			timestamp = mrpt::Clock::now();
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

   public:
	/**  Constructor: Inits ROS system */
	ReactiveNav2DNode(int argc, char** argv)
		: m_auxinit(argc, argv),
		  m_nh(),
		  m_localn("~"),
		  m_reactive_if(*this),
		  m_reactive_nav_engine(m_reactive_if)
	{
		// Load params:
		std::string cfg_file_reactive;
		m_localn.param(
			"cfg_file_reactive", cfg_file_reactive, cfg_file_reactive);
		m_localn.param(
			"target_allowed_distance", m_target_allowed_distance,
			m_target_allowed_distance);
		m_localn.param("nav_period", m_nav_period, m_nav_period);
		m_localn.param(
			"frameid_reference", m_frameid_reference, m_frameid_reference);
		m_localn.param("frameid_robot", m_frameid_robot, m_frameid_robot);
		m_localn.param(
			"topic_robot_shape", m_sub_topic_robot_shape,
			m_sub_topic_robot_shape);

		m_localn.param("topic_wp_seq", m_sub_topic_wp_seq, m_sub_topic_wp_seq);
		m_localn.param(
			"topic_odometry", m_sub_topic_odometry, m_sub_topic_odometry);
		m_localn.param(
			"topic_cmd_vel", m_pub_topic_cmd_vel, m_pub_topic_cmd_vel);
		m_localn.param(
			"topic_obstacles", m_sub_topic_local_obstacles,
			m_sub_topic_local_obstacles);

		m_localn.param("save_nav_log", m_save_nav_log, m_save_nav_log);

		m_localn.param("ptg_plugin_files", m_plugin_file, m_plugin_file);

		if (!m_plugin_file.empty())
		{
			ROS_INFO_STREAM("About to load plugins: " << m_plugin_file);
			std::string errorMsgs;
			if (!mrpt::system::loadPluginModules(m_plugin_file, errorMsgs))
			{
				ROS_ERROR_STREAM("Error loading rnav plugins: " << errorMsgs);
			}
			ROS_INFO_STREAM("Pluginns loaded OK.");
		}

		ROS_ASSERT(m_nav_period > 0);
		ROS_ASSERT_MSG(
			!cfg_file_reactive.empty(),
			"Mandatory param 'cfg_file_reactive' is missing!");
		ROS_ASSERT_MSG(
			mrpt::system::fileExists(cfg_file_reactive),
			"Config file not found: '%s'", cfg_file_reactive.c_str());

		m_reactive_nav_engine.enableLogFile(m_save_nav_log);

		// Load reactive config:
		// ----------------------------------------------------
		try
		{
			CConfigFile cfgFil(cfg_file_reactive);
			m_reactive_nav_engine.loadConfigFile(cfgFil);
		}
		catch (std::exception& e)
		{
			ROS_ERROR(
				"Exception initializing reactive navigation engine:\n%s",
				e.what());
			throw;
		}

		// load robot shape: (1) default, (2) via params, (3) via topic
		// ----------------------------------------------------------------
		// m_reactive_nav_engine.changeRobotShape();

		// Init this subscriber first so we know asap the desired robot shape,
		// if provided via a topic:
		if (!m_sub_topic_robot_shape.empty())
		{
			ROS_INFO(
				"Subscribing to robot shape via topic '%s'...",
				m_sub_topic_robot_shape.c_str());
			m_sub_robot_shape = m_nh.subscribe<geometry_msgs::Polygon>(
				m_sub_topic_robot_shape, 1,
				&ReactiveNav2DNode::onRosSetRobotShape, this);
		}
		else
		{
			// Load robot shape: 1/2 polygon
			// ---------------------------------------------
			CConfigFile c(cfg_file_reactive);
			std::string s = "CReactiveNavigationSystem";

			std::vector<float> xs, ys;
			c.read_vector(
				s, "RobotModel_shape2D_xs", std::vector<float>(), xs, false);
			c.read_vector(
				s, "RobotModel_shape2D_ys", std::vector<float>(), ys, false);
			ASSERTMSG_(
				xs.size() == ys.size(),
				"Config parameters `RobotModel_shape2D_xs` and "
				"`RobotModel_shape2D_ys` "
				"must have the same length!");
			if (!xs.empty())
			{
				mrpt::math::CPolygon poly;
				poly.resize(xs.size());
				for (size_t i = 0; i < xs.size(); i++)
				{
					poly[i].x = xs[i];
					poly[i].y = ys[i];
				}

				std::lock_guard<std::mutex> csl(m_reactive_nav_engine_cs);
				m_reactive_nav_engine.changeRobotShape(poly);
			}

			// Load robot shape: 2/2 circle
			// ---------------------------------------------
			if (const double robot_radius = c.read_double(
					s, "RobotModel_circular_shape_radius", -1.0, false);
				robot_radius > 0)
			{
				std::lock_guard<std::mutex> csl(m_reactive_nav_engine_cs);
				m_reactive_nav_engine.changeRobotCircularShapeRadius(
					robot_radius);
			}
		}

		// Init ROS publishers:
		// -----------------------
		m_pub_cmd_vel =
			m_nh.advertise<geometry_msgs::Twist>(m_pub_topic_cmd_vel, 1);

		// Init ROS subs:
		// -----------------------
		// odometry
		m_sub_odometry = m_nh.subscribe(
			m_sub_topic_odometry, 1, &ReactiveNav2DNode::onOdometryReceived,
			this);
		// waypoints
		m_sub_wp_seq = m_nh.subscribe(
			m_sub_topic_wp_seq, 1, &ReactiveNav2DNode::onWaypointSeqReceived,
			this);
		// "/reactive_nav_goal", "/move_base_simple/goal" (
		// geometry_msgs/PoseStamped )
		m_sub_nav_goal = m_nh.subscribe<geometry_msgs::PoseStamped>(
			m_sub_topic_reactive_nav_goal, 1,
			&ReactiveNav2DNode::onRosGoalReceived, this);
		m_sub_local_obs = m_nh.subscribe<sensor_msgs::PointCloud2>(
			m_sub_topic_local_obstacles, 1,
			&ReactiveNav2DNode::onRosLocalObstacles, this);

		// Init timers:
		// ----------------------------------------------------
		m_timer_run_nav = m_nh.createTimer(
			ros::Duration(m_nav_period), &ReactiveNav2DNode::onDoNavigation,
			this);

	}  // end ctor

	/**
	 * @brief Issue a navigation command
	 * @param target The target location
	 */
	void navigateTo(const mrpt::math::TPose2D& target)
	{
		ROS_INFO(
			"[navigateTo] Starting navigation to %s",
			target.asString().c_str());

		CAbstractPTGBasedReactive::TNavigationParamsPTG navParams;

		CAbstractNavigator::TargetInfo target_info;
		target_info.target_coords.x = target.x;
		target_info.target_coords.y = target.y;
		target_info.targetAllowedDistance = m_target_allowed_distance;
		target_info.targetIsRelative = false;

		// API for multiple waypoints:
		//...
		// navParams.multiple_targets.push_back(target_info);

		// API for single targets:
		navParams.target = target_info;

		// Optional: restrict the PTGs to use
		// navParams.restrict_PTG_indices.push_back(1);

		{
			std::lock_guard<std::mutex> csl(m_reactive_nav_engine_cs);
			m_reactive_nav_engine.navigate(&navParams);
		}
	}

	/** Callback: On run navigation */
	void onDoNavigation(const ros::TimerEvent&)
	{
		// 1st time init:
		// ----------------------------------------------------
		if (!m_1st_time_init)
		{
			m_1st_time_init = true;
			ROS_INFO(
				"[ReactiveNav2DNode] Initializing reactive navigation "
				"engine...");
			{
				std::lock_guard<std::mutex> csl(m_reactive_nav_engine_cs);
				m_reactive_nav_engine.initialize();
			}
			ROS_INFO(
				"[ReactiveNav2DNode] Reactive navigation engine init done!");
		}

		CTimeLoggerEntry tle(m_profiler, "onDoNavigation");
		// Main nav loop (in whatever state nav is: IDLE, NAVIGATING, etc.)
		m_reactive_nav_engine.navigationStep();
	}

	void onOdometryReceived(const nav_msgs::Odometry& msg)
	{
		std::lock_guard<std::mutex> csl(m_odometry_cs);
		tf2::Quaternion quat(
			msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
			msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
		tf2::Matrix3x3 mat(quat);
		double roll, pitch, yaw;
		mat.getRPY(roll, pitch, yaw);
		m_odometry.odometry = mrpt::poses::CPose2D(
			msg.pose.pose.position.x, msg.pose.pose.position.y, yaw);

		m_odometry.velocityLocal.vx = msg.twist.twist.linear.x;
		m_odometry.velocityLocal.vy = msg.twist.twist.linear.y;
		m_odometry.velocityLocal.omega = msg.twist.twist.angular.z;
		m_odometry.hasVelocities = true;

		ROS_DEBUG_STREAM("Odometry updated");
	}

	void updateWaypointSequence(const mrpt_msgs::WaypointSequence& msg)
	{
		mrpt::nav::TWaypointSequence wps;

		for (const auto& wp : msg.waypoints)
		{
			tf2::Quaternion quat(
				wp.target.orientation.x, wp.target.orientation.y,
				wp.target.orientation.z, wp.target.orientation.w);
			tf2::Matrix3x3 mat(quat);
			double roll, pitch, yaw;
			mat.getRPY(roll, pitch, yaw);
			auto waypoint = mrpt::nav::TWaypoint(
				wp.target.position.x, wp.target.position.y, wp.allowed_distance,
				wp.allow_skip);

			if (yaw == yaw && !wp.ignore_heading)  // regular number, not NAN
				waypoint.target_heading = yaw;

			wps.waypoints.push_back(waypoint);
		}

		ROS_INFO_STREAM("New navigateWaypoints() command");
		{
			std::lock_guard<std::mutex> csl(m_reactive_nav_engine_cs);
			m_reactive_nav_engine.navigateWaypoints(wps);
		}
	}
	void onWaypointSeqReceived(const mrpt_msgs::WaypointSequence& wps)
	{
		updateWaypointSequence(wps);
	}

	void onRosGoalReceived(const geometry_msgs::PoseStampedConstPtr& trg_ptr)
	{
		geometry_msgs::PoseStamped trg = *trg_ptr;

		ROS_INFO(
			"Nav target received via topic sub: (%.03f,%.03f, %.03fdeg) "
			"[frame_id=%s]",
			trg.pose.position.x, trg.pose.position.y,
			trg.pose.orientation.z * 180.0 / M_PI, trg.header.frame_id.c_str());

		// Convert to the "m_frameid_reference" frame of coordinates:
		if (trg.header.frame_id != m_frameid_reference)
		{
			ros::Duration timeout(0.2);
			try
			{
				geometry_msgs::TransformStamped ref_to_trgFrame =
					m_tf_buffer.lookupTransform(
						trg.header.frame_id, m_frameid_reference, ros::Time(0),
						timeout);

				tf2::doTransform(trg, trg, ref_to_trgFrame);
			}
			catch (const tf2::TransformException& ex)
			{
				ROS_ERROR("%s", ex.what());
				return;
			}
		}

		this->navigateTo(mrpt::math::TPose2D(
			trg.pose.position.x, trg.pose.position.y, trg.pose.orientation.z));
	}

	void onRosLocalObstacles(const sensor_msgs::PointCloud2::ConstPtr& obs)
	{
		std::lock_guard<std::mutex> csl(m_last_obstacles_cs);
		mrpt::ros1bridge::fromROS(*obs, m_last_obstacles);
		// ROS_DEBUG("Local obstacles received: %u points", static_cast<unsigned
		// int>(m_last_obstacles.size()) );
	}

	void onRosSetRobotShape(const geometry_msgs::Polygon::ConstPtr& newShape)
	{
		ROS_INFO_STREAM(
			"[onRosSetRobotShape] Robot shape received via topic: "
			<< *newShape);

		mrpt::math::CPolygon poly;
		poly.resize(newShape->points.size());
		for (size_t i = 0; i < newShape->points.size(); i++)
		{
			poly[i].x = newShape->points[i].x;
			poly[i].y = newShape->points[i].y;
		}

		{
			std::lock_guard<std::mutex> csl(m_reactive_nav_engine_cs);
			m_reactive_nav_engine.changeRobotShape(poly);
		}
	}

};	// end class

int main(int argc, char** argv)
{
	ReactiveNav2DNode the_node(argc, argv);
	ros::spin();
	return 0;
}
