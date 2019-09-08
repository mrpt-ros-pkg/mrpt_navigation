/***********************************************************************************
 * Revised BSD License *
 * Copyright (c) 2014-2015, Jose-Luis Blanco <jlblanco@ual.es> *
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

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Polygon.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

#include <mrpt/version.h>

// Use modern headers ------------
#include <mrpt/nav/reactive/CReactiveNavigationSystem.h>
#include <mrpt/maps/CSimplePointsMap.h>
using namespace mrpt::nav;
using mrpt::maps::CSimplePointsMap;

#include <mrpt/version.h>
#if MRPT_VERSION >= 0x199
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/config/CConfigFile.h>
using namespace mrpt::system;
using namespace mrpt::config;
#else
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/utils/CConfigFileMemory.h>
#include <mrpt/utils/CConfigFile.h>
using namespace mrpt::utils;
#endif

#include <mrpt/system/filesystem.h>

#include <mrpt_bridge/pose.h>
#include <mrpt_bridge/point_cloud.h>
#include <mrpt_bridge/time.h>

#include <mrpt/kinematics/CVehicleVelCmd_DiffDriven.h>

#include <mutex>

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
	TAuxInitializer m_auxinit;  //!< Just to make sure ROS is init first
	ros::NodeHandle m_nh;  //!< The node handle
	ros::NodeHandle m_localn;  //!< "~"

	/** @name ROS pubs/subs
	 *  @{ */
	ros::Subscriber m_sub_nav_goal;
	ros::Subscriber m_sub_local_obs;
	ros::Subscriber m_sub_robot_shape;
	ros::Publisher m_pub_cmd_vel;
	tf::TransformListener m_tf_listener;  //!< Use to retrieve TF data
	/** @} */

	bool m_1st_time_init;  //!< Reactive initialization done?
	double m_target_allowed_distance;
	double m_nav_period;

	std::string m_pub_topic_reactive_nav_goal;
	std::string m_sub_topic_local_obstacles;
	std::string m_sub_topic_robot_shape;

	std::string m_frameid_reference;
	std::string m_frameid_robot;

	bool m_save_nav_log;

	ros::Timer m_timer_run_nav;

	CSimplePointsMap m_last_obstacles;
	std::mutex m_last_obstacles_cs;

	struct MyReactiveInterface :

		public mrpt::nav::CRobot2NavInterface
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
			tf::StampedTransform txRobotPose;
			try
			{
				CTimeLoggerEntry tle2(
					m_parent.m_profiler,
					"getCurrentPoseAndSpeeds.lookupTransform_sensor");
				m_parent.m_tf_listener.lookupTransform(
					m_parent.m_frameid_reference, m_parent.m_frameid_robot,
					ros::Time(0), txRobotPose);
			}
			catch (tf::TransformException& ex)
			{
				ROS_ERROR("%s", ex.what());
				return false;
			}

			mrpt::poses::CPose3D curRobotPose;
			mrpt_bridge::convert(txRobotPose, curRobotPose);

			mrpt_bridge::convert(txRobotPose.stamp_, timestamp);
			// Explicit 3d->2d to confirm we know we're losing information
			curPose =
#if MRPT_VERSION >= 0x199
				mrpt::poses::CPose2D(curRobotPose).asTPose();
#else
				mrpt::math::TPose2D(mrpt::poses::CPose2D(curRobotPose));
#endif
			curOdometry = curPose;

			curV = curW = 0;
			MRPT_TODO("Retrieve current speeds from odometry");
			ROS_DEBUG(
				"[getCurrentPoseAndSpeeds] Latest pose: %s",
				curPose.asString().c_str());

			curVel.vx = curV * cos(curPose.phi);
			curVel.vy = curV * sin(curPose.phi);
			curVel.omega = curW;

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

		virtual void sendNavigationStartEvent() {}
		virtual void sendNavigationEndEvent() {}
		virtual void sendNavigationEndDueToErrorEvent() {}
		virtual void sendWaySeemsBlockedEvent() {}
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
		  m_1st_time_init(false),
		  m_target_allowed_distance(0.40f),
		  m_nav_period(0.100),
		  m_pub_topic_reactive_nav_goal("reactive_nav_goal"),
		  m_sub_topic_local_obstacles("local_map_pointcloud"),
		  m_sub_topic_robot_shape(""),
		  m_frameid_reference("/map"),
		  m_frameid_robot("base_link"),
		  m_save_nav_log(false),
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
		m_localn.param("save_nav_log", m_save_nav_log, m_save_nav_log);

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
			m_sub_robot_shape = m_nh.subscribe<geometry_msgs::Polygon>(
				m_sub_topic_robot_shape, 1,
				&ReactiveNav2DNode::onRosSetRobotShape, this);
			ROS_INFO(
				"Params say robot shape will arrive via topic '%s'... waiting "
				"3 seconds for it.",
				m_sub_topic_robot_shape.c_str());
			ros::Duration(3.0).sleep();
			for (size_t i = 0; i < 100; i++) ros::spinOnce();
			ROS_INFO("Wait done.");
		}

		// Init ROS publishers:
		// -----------------------
		m_pub_cmd_vel = m_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

		// Init ROS subs:
		// -----------------------
		// "/reactive_nav_goal", "/move_base_simple/goal" (
		// geometry_msgs/PoseStamped )
		m_sub_nav_goal = m_nh.subscribe<geometry_msgs::PoseStamped>(
			m_pub_topic_reactive_nav_goal, 1,
			&ReactiveNav2DNode::onRosGoalReceived, this);
		m_sub_local_obs = m_nh.subscribe<sensor_msgs::PointCloud>(
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

		m_reactive_nav_engine.navigationStep();
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
			try
			{
				geometry_msgs::PoseStamped trg2;
				m_tf_listener.transformPose(m_frameid_reference, trg, trg2);
				trg = trg2;
			}
			catch (tf::TransformException& ex)
			{
				ROS_ERROR("%s", ex.what());
				return;
			}
		}

		this->navigateTo(mrpt::math::TPose2D(
			trg.pose.position.x, trg.pose.position.y, trg.pose.orientation.z));
	}

	void onRosLocalObstacles(const sensor_msgs::PointCloudConstPtr& obs)
	{
		std::lock_guard<std::mutex> csl(m_last_obstacles_cs);
		mrpt_bridge::point_cloud::ros2mrpt(*obs, m_last_obstacles);
		// ROS_DEBUG("Local obstacles received: %u points", static_cast<unsigned
		// int>(m_last_obstacles.size()) );
	}

	void onRosSetRobotShape(const geometry_msgs::PolygonConstPtr& newShape)
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

};  // end class

int main(int argc, char** argv)
{
	ReactiveNav2DNode the_node(argc, argv);
	ros::spin();
	return 0;
}
