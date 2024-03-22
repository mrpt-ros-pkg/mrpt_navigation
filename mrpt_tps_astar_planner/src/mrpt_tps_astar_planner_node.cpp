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
#include <mrpt/version.h>
#include <mrpt_tps_astar_planner/mrpt_tps_astar_planner_node.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <functional>
#include <geometry_msgs/msg/PoseStamped.hpp>
#include <geometry_msgs/msg/PoseWithCovarianceStamped.hpp>
#include <geometry_msgs/msg/Twist.hpp>
#include <geometry_msgs/msh/PoseArray.hpp>
#include <memory>
#include <mrpt_msgs/msg/Waypoint.hpp>
#include <mrpt_msgs/msg/WaypointSequence.hpp>
#include <mutex>
#include <nav_msgs/msg/OccupancyGrid.hpp>
#include <nav_msgs/msg/Odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/PointCloud.hpp>
#include <sstream>
#include <std_msgs/msg/Bool.hpp>
#include <string>

// for debugging
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/stock_objects.h>

class TPS_Astar_Nav_Node
{
   private:
	struct TAuxInitializer
	{
		TAuxInitializer(int argc, char** argv)
		{
			ros::init(argc, argv, "tps_astar_nav");
		}
	};

	mrpt::system::CTimeLogger m_profiler;
	TAuxInitializer m_auxinit;	//!< Just to make sure ROS is init first
	ros::NodeHandle m_nh;  //!< The node handle

	std::once_flag m_init_flag;	 //!< Perform initialization once
	std::once_flag m_map_received_flag;	 //!< Receive map once
	mrpt::maps::CPointsMap::Ptr m_grid_map;	 //!< DS for the Gridmap
	mrpt_msgs::WaypointSequence m_wps_msg;

	mrpt::math::TPose2D m_nav_goal;	 //!< Navigation Goal position
	mrpt::math::TPose2D m_start_pose;  //!< Navigation start position
	mrpt::math::TTwist2D m_start_vel;  //!< Robot velocity at start
	mpp::VehicleLocalizationState
		m_localization_pose;  //!< DS for localization info
	mpp::VehicleOdometryState m_odometry;  //!< DS for odometry info
	mrpt::maps::CPointsMap::Ptr
		m_obstacle_src;	 //!< DS for holding obstacle info from sensors
	std::once_flag m_init_nav_flag;	 //!< Initialize Nav Engine once
	bool m_nav_engine_init;	 //!< Flag to indicate nav engine is initialized
	ros::Timer m_timer_run_nav;	 //!< Periodic timer for navigator
	ros::Timer m_timer_enqueue;	 //!< Periodic timer for enqueued command status
								 //!< checker
	double m_nav_period;  //!< Time period(Frequency) of Nav Step
	double m_enq_cmd_check_time;  //!< Time period(frequency) of enqueued
								  //!< command status checker

	ros::Subscriber m_sub_map;	//!< Subscriber to Map
	ros::Subscriber
		m_sub_localization_pose;  //!< Subscriber to localization info
	ros::Subscriber m_sub_odometry;	 //!< Subscriber to Odometry info from robot
	ros::Subscriber m_sub_obstacles;  //!< Subsciber to obstacle map info
	ros::Subscriber
		m_sub_replan;  //!< Subscribe to topic from rnav that tells to replan

	ros::Publisher
		m_pub_cmd_vel;	//!< Publisher for velocity commands for the robot
	ros::Publisher m_pub_wp_seq;  //!< Publisher for Waypoint sequence

	std::string m_sub_map_str;	//!< parameterized name for map subscriber
	std::string m_sub_localization_str;	 //!< parameterized name for
										 //!< localization subscriber
	std::string
		m_sub_odometry_str;	 //!< parameterized name for odometry subscriber
	std::string m_sub_obstacles_str;  //!< parameterized name for obsctacle
									  //!< source map subscriber
	std::string m_sub_replan_str;  //!< parameterized name for replan subscriber

	std::string m_pub_cmd_vel_str;	//!< parameterized name for velocity command
									//!< publisher
	std::string m_pub_wp_seq_str;  //!< parameterized name for waypoint sequence
								   //!< publisher

	std::mutex m_obstacles_cs;	//!< mutex for obstacle data
	std::mutex m_localization_cs;  //!< mutex for localization data
	std::mutex m_odometry_cs;  //!< mutex for odometry data
	std::mutex m_next_cmd_cs;  //!< mutex for enqueued command

	// for debugging
	bool m_debug;
	bool m_gui_mrpt;
	std::string m_gui_mrpt_str;
	mrpt::gui::CDisplayWindow3D::Ptr m_win_3d;
	mrpt::opengl::COpenGLScene m_scene;

	std::function<void()>
		on_enqueued_motion_fired;  //!< callback for jackal robot when enqueued
								   //!< cmd fired
	std::function<void()>
		on_enqueued_motion_timeout;	 //!< callback for jackal robot when enqueud
									 //!< cmd timed out
	mrpt::math::TPose2D m_motion_trigger_pose;	//!< Pose at which enqueued cmd
												//!< should be executed
	mrpt::math::TPose2D
		m_motion_trigger_tolerance;	 //!< Pose tolerance for enqueued cmd
	double m_motion_trigger_timeout;  //!< Timeout period within which the
									  //!< enqueud cmd should execute
	mrpt::kinematics::CVehicleVelCmd::Ptr
		m_next_cmd;	 //!< DS for enqueued motion cmd
	mrpt::kinematics::CVehicleVelCmd::Ptr
		m_NOP_cmd;	//!< DS for NO-op, continue with previous command
	double m_enq_motion_timer;	//!< Timer that checks if enqueued cmd timed out

	/**
	 * @brief Jackal Interface is the robot interface for the NavEngine.
	 * NavEngine algorithms callback into the interface functions to execute
	 * Robot ODOA
	 */
	struct Jackal_Interface : public mpp::VehicleMotionInterface,
							  mpp::ObstacleSource
	{
		TPS_Astar_Nav_Node& m_parent;  //!< Top level parent class object
		bool m_enqueued_motion_pending;	 //!< indicator that enqueued command is
										 //!< pending and needs to be executed
		bool m_enqueued_motion_timeout;	 //!< indicator that enqueued command
										 //!< timedout
		mpp::VehicleOdometryState m_enqueued_motion_trigger_odom;
		std::mutex m_enqueued_motion_mutex;	 //!< mutex for enqueued commands

		/* Ctor*/
		Jackal_Interface(TPS_Astar_Nav_Node& parent)
			: m_parent(parent),
			  m_enqueued_motion_pending(false),
			  m_enqueued_motion_timeout(false)
		{
			m_parent.on_enqueued_motion_fired = [this]() {
				ROS_INFO_STREAM("[Jackal Robot] Enqueud motion fired");
				changeSpeeds(*m_parent.m_next_cmd);
				ROS_INFO_STREAM("[Jackal Robot] change speeds");
				{
					// std::lock_guard<std::mutex> csl(m_enqueued_motion_mutex);
					m_enqueued_motion_pending = false;
					m_enqueued_motion_timeout = false;
				}
				ROS_INFO_STREAM("Mutex Release");
				return;
			};

			m_parent.on_enqueued_motion_timeout = [this]() {
				ROS_INFO_STREAM("[Jackal Robot] Enqueud motion timedout");
				// std::lock_guard<std::mutex> csl(m_enqueued_motion_mutex);
				m_enqueued_motion_pending = false;
				m_enqueued_motion_timeout = true;
				return;
			};
		}

		/**
		 * @brief Provides access to the vehicle localization data.
		 *
		 * In case of a hardware/communication error, leave `valid=false` in the
		 * return structure.
		 */
		mpp::VehicleLocalizationState get_localization() override
		{
			return m_parent.get_localization_state();
		}

		/** @brief Access method to the vehicle odometry data.
		 *
		 *
		 * In case of a hardware/communication error, leave `valid=false` in the
		 * return structure.
		 */
		mpp::VehicleOdometryState get_odometry() override
		{
			return m_parent.get_odometry_state();
		}

		/**
		 * @brief Refer Parent class for docs
		 *
		 */
		bool motion_execute(
			const std::optional<mrpt::kinematics::CVehicleVelCmd::Ptr>&
				immediate,
			const std::optional<mpp::EnqueuedMotionCmd>& next) override
		{
			if (immediate.has_value())
			{
				std::lock_guard<std::mutex> csl(m_enqueued_motion_mutex);
				ROS_INFO_STREAM("[Jackal_Robot] Motion command received");
				mrpt::kinematics::CVehicleVelCmd& vel_cmd =
					*(immediate.value());
				changeSpeeds(vel_cmd);
				m_parent.m_NOP_cmd = immediate.value();
				m_enqueued_motion_pending = false;
				m_enqueued_motion_timeout = false;
			}

			if (next.has_value())
			{
				// ROS_INFO_STREAM("[Jackal_Robot] Enqueued command received");
				std::lock_guard<std::mutex> csl(m_enqueued_motion_mutex);
				auto& enq_cmd = next.value();
				m_enqueued_motion_pending = true;
				m_enqueued_motion_timeout = false;
				m_parent.m_motion_trigger_pose = enq_cmd.nextCondition.position;
				m_parent.m_motion_trigger_tolerance =
					enq_cmd.nextCondition.tolerance;
				m_parent.m_motion_trigger_timeout =
					enq_cmd.nextCondition.timeout;
				m_parent.m_next_cmd = next->nextCmd;
				m_parent.m_enq_motion_timer = 0.0;
				ROS_INFO_STREAM(
					"[Jackal_Robot] Next cmd timeout = "
					<< m_parent.m_motion_trigger_timeout
					<< " Enq_Motion_timer = " << m_parent.m_enq_motion_timer);
			}

			if (!immediate.has_value() && !next.has_value())
			{
				std::lock_guard<std::mutex> csl(m_enqueued_motion_mutex);
				ROS_INFO_STREAM("[Jackal_Robot] NOP command received");
				if (m_parent.m_NOP_cmd)
				{
					changeSpeeds(*m_parent.m_NOP_cmd);
					return true;
				}
			}

			return true;
		}

		bool supports_enqeued_motions() const override { return true; }

		bool enqeued_motion_pending() const override
		{
			auto lck = mrpt::lockHelper(m_enqueued_motion_mutex);
			// ROS_INFO_STREAM("Enqueued motion command pending ?"<<
			// 				(m_enqueued_motion_pending?"True":"False"));
			return m_enqueued_motion_pending;
		}

		bool enqeued_motion_timed_out() const override
		{
			auto lck = mrpt::lockHelper(m_enqueued_motion_mutex);
			// ROS_INFO_STREAM("Enqueued motion command timed out ?"<<
			// 				(m_enqueued_motion_timeout?"True":"False"));
			return m_enqueued_motion_timeout;
		}

		std::optional<mpp::VehicleOdometryState>
			enqued_motion_last_odom_when_triggered() const override
		{
			auto lck = mrpt::lockHelper(m_enqueued_motion_mutex);
			return m_enqueued_motion_trigger_odom;
		}

		bool changeSpeeds(const mrpt::kinematics::CVehicleVelCmd& vel_cmd)
		{
			// ROS_INFO_STREAM("[Jackal_Robot] change speeds");
			using namespace mrpt::kinematics;
			const CVehicleVelCmd_DiffDriven* vel_cmd_diff_driven =
				dynamic_cast<const CVehicleVelCmd_DiffDriven*>(&vel_cmd);
			ASSERT_(vel_cmd_diff_driven);

			const double v = vel_cmd_diff_driven->lin_vel;
			const double w = vel_cmd_diff_driven->ang_vel;
			ROS_INFO_STREAM(
				"changeSpeeds: v= " << v << "m/s and w=" << w * 180.0f / M_PI
									<< "deg/s");
			geometry_msgs::Twist cmd;
			cmd.linear.x = v;
			cmd.angular.z = w;
			m_parent.publish_cmd_vel(cmd);
			return true;
		}

		void stop(const mpp::STOP_TYPE stopType) override
		{
			if (stopType == mpp::STOP_TYPE::EMERGENCY)
			{
				const auto cmd = getEmergencyStopCmd();
				m_enqueued_motion_pending = false;
				m_enqueued_motion_timeout = false;
				changeSpeeds(*cmd);
			}
			else
			{
				const auto cmd = getStopCmd();
				m_enqueued_motion_pending = false;
				m_enqueued_motion_timeout = false;
				changeSpeeds(*cmd);
			}
		}

		mrpt::kinematics::CVehicleVelCmd::Ptr getStopCmd()
		{
			mrpt::kinematics::CVehicleVelCmd::Ptr vel =
				mrpt::kinematics::CVehicleVelCmd::Ptr(
					new mrpt::kinematics::CVehicleVelCmd_DiffDriven);
			vel->setToStop();
			return vel;
		}

		mrpt::kinematics::CVehicleVelCmd::Ptr getEmergencyStopCmd()
		{
			return getStopCmd();
		}

		void stop_watchdog() override
		{
			ROS_INFO_STREAM("TPS_Astar_Navigator watchdog timer stopped");
		}

		void start_watchdog(const size_t periodMilliseconds) override
		{
			ROS_INFO_STREAM("TPS_Astar_Navigator start watchdog timer");
		}

		void on_nav_end_due_to_error() override
		{
			ROS_INFO_STREAM("[TPS_Astar_Navigator] Nav End due to error");
			stop(mpp::STOP_TYPE::EMERGENCY);
		}

		void on_nav_start() override
		{
			ROS_INFO_STREAM(
				"[TPS_Astar_Navigator] Nav starting to Goal ="
				<< m_parent.m_nav_goal.asString());
		}

		void on_nav_end() override
		{
			ROS_INFO_STREAM("[TPS_Astar_Navigator] Nav End");
			stop(mpp::STOP_TYPE::REGULAR);
		}

		void on_path_seems_blocked() override
		{
			ROS_INFO_STREAM("[TPS_Astar_Navigator] Path Seems blocked");
			stop(mpp::STOP_TYPE::REGULAR);
		}

		void on_apparent_collision() override
		{
			ROS_INFO_STREAM("[TPS_Astar_Navigator] Apparent collision");
			// stop(mpp::STOP_TYPE::REGULAR);
		}

		mrpt::maps::CPointsMap::Ptr obstacles(
			[[maybe_unused]] mrpt::system::TTimeStamp t =
				mrpt::system::TTimeStamp()) override
		{
			return m_parent.get_current_obstacles();
		}

		CObject* clone() const override { return nullptr; }
	};

	std::shared_ptr<Jackal_Interface> m_jackal_robot;

	mpp::PlannerOutput m_activePlanOutput;
	std::vector<mpp::CostEvaluator::Ptr> m_costEvaluators;
	bool m_path_plan_done;

	std::shared_ptr<mpp::TPS_Navigator> m_nav_engine;
	mpp::WaypointSequence m_waypts;	 //<! DS for waypoints
	mpp::WaypointStatusSequence m_wayptsStatus;	 //<! DS for waypoint status

	bool stringToBool(const std::string& str);

   public:
	TPS_Astar_Nav_Node(int argc, char** argv);
	~TPS_Astar_Nav_Node(){};
	template <typename T>
	std::vector<T> processStringParam(const std::string& param_str);
	/* Define callbacks*/
	void callbackMap(const nav_msgs::OccupancyGrid& _map);
	void callbackLocalization(
		const geometry_msgs::PoseWithCovarianceStamped& _pose);
	void callbackOdometry(const nav_msgs::Odometry& _odom);
	void callbackObstacles(const sensor_msgs::PointCloud& _pc);
	void callbackReplan(const std_msgs::Bool& _flag);
	/* update methods*/
	void updateMap(const nav_msgs::OccupancyGrid& _msg);
	void updateLocalization(
		const geometry_msgs::PoseWithCovarianceStamped& _pose);
	void updateOdom(const nav_msgs::Odometry& _odom);
	void updateObstacles(const sensor_msgs::PointCloud& _pc);

	bool do_path_plan(mrpt::math::TPose2D& start, mrpt::math::TPose2D& goal);
	void init3DDebug();

	/*Getter functions for Robot interface*/
	mpp::VehicleLocalizationState get_localization_state();
	mpp::VehicleOdometryState get_odometry_state();
	mrpt::maps::CPointsMap::Ptr get_current_obstacles();
	void publish_cmd_vel(const geometry_msgs::Twist& cmd_vel);
	void publish_waypoint_sequence(const mrpt_msgs::WaypointSequence& wps);

	/* Navigator methods*/
	void initializeNavigator();
	void navigateTo(const mrpt::math::TPose2D& target);
	void onDoNavigation(const ros::TimerEvent&);
	void checkEnqueuedMotionCmds(const ros::TimerEvent&);
};

TPS_Astar_Nav_Node::TPS_Astar_Nav_Node(int argc, char** argv)
	: m_auxinit(argc, argv),
	  m_nh(),
	  m_localn("~"),
	  m_nav_goal(mrpt::math::TPose2D(0.0, 0.0, 0.0)),
	  m_start_pose(mrpt::math::TPose2D(0.0, 0.0, 0.0)),
	  m_start_vel(mrpt::math::TTwist2D(0.0, 0.0, 0.0)),
	  m_debug(true),
	  m_gui_mrpt(false),
	  m_nav_period(1.00),
	  m_nav_engine_init(false),
	  m_path_plan_done(false),
	  m_motion_trigger_timeout(0.0),
	  m_enq_motion_timer(0.0),
	  m_enq_cmd_check_time(0.020)
{
	std::string nav_goal_str = "[0.0, 0.0, 0.0]";
	std::string start_pose_str = "[0.0, 0.0, 0.0]";
	std::string vel_str = "2.0";
	m_localn.param("nav_goal", nav_goal_str, nav_goal_str);
	std::vector<double> goal_pose = processStringParam<double>(nav_goal_str);
	if (goal_pose.size() != 3)
	{
		ROS_ERROR("Invalid nav_goal parameter.");
		return;
	}
	m_nav_goal = mrpt::math::TPose2D(goal_pose[0], goal_pose[1], goal_pose[2]);
	ROS_INFO_STREAM(
		"[TPS_Astar_Nav_Node]nav goal received =" << m_nav_goal.asString());

	m_localn.param("start_pose", start_pose_str, start_pose_str);
	std::vector<double> start_pose = processStringParam<double>(start_pose_str);
	if (start_pose.size() != 3)
	{
		ROS_ERROR("Invalid start pose parameter.");
		return;
	}
	m_start_pose =
		mrpt::math::TPose2D(start_pose[0], start_pose[1], start_pose[2]);
	ROS_INFO_STREAM(
		"[TPS_Astar_Nav_Node] start pose received ="
		<< m_start_pose.asString());

	m_localn.param("start_vel", vel_str, vel_str);
	m_start_vel = mrpt::math::TTwist2D(std::stod(vel_str), 0.0, 0.0);
	ROS_INFO_STREAM(
		"[TPS_Astar_Nav_Node]starting velocity =" << m_start_vel.asString());

	m_localn.param("mrpt_gui", m_gui_mrpt_str, m_gui_mrpt_str);
	if (!m_gui_mrpt_str.empty())
		m_gui_mrpt = stringToBool(m_gui_mrpt_str);
	else
		m_gui_mrpt = false;

	m_localn.param("topic_map_sub", m_sub_map_str, m_sub_map_str);
	m_sub_map = m_nh.subscribe(
		m_sub_map_str, 1, &TPS_Astar_Nav_Node::callbackMap, this);

	m_localn.param(
		"topic_localization_sub", m_sub_localization_str,
		m_sub_localization_str);
	m_sub_localization_pose = m_nh.subscribe(
		m_sub_localization_str, 1, &TPS_Astar_Nav_Node::callbackLocalization,
		this);

	m_localn.param(
		"topic_odometry_sub", m_sub_odometry_str, m_sub_odometry_str);
	m_sub_odometry = m_nh.subscribe(
		m_sub_odometry_str, 1, &TPS_Astar_Nav_Node::callbackOdometry, this);

	m_localn.param(
		"topic_obstacles_sub", m_sub_obstacles_str, m_sub_obstacles_str);
	m_sub_obstacles = m_nh.subscribe(
		m_sub_obstacles_str, 1, &TPS_Astar_Nav_Node::callbackObstacles, this);

	m_localn.param("topic_replan_sub", m_sub_replan_str, m_sub_replan_str);
	m_sub_replan = m_nh.subscribe(
		m_sub_replan_str, 1, &TPS_Astar_Nav_Node::callbackReplan, this);

	m_localn.param("topic_cmd_vel_pub", m_pub_cmd_vel_str, m_pub_cmd_vel_str);
	m_pub_cmd_vel = m_nh.advertise<geometry_msgs::Twist>(m_pub_cmd_vel_str, 1);

	m_localn.param("topic_wp_seq_pub", m_pub_wp_seq_str, m_pub_wp_seq_str);
	m_pub_wp_seq =
		m_nh.advertise<mrpt_msgs::WaypointSequence>(m_pub_wp_seq_str, 1);

	// Init timers:
	m_timer_run_nav = m_nh.createTimer(
		ros::Duration(m_nav_period), &TPS_Astar_Nav_Node::onDoNavigation, this);

	// Odometry publisher runs at 50Hz, so this functions runs at the same
	// periodicity
	m_timer_enqueue = m_nh.createTimer(
		ros::Duration(m_enq_cmd_check_time),
		&TPS_Astar_Nav_Node::checkEnqueuedMotionCmds, this);

	if (m_nav_engine)
	{
		ROS_INFO_STREAM(
			"TPS Astart Navigator already initialized, resetting nav engine");
		m_nav_engine.reset();
	}

	m_nav_engine = std::make_shared<mpp::TPS_Navigator>();

	m_jackal_robot = std::make_shared<Jackal_Interface>(*this);
}

bool TPS_Astar_Nav_Node::stringToBool(const std::string& str)
{
	if (str == "true" || str == "True" || str == "TRUE" || str == "1")
		return true;
	else
		return false;
	//    if(str == "false" || str == "False" || str == "FALSE" || str == "0")
	//    return false;
}
template <typename T>
std::vector<T> TPS_Astar_Nav_Node::processStringParam(
	const std::string& param_str)
{
	std::string str = param_str;
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
	// ROS_INFO_STREAM("Navigator Map received for planning");
	std::call_once(
		m_map_received_flag, [this, _map]() { this->updateMap(_map); });
}

void TPS_Astar_Nav_Node::callbackLocalization(
	const geometry_msgs::PoseWithCovarianceStamped& _pose)
{
	updateLocalization(_pose);
}

void TPS_Astar_Nav_Node::callbackOdometry(const nav_msgs::Odometry& _odom)
{
	updateOdom(_odom);
}

void TPS_Astar_Nav_Node::callbackObstacles(const sensor_msgs::PointCloud& _pc)
{
	updateObstacles(_pc);
}

void TPS_Astar_Nav_Node::callbackReplan(const std_msgs::Bool& _flag)
{
	auto& pose = m_localization_pose;

	if (pose.valid && _flag.data)
	{
		m_path_plan_done = do_path_plan(pose.pose, m_nav_goal);
	}
}

void TPS_Astar_Nav_Node::publish_cmd_vel(const geometry_msgs::Twist& cmd_vel)
{
	ROS_INFO_STREAM("Publishing velocity command" << cmd_vel);
	m_pub_cmd_vel.publish(cmd_vel);
}

void TPS_Astar_Nav_Node::publish_waypoint_sequence(
	const mrpt_msgs::WaypointSequence& wps)
{
	m_pub_wp_seq.publish(wps);
}

void TPS_Astar_Nav_Node::init3DDebug()
{
	ROS_INFO("init3DDebug");

	if (!m_win_3d)
	{
		m_win_3d = mrpt::gui::CDisplayWindow3D::Create(
			"Pathplanning-TPS-AStar", 1000, 600);
		m_win_3d->setCameraZoom(20);
		m_win_3d->setCameraAzimuthDeg(-45);

		auto plane = m_grid_map->getVisualization();
		m_scene.insert(plane);

		{
			mrpt::opengl::COpenGLScene::Ptr ptr_scene =
				m_win_3d->get3DSceneAndLock();

			ptr_scene->insert(plane);

			ptr_scene->enableFollowCamera(true);

			m_win_3d->unlockAccess3DScene();
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
	std::lock_guard<std::mutex> csl(m_localization_cs);
	m_localization_pose.frame_id = msg.header.frame_id;
	m_localization_pose.valid = true;
	m_localization_pose.pose.x = msg.pose.pose.position.x;
	m_localization_pose.pose.y = msg.pose.pose.position.y;
	m_localization_pose.pose.phi = yaw;
	m_localization_pose.timestamp = mrpt::ros1bridge::fromROS(msg.header.stamp);
	// ROS_INFO_STREAM("Localization update complete");
}

void TPS_Astar_Nav_Node::updateOdom(const nav_msgs::Odometry& msg)
{
	tf2::Quaternion quat(
		msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
		msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
	tf2::Matrix3x3 mat(quat);
	double roll, pitch, yaw;
	mat.getRPY(roll, pitch, yaw);
	std::lock_guard<std::mutex> csl(m_odometry_cs);
	m_odometry.odometry.x = msg.pose.pose.position.x;
	m_odometry.odometry.y = msg.pose.pose.position.y;
	m_odometry.odometry.phi = yaw;

	m_odometry.odometryVelocityLocal.vx = msg.twist.twist.linear.x;
	m_odometry.odometryVelocityLocal.vy = msg.twist.twist.linear.y;
	m_odometry.odometryVelocityLocal.omega = msg.twist.twist.angular.z;

	m_odometry.valid = true;
	m_odometry.timestamp = mrpt::system::now();
	/*TODO*/
	m_odometry.pendedActionExists = m_jackal_robot->enqeued_motion_pending();
	// ROS_INFO_STREAM("Odometry update complete");
}

void TPS_Astar_Nav_Node::updateObstacles(const sensor_msgs::PointCloud& _pc)
{
	mrpt::maps::CSimplePointsMap point_cloud;
	if (!mrpt::ros1bridge::fromROS(_pc, point_cloud))
	{
		ROS_ERROR("Failed to convert Point Cloud to MRPT Points Map");
	}
	std::lock_guard<std::mutex> csl(m_obstacles_cs);
	m_obstacle_src = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(
		std::make_shared<mrpt::maps::CSimplePointsMap>(point_cloud));

	// ROS_INFO_STREAM("Obstacles update complete");
}

void TPS_Astar_Nav_Node::updateMap(const nav_msgs::OccupancyGrid& msg)
{
	mrpt::maps::COccupancyGridMap2D grid;
	// ASSERT_(grid.countMapsByClass<mrpt::maps::COccupancyGridMap2D>());
	mrpt::ros1bridge::fromROS(msg, grid);
	auto obsPts = mrpt::maps::CSimplePointsMap::Create();
	grid.getAsPointCloud(*obsPts);
	ROS_INFO_STREAM("Setting gridmap for planning");
	m_grid_map = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(obsPts);

	m_path_plan_done = do_path_plan(m_start_pose, m_nav_goal);
}

void TPS_Astar_Nav_Node::initializeNavigator()
{
	if (!m_nav_engine)
	{
		ROS_ERROR("TPS_AStar Not created!");
		return;
	}

	m_nav_engine->setMinLoggingLevel(mrpt::system::VerbosityLevel::LVL_DEBUG);
	m_nav_engine->config_.vehicleMotionInterface =
		std::dynamic_pointer_cast<mpp::VehicleMotionInterface>(
			/*std::make_shared<Jackal_Interface>*/ (m_jackal_robot));
	m_nav_engine->config_.vehicleMotionInterface->setMinLoggingLevel(
		mrpt::system::VerbosityLevel::LVL_DEBUG);
	m_nav_engine->config_.globalMapObstacleSource =
		mpp::ObstacleSource::FromStaticPointcloud(m_grid_map);
	m_nav_engine->config_.localSensedObstacleSource =
		mpp::ObstacleSource::FromStaticPointcloud(m_obstacle_src);

	{
		std::string ptg_ini_file;
		m_localn.param("ptg_ini", ptg_ini_file, ptg_ini_file);

		ROS_ASSERT_MSG(
			mrpt::system::fileExists(ptg_ini_file),
			"PTG ini file not found: '%s'", ptg_ini_file.c_str());
		mrpt::config::CConfigFile cfg(ptg_ini_file);
		m_nav_engine->config_.ptgs.initFromConfigFile(cfg, "SelfDriving");
	}

	{
		// cost map:
		std::string costmap_param_file;

		m_localn.param(
			"global_costmap_parameters", costmap_param_file,
			costmap_param_file);

		ROS_ASSERT_MSG(
			mrpt::system::fileExists(costmap_param_file),
			"costmap params file not found: '%s'", costmap_param_file.c_str());

		m_nav_engine->config_.globalCostParameters =
			mpp::CostEvaluatorCostMap::Parameters::FromYAML(
				mrpt::containers::yaml::FromFile(costmap_param_file));

		m_nav_engine->config_.localCostParameters =
			mpp::CostEvaluatorCostMap::Parameters::FromYAML(
				mrpt::containers::yaml::FromFile(costmap_param_file));
	}

	// Preferred waypoints:
	{
		std::string wp_params_file;
		m_localn.param(
			"prefer_waypoints_parameters", wp_params_file, wp_params_file);

		ROS_ASSERT_MSG(
			mrpt::system::fileExists(wp_params_file),
			"Prefer waypoints params file not found: '%s'",
			wp_params_file.c_str());
		m_nav_engine->config_.preferWaypointsParameters =
			mpp::CostEvaluatorPreferredWaypoint::Parameters::FromYAML(
				mrpt::containers::yaml::FromFile(wp_params_file));
	}

	{
		std::string planner_parameters_file;
		m_localn.param(
			"planner_parameters", planner_parameters_file,
			planner_parameters_file);

		ROS_ASSERT_MSG(
			mrpt::system::fileExists(planner_parameters_file),
			"Planner params file not found: '%s'",
			planner_parameters_file.c_str());

		m_nav_engine->config_.plannerParams =
			mpp::TPS_Astar_Parameters::FromYAML(
				mrpt::containers::yaml::FromFile(planner_parameters_file));
	}

	{
		std::string nav_engine_parameters_file;
		m_localn.param(
			"nav_engine_parameters", nav_engine_parameters_file,
			nav_engine_parameters_file);

		ROS_ASSERT_MSG(
			mrpt::system::fileExists(nav_engine_parameters_file),
			"Planner params file not found: '%s'",
			nav_engine_parameters_file.c_str());

		m_nav_engine->config_.loadFrom(
			mrpt::containers::yaml::FromFile(nav_engine_parameters_file));
	}

	m_nav_engine->initialize();

	ROS_INFO_STREAM("TPS_Astar Navigator intialized");

	if (m_path_plan_done)
	{
		navigateTo(m_nav_goal);
	}
	m_nav_engine_init = true;
}

void TPS_Astar_Nav_Node::navigateTo(const mrpt::math::TPose2D& target)
{
	// mpp::Waypoint waypoint(target.x, target.y, 1.0, false, 0.0, 0.0);

	// ROS_INFO_STREAM("[TPS_Astar_Nav_Node] navigateTo"<<waypoint.getAsText());

	// m_waypts.waypoints.push_back(waypoint);

	m_nav_engine->request_navigation(m_waypts);
}

bool TPS_Astar_Nav_Node::do_path_plan(
	mrpt::math::TPose2D& start, mrpt::math::TPose2D& goal)
{
	ROS_INFO_STREAM("Do path planning");
	auto obs = mpp::ObstacleSource::FromStaticPointcloud(m_grid_map);
	mpp::PlannerInput planner_input;

	planner_input.stateStart.pose = start;
	planner_input.stateStart.vel = m_start_vel;
	planner_input.stateGoal.state = goal;
	planner_input.obstacles.emplace_back(obs);
	auto bbox = obs->obstacles()->boundingBox();

	{
		const auto bboxMargin = mrpt::math::TPoint3Df(2.0, 2.0, .0);
		const auto ptStart = mrpt::math::TPoint3Df(
			planner_input.stateStart.pose.x, planner_input.stateStart.pose.y,
			0);
		const auto ptGoal = mrpt::math::TPoint3Df(
			planner_input.stateGoal.asSE2KinState().pose.x,
			planner_input.stateGoal.asSE2KinState().pose.y, 0);
		bbox.updateWithPoint(ptStart - bboxMargin);
		bbox.updateWithPoint(ptStart + bboxMargin);
		bbox.updateWithPoint(ptGoal - bboxMargin);
		bbox.updateWithPoint(ptGoal + bboxMargin);
	}

	planner_input.worldBboxMax = {bbox.max.x, bbox.max.y, M_PI};
	planner_input.worldBboxMin = {bbox.min.x, bbox.min.y, -M_PI};

	std::cout << "Start state: " << planner_input.stateStart.asString() << "\n";
	std::cout << "Goal state : " << planner_input.stateGoal.asString() << "\n";
	std::cout << "Obstacles  : " << obs->obstacles()->size() << " points\n";
	std::cout << "World bbox : " << planner_input.worldBboxMin.asString()
			  << " - " << planner_input.worldBboxMax.asString() << "\n";

	mpp::Planner::Ptr planner = mpp::TPS_Astar::Create();

	// Enable time profiler:
	planner->profiler_().enable(true);

	{
		// cost map:
		std::string costmap_param_file;

		m_localn.param(
			"global_costmap_parameters", costmap_param_file,
			costmap_param_file);

		ROS_ASSERT_MSG(
			mrpt::system::fileExists(costmap_param_file),
			"costmap params file not found: '%s'", costmap_param_file.c_str());

		const auto costMapParams =
			mpp::CostEvaluatorCostMap::Parameters::FromYAML(
				mrpt::containers::yaml::FromFile(costmap_param_file));

		auto costmap = mpp::CostEvaluatorCostMap::FromStaticPointObstacles(
			*m_grid_map, costMapParams, planner_input.stateStart.pose);

		ROS_INFO_STREAM("******************************* Costmap file read");

		planner->costEvaluators_.push_back(costmap);
	}

	{
		std::string planner_parameters_file;
		m_localn.param(
			"planner_parameters", planner_parameters_file,
			planner_parameters_file);

		ROS_ASSERT_MSG(
			mrpt::system::fileExists(planner_parameters_file),
			"Planner params file not found: '%s'",
			planner_parameters_file.c_str());

		const auto c =
			mrpt::containers::yaml::FromFile(planner_parameters_file);
		planner->params_from_yaml(c);
		std::cout << "Loaded these planner params:\n";
		planner->params_as_yaml().printAsYAML();
	}

	// Insert custom progress callback:
	planner->progressCallback_ = [](const mpp::ProgressCallbackData& pcd) {
		std::cout << "[progressCallback] bestCostFromStart: "
				  << pcd.bestCostFromStart
				  << " bestCostToGoal: " << pcd.bestCostToGoal
				  << " bestPathLength: " << pcd.bestPath.size() << std::endl;
	};

	{
		std::string ptg_ini_file;
		m_localn.param("ptg_ini", ptg_ini_file, ptg_ini_file);

		ROS_ASSERT_MSG(
			mrpt::system::fileExists(ptg_ini_file),
			"PTG ini file not found: '%s'", ptg_ini_file.c_str());
		mrpt::config::CConfigFile cfg(ptg_ini_file);
		planner_input.ptgs.initFromConfigFile(cfg, "SelfDriving");

		ROS_INFO_STREAM("PTG ini");
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
		m_activePlanOutput = plan;
		m_costEvaluators = planner->costEvaluators_;
	}

	// backtrack:
	auto [plannedPath, pathEdges] =
		plan.motionTree.backtrack_path(*plan.bestNodeId);

#if 0  // JLBC: disabled to check if this is causing troubles
    mpp::refine_trajectory(plannedPath, pathEdges, planner_input.ptgs);
#endif

	// Show plan in a GUI for debugging
	if (plan.success)  // && m_gui_mrpt
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
		// so, correct that relative pose so we keep everything in global frame:
		const auto& startPose = plan.originalInput.stateStart.pose;
		for (auto& kv : interpPath)
			kv.second.state.pose = startPose + kv.second.state.pose;
	}

	m_wps_msg = mrpt_msgs::WaypointSequence();
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

            m_wps_msg.waypoints.push_back(wp_msg);
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

			m_wps_msg.waypoints.push_back(wp_msg);
		}
#endif

		auto wp_msg = mrpt_msgs::Waypoint();
		wp_msg.target.position.x = m_nav_goal.x;
		wp_msg.target.position.y = m_nav_goal.y;
		wp_msg.target.position.z = 0.0;
		tf2::Quaternion quaternion;
		quaternion.setRPY(0.0, 0.0, m_nav_goal.phi);
		wp_msg.target.orientation.x = quaternion.x();
		wp_msg.target.orientation.y = quaternion.y();
		wp_msg.target.orientation.z = quaternion.z();
		wp_msg.target.orientation.w = quaternion.w();

		wp_msg.allowed_distance = 0.4;	// TODO: Make a param
		wp_msg.allow_skip = false;

		m_wps_msg.waypoints.push_back(wp_msg);
	}
	return plan.success;
}

mpp::VehicleLocalizationState TPS_Astar_Nav_Node::get_localization_state()
{
	std::lock_guard<std::mutex> csl(m_localization_cs);
	return m_localization_pose;
}
mpp::VehicleOdometryState TPS_Astar_Nav_Node::get_odometry_state()
{
	std::lock_guard<std::mutex> csl(m_odometry_cs);
	return m_odometry;
}
mrpt::maps::CPointsMap::Ptr TPS_Astar_Nav_Node::get_current_obstacles()
{
	std::lock_guard<std::mutex> csl(m_obstacles_cs);
	return m_obstacle_src;
}

void TPS_Astar_Nav_Node::onDoNavigation(const ros::TimerEvent&)
{
	if (m_path_plan_done)
	{
		publish_waypoint_sequence(m_wps_msg);
	}

	// if(m_obstacle_src && m_localization_pose.valid)
	// {
	//     std::call_once(m_init_nav_flag,[this]()
	//     {this->initializeNavigator();});
	// }

	// if(m_nav_engine_init)
	// {
	//     try
	//     {
	// 	    m_nav_engine->navigation_step();
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
	// if(m_odometry.valid) // && m_jackal_robot->enqeued_motion_pending())
	// {
	//     auto& odo = m_odometry.odometry;
	//     ROS_INFO_STREAM("Odo: "<<  odo.asString());
	//     auto& pose = m_localization_pose.pose;
	//     ROS_INFO_STREAM("Pose: "<< pose.asString());
	//     ROS_INFO_STREAM("Trigger Pose "<< m_motion_trigger_pose.asString());
	//     if(std::abs(odo.x - m_motion_trigger_pose.x) <
	//     m_motion_trigger_tolerance.x &&
	//        std::abs(odo.y - m_motion_trigger_pose.y) <
	//        m_motion_trigger_tolerance.y && std::abs(odo.phi -
	//        m_motion_trigger_pose.phi) < m_motion_trigger_tolerance.phi)
	//     {
	//         std::lock_guard<std::mutex>
	//         csl(m_jackal_robot->m_enqueued_motion_mutex);
	//         //ROS_INFO_STREAM("Enqueued motion fired");
	//         on_enqueued_motion_fired();
	//         m_jackal_robot->m_enqueued_motion_trigger_odom = m_odometry;
	//         ROS_INFO_STREAM("calling change speeds upon pend action fired");
	//         //m_jackal_robot->changeSpeeds(*m_next_cmd);
	//         ROS_INFO_STREAM("Change speeds complete");
	//         m_NOP_cmd = m_next_cmd;
	//         ROS_INFO_STREAM("Next NOP command set");
	//     }
	//     else
	//     {
	//         ROS_INFO_STREAM("[TPS_Astar_Nav_Node] Enqueued motion timer =
	//         "<<m_enq_motion_timer); m_enq_motion_timer +=
	//         m_enq_cmd_check_time; if(m_jackal_robot->enqeued_motion_pending()
	//         &&
	//           m_enq_motion_timer > m_motion_trigger_timeout)
	//         {
	//             std::lock_guard<std::mutex>
	//             csl(m_jackal_robot->m_enqueued_motion_mutex);
	//             on_enqueued_motion_timeout();
	//             m_enq_motion_timer = 0.0;
	//         }
	//     }
	// }
}

int main(int argc, char** argv)
{
	TPS_Astar_Nav_Node the_node(argc, argv);
	// the_node.do_path_plan();
	ros::spin();
	return 0;
}
