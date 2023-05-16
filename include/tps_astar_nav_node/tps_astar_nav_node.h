#pragma once 

#include <ros/ros.h> 
#include <sstream>
#include <string>
#include <mutex>
#include <functional>
#include <memory>
#include <optional>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TTwist2D.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/ros1bridge/map.h>
#include <mrpt/ros1bridge/time.h>
#include <mrpt/ros1bridge/point_cloud.h>
#include <mrpt/ros1bridge/pose.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/datetime.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/kinematics/CVehicleVelCmd_DiffDriven.h>
#include <mrpt/version.h>
#include <selfdriving/algos/CostEvaluatorCostMap.h>
#include <selfdriving/algos/CostEvaluatorPreferredWaypoint.h>
#include <selfdriving/algos/TPS_Astar.h>
#include <selfdriving/algos/refine_trajectory.h>
#include <selfdriving/interfaces/ObstacleSource.h>
#include <selfdriving/interfaces/VehicleMotionInterface.h>
#include <selfdriving/data/EnqueuedMotionCmd.h>
// #include <selfdriving/algos/NavEngine.h>
#include <selfdriving/data/PlannerOutput.h>
#include <selfdriving/data/MotionPrimitivesTree.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include "tps_astar_nav_node/tps_navigator.h"

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
	ros::NodeHandle m_localn;  //!< "~" for handling params
    std::once_flag m_init_flag; //!< Perform initialization once
    std::once_flag m_map_received_flag; //!< Receive map once
    mrpt::maps::CPointsMap::Ptr m_grid_map; //!< DS for the Gridmap


    mrpt::math::TPose2D m_nav_goal; //!< Navigation Goal position
    mrpt::math::TPose2D m_start_pose; //!< Navigation start position
    mrpt::math::TTwist2D m_start_vel; //!< Robot velocity at start
	selfdriving::VehicleLocalizationState m_localization_pose; //!< DS for localization info
	selfdriving::VehicleOdometryState m_odometry; //!< DS for odometry info
	mrpt::maps::CPointsMap::Ptr m_obstacle_src; //!< DS for holding obstacle info from sensors
	std::once_flag m_init_nav_flag; //!< Initialize Nav Engine once
	bool m_nav_engine_init; //!< Flag to indicate nav engine is initialized
	ros::Timer m_timer_run_nav; //!< Periodic timer for navigator
	ros::Timer m_timer_enqueue; //!< Periodic timer for enqueued command status checker
	double m_nav_period; //!< Time period(Frequency) of Nav Step
	double m_enq_cmd_check_time; //!< Time period(frequency) of enqueued command status checker

    ros::Subscriber m_sub_map; //!< Subscriber to Map
	ros::Subscriber m_sub_localization_pose; //!< Subscriber to localization info
	ros::Subscriber m_sub_odometry; //!< Subscriber to Odometry info from robot
	ros::Subscriber m_sub_obstacles; //!< Subsciber to obstacle map info
	ros::Publisher  m_pub_cmd_vel; //!< Publisher for velocity commands for the robot

	std::string m_sub_map_str; //!< parameterized name for map subscriber
	std::string m_sub_localization_str; //!<parameterized name for localization subscriber
	std::string m_sub_odometry_str; //!< parameterized name for odometry subscriber
	std::string m_sub_obstacles_str; //!< parameterized name for obsctacle source map subscriber
	std::string m_pub_cmd_vel_str; //!< parameterized name for velocity command publisher

	std::mutex m_obstacles_cs; //!< mutex for obstacle data
	std::mutex m_localization_cs; //!< mutex for localization data
	std::mutex m_odometry_cs; //!< mutex for odometry data
	std::mutex m_next_cmd_cs; //!< mutex for enqueued command

	//for debugging
	bool m_debug;
	bool m_gui_mrpt;
	mrpt::gui::CDisplayWindow3D::Ptr m_win_3d;
	mrpt::opengl::COpenGLScene m_scene;

	std::function<void()> on_enqueued_motion_fired; //!< callback for jackal robot when enqueued cmd fired
	std::function<void()> on_enqueued_motion_timeout; //!< callback for jackal robot when enqueud cmd timed out
	mrpt::math::TPose2D m_motion_trigger_pose; //!< Pose at which enqueued cmd should be executed
	mrpt::math::TPose2D m_motion_trigger_tolerance; //!< Pose tolerance for enqueued cmd
	double m_motion_trigger_timeout; //!< Timeout period within which the enqueud cmd should execute
	mrpt::kinematics::CVehicleVelCmd::Ptr m_next_cmd; //!< DS for enqueued motion cmd
	mrpt::kinematics::CVehicleVelCmd::Ptr m_NOP_cmd; //!< DS for NO-op, continue with previous command
	double m_enq_motion_timer; //!< Timer that checks if enqueued cmd timed out

	/**
	 * @brief Jackal Interface is the robot interface for the NavEngine. 
	 * NavEngine algorithms callback into the interface functions to execute Robot ODOA
	 */
	struct Jackal_Interface : public selfdriving::VehicleMotionInterface,
									 selfdriving::ObstacleSource
	{
		TPS_Astar_Nav_Node& m_parent; //!< Top level parent class object
		bool m_enqueued_motion_pending; //!< indicator that enqueued command is pending and needs to be executed
		bool m_enqueued_motion_timeout; //!< indicator that enqueued command timedout
		selfdriving::VehicleOdometryState m_enqueued_motion_trigger_odom;
		std::mutex m_enqueued_motion_mutex; //!< mutex for enqueued commands

		/* Ctor*/
		Jackal_Interface(TPS_Astar_Nav_Node& parent) : 
		m_parent(parent),
		m_enqueued_motion_pending(false),
		m_enqueued_motion_timeout(false) 
		{
			m_parent.on_enqueued_motion_fired = [this](){
				ROS_INFO_STREAM("[Jackal Robot] Enqueud motion fired");
				changeSpeeds(*m_parent.m_next_cmd);
				ROS_INFO_STREAM("[Jackal Robot] change speeds");
				{
					//std::lock_guard<std::mutex> csl(m_enqueued_motion_mutex);
					m_enqueued_motion_pending = false;
					m_enqueued_motion_timeout = false;
				} 
				ROS_INFO_STREAM("Mutex Release");
				return;
			};

			m_parent.on_enqueued_motion_timeout = [this] () {
				ROS_INFO_STREAM("[Jackal Robot] Enqueud motion timedout");
				//std::lock_guard<std::mutex> csl(m_enqueued_motion_mutex);
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
		selfdriving::VehicleLocalizationState get_localization() override
		{
			return m_parent.get_localization_state();
		}

		/** @brief Access method to the vehicle odometry data.
		 * 
		 *
		 * In case of a hardware/communication error, leave `valid=false` in the
		 * return structure.
		 */
		selfdriving::VehicleOdometryState get_odometry() override
		{
			return m_parent.get_odometry_state();
		}

		/**
		 * @brief Refer Parent class for docs 
		 * 
		 */
		bool motion_execute(
        		const std::optional<mrpt::kinematics::CVehicleVelCmd::Ptr>& immediate,
        		const std::optional<selfdriving::EnqueuedMotionCmd>&   next)override
		{
			if (immediate.has_value())
			{
				std::lock_guard<std::mutex> csl(m_enqueued_motion_mutex);
				ROS_INFO_STREAM("[Jackal_Robot] Motion command received");
				mrpt::kinematics::CVehicleVelCmd& vel_cmd = *(immediate.value());
				changeSpeeds(vel_cmd);
				m_parent.m_NOP_cmd = immediate.value();
				m_enqueued_motion_pending = false;
				m_enqueued_motion_timeout = false;
			}

			if(next.has_value())
			{
				//ROS_INFO_STREAM("[Jackal_Robot] Enqueued command received");
				std::lock_guard<std::mutex> csl(m_enqueued_motion_mutex);
				auto& enq_cmd = next.value();
				m_enqueued_motion_pending = true;
				m_enqueued_motion_timeout = false;
				m_parent.m_motion_trigger_pose = enq_cmd.nextCondition.position;
				m_parent.m_motion_trigger_tolerance = enq_cmd.nextCondition.tolerance;
				m_parent.m_motion_trigger_timeout = enq_cmd.nextCondition.timeout;
				m_parent.m_next_cmd = next->nextCmd;
				m_parent.m_enq_motion_timer = 0.0;
				ROS_INFO_STREAM("[Jackal_Robot] Next cmd timeout = "<<m_parent.m_motion_trigger_timeout
								<< " Enq_Motion_timer = "<< m_parent.m_enq_motion_timer);
			}

			if(!immediate.has_value() && !next.has_value())
			{
				std::lock_guard<std::mutex> csl(m_enqueued_motion_mutex);
				ROS_INFO_STREAM("[Jackal_Robot] NOP command received");
				if(m_parent.m_NOP_cmd)
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

		std::optional<selfdriving::VehicleOdometryState> enqued_motion_last_odom_when_triggered() const override
    	{
			auto lck = mrpt::lockHelper(m_enqueued_motion_mutex);
        	return m_enqueued_motion_trigger_odom;
    	}

		bool changeSpeeds(
			const mrpt::kinematics::CVehicleVelCmd& vel_cmd)
		{
			// ROS_INFO_STREAM("[Jackal_Robot] change speeds");
			using namespace mrpt::kinematics;
			const CVehicleVelCmd_DiffDriven* vel_cmd_diff_driven =
				dynamic_cast<const CVehicleVelCmd_DiffDriven*>(&vel_cmd);
			ASSERT_(vel_cmd_diff_driven);

			const double v = vel_cmd_diff_driven->lin_vel;
			const double w = vel_cmd_diff_driven->ang_vel;
			ROS_INFO_STREAM(
				"changeSpeeds: v= "<< v <<"m/s and w="<<w * 180.0f / M_PI<<"deg/s");
			geometry_msgs::Twist cmd;
			cmd.linear.x = v;
			cmd.angular.z = w;
			m_parent.publish_cmd_vel(cmd);
			return true;
		}


		void stop(const selfdriving::STOP_TYPE stopType)override
		{
			if (stopType == selfdriving::STOP_TYPE::EMERGENCY) 
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

		void stop_watchdog()override
		{
			ROS_INFO_STREAM("TPS_Astar_Navigator watchdog timer stopped");	
		}

		void start_watchdog(const size_t periodMilliseconds) override
		{
			ROS_INFO_STREAM("TPS_Astar_Navigator start watchdog timer");
		}

		void on_nav_end_due_to_error()override
		{
			ROS_INFO_STREAM("[TPS_Astar_Navigator] Nav End due to error");
			stop(selfdriving::STOP_TYPE::EMERGENCY);
		}
    	
		void on_nav_start()override
		{
			ROS_INFO_STREAM("[TPS_Astar_Navigator] Nav starting to Goal ="<<m_parent.m_nav_goal.asString());
		}

    	void on_nav_end()override
		{
			ROS_INFO_STREAM("[TPS_Astar_Navigator] Nav End");
			stop(selfdriving::STOP_TYPE::REGULAR);

		}

    	void on_path_seems_blocked()override
		{
			ROS_INFO_STREAM("[TPS_Astar_Navigator] Path Seems blocked");
			stop(selfdriving::STOP_TYPE::REGULAR);
		}

    	void on_apparent_collision()override
		{
			ROS_INFO_STREAM("[TPS_Astar_Navigator] Apparent collision");
			//stop(selfdriving::STOP_TYPE::REGULAR);
		}

		mrpt::maps::CPointsMap::Ptr obstacles( [[maybe_unused]]
                        mrpt::system::TTimeStamp t = mrpt::system::TTimeStamp())override
		{
			return m_parent.get_current_obstacles();
		}

		CObject* clone() const override{return nullptr;}

	};

	std::shared_ptr<Jackal_Interface> m_jackal_robot;

	selfdriving::PlannerOutput m_activePlanOutput;
	std::vector<selfdriving::CostEvaluator::Ptr> m_costEvaluators;
	bool m_path_plan_done;

	std::shared_ptr<selfdriving::TPS_Navigator> m_nav_engine;
	selfdriving::WaypointSequence m_waypts;	 //<! DS for waypoints
	selfdriving::WaypointStatusSequence m_wayptsStatus;	 //<! DS for waypoint status

	public: 
	TPS_Astar_Nav_Node(int argc, char** argv);
	~TPS_Astar_Nav_Node(){};
	template <typename T>
	std::vector<T> processStringParam(const std::string& param_str);
	/* Define callbacks*/
	void callbackMap(const nav_msgs::OccupancyGrid& _map);
	void callbackLocalization(const geometry_msgs::PoseWithCovarianceStamped& _pose);
	void callbackOdometry(const nav_msgs::Odometry& _odom);
	void callbackObstacles(const sensor_msgs::PointCloud& _pc);
	/* update methods*/
	void updateMap(const nav_msgs::OccupancyGrid& msg);
	void updateLocalization(const geometry_msgs::PoseWithCovarianceStamped& _pose);
	void updateOdom(const nav_msgs::Odometry& _odom);
	void updateObstacles(const sensor_msgs::PointCloud& _pc);

	bool do_path_plan();
	void init3DDebug();

	/*Getter functions for Robot interface*/
	selfdriving::VehicleLocalizationState get_localization_state();
	selfdriving::VehicleOdometryState get_odometry_state();
	mrpt::maps::CPointsMap::Ptr get_current_obstacles();
	void publish_cmd_vel(const geometry_msgs::Twist& cmd_vel);

	/* Navigator methods*/
	void initializeNavigator();
	void navigateTo(const mrpt::math::TPose2D& target);
	void onDoNavigation(const ros::TimerEvent&);
	void checkEnqueuedMotionCmds(const ros::TimerEvent&);
};