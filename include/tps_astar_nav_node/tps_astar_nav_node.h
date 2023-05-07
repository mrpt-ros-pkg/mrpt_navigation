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
#include <selfdriving/interfaces/ObstacleSource.h>
#include <selfdriving/interfaces/VehicleMotionInterface.h>
#include <selfdriving/algos/NavEngine.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>

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
	ros::NodeHandle m_localn;  //!< "~"
    std::once_flag m_init_flag;
    std::once_flag m_map_received_flag;
    mrpt::maps::CPointsMap::Ptr m_grid_map;


    mrpt::math::TPose2D m_nav_goal;
    mrpt::math::TPose2D m_start_pose;
    mrpt::math::TTwist2D m_start_vel;
	selfdriving::VehicleLocalizationState m_localization_pose;
	selfdriving::VehicleOdometryState m_odometry;
	mrpt::maps::CPointsMap::Ptr m_obstacle_src;

    ros::Subscriber m_sub_map;
	ros::Subscriber m_sub_localization_pose;
	ros::Subscriber m_sub_odometry;
	ros::Subscriber m_sub_obstacles;
	ros::Publisher  m_pub_cmd_vel;

	std::string m_sub_map_str;
	std::string m_sub_localization_str;
	std::string m_sub_odometry_str;
	std::string m_sub_obstacles_str;
	std::string m_pub_cmd_vel_str;

	//for debugging
	bool m_debug;
	bool m_gui_mrpt;
	mrpt::gui::CDisplayWindow3D::Ptr m_win_3d;
	mrpt::opengl::COpenGLScene m_scene;

	struct Jackal_Interface : public selfdriving::VehicleMotionInterface,
									 selfdriving::ObstacleSource
	{
		TPS_Astar_Nav_Node& m_parent;
		bool m_enqueued_motion_pending;
		bool m_enqueued_motion_timeout;
		std::mutex m_enqueued_motion_mutex;

		Jackal_Interface(TPS_Astar_Nav_Node& parent) : m_parent(parent) 
		{

		}

		selfdriving::VehicleLocalizationState get_localization() override
		{
			return m_parent.get_localization_state();
		}

		selfdriving::VehicleOdometryState get_odometry() override
		{
			return m_parent.get_odometry_state();
		}

		bool motion_execute(
        		const std::optional<mrpt::kinematics::CVehicleVelCmd::Ptr>& immediate,
        		const std::optional<selfdriving::EnqueuedMotionCmd>&   next)override
		{

		}

		bool supports_enqeued_motions() const override { return true; }

		bool enqeued_motion_pending() const override
		{
			auto lck = mrpt::lockHelper(m_enqueued_motion_mutex);
			ROS_INFO_STREAM("Enqueued motion command pending ?"<<
							(m_enqueued_motion_pending)?"True":"False");
			return m_enqueued_motion_pending;
		}

		bool enqeued_motion_timed_out() const override
		{
			auto lck = mrpt::lockHelper(m_enqueued_motion_mutex);
			ROS_INFO_STREAM("Enqueued motion command timed out ?"<<
							(m_enqueued_motion_timeout)?"True":"False");
			return m_enqueued_motion_timeout;
		}

		bool changeSpeeds(
			const mrpt::kinematics::CVehicleVelCmd& vel_cmd)
		{
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

		}
    	
		void on_nav_start()override
		{

		}

    	void on_nav_end()override
		{

		}

    	void on_path_seems_blocked()override
		{

		}

    	void on_apparent_collision()override
		{

		}

		mrpt::maps::CPointsMap::Ptr obstacles( [[maybe_unused]]
                        mrpt::system::TTimeStamp t = mrpt::system::TTimeStamp())override
		{
			return m_parent.get_current_obstacles();
		}

	};

	public: 
	TPS_Astar_Nav_Node(int argc, char** argv);
	~TPS_Astar_Nav_Node(){};
	template <typename T>
	std::vector<T> processStringParam(const std::string& param_str);
	void callbackMap(const nav_msgs::OccupancyGrid& _map);
	void callbackLocalization(const geometry_msgs::PoseWithCovarianceStamped& _pose);
	void callbackOdometry(const nav_msgs::Odometry& _odom);
	void callbackObstacles(const sensor_msgs::PointCloud& _pc);
	void updateMap(const nav_msgs::OccupancyGrid& msg);
	void updateLocalization(const geometry_msgs::PoseWithCovarianceStamped& _pose);
	void updateOdom(const nav_msgs::Odometry& _odom);
	void updateObstacles(const sensor_msgs::PointCloud& _pc);
	void do_path_plan();
	void init3DDebug();
	selfdriving::VehicleLocalizationState get_localization_state() const{ return m_localization_pose;}
	selfdriving::VehicleOdometryState get_odometry_state() const{ return m_odometry;}
	mrpt::maps::CPointsMap::Ptr get_current_obstacles() const{ return m_obstacle_src; }
	void publish_cmd_vel(const geometry_msgs::Twist& cmd_vel);

};