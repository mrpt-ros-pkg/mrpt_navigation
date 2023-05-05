#pragma once 

#include <ros/ros.h> 
#include <sstream>
#include <string>
#include <mutex>
#include <functional>
#include <memory>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TTwist2D.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/ros1bridge/map.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/version.h>
#include <selfdriving/algos/CostEvaluatorCostMap.h>
#include <selfdriving/algos/CostEvaluatorPreferredWaypoint.h>
#include <selfdriving/algos/TPS_Astar.h>
#include <selfdriving/interfaces/ObstacleSource.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

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

    ros::Subscriber m_sub_map;

	//for debugging
	bool m_debug;
	bool m_gui_mrpt;
	mrpt::gui::CDisplayWindow3D::Ptr m_win_3d;
	mrpt::opengl::COpenGLScene m_scene;

	public: 
	TPS_Astar_Nav_Node(int argc, char** argv);
	~TPS_Astar_Nav_Node(){};
	template <typename T>
	std::vector<T> processStringParam(const std::string& param_str);
	void callbackMap(const nav_msgs::OccupancyGrid& _map);
	void updateMap(const nav_msgs::OccupancyGrid& msg);
	void do_path_plan();
	void init3DDebug();
	void show3DDebug();

};