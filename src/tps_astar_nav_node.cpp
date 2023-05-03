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
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/ros1bridge/map.h>
#include <selfdriving/algos/TPS_Astar.h>
#include <selfdriving/interfaces/ObstacleSource.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>


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
    //ros::Subscriber m_sub_map_meta_data;



    public:
    TPS_Astar_Nav_Node(int argc, char** argv):
                m_auxinit(argc, argv),
                m_nh(),
                m_localn("~"),
                m_nav_goal(mrpt::math::TPose2D(0.0, 0.0, 0.0)),
                m_start_pose(mrpt::math::TPose2D(0.0, 0.0, 0.0)),
                m_start_vel(mrpt::math::TTwist2D(0.0, 0.0, 0.0))
    {
        std::string nav_goal_str = "[0.0, 0.0, 0.0]";
        std::string start_pose_str = "[0.0, 0.0, 0.0]";
        std::string vel_str = "2.0";
        m_localn.param(
			"nav_goal", nav_goal_str, nav_goal_str);
        std::vector<double>goal_pose = processStringParam<double>(nav_goal_str);
        if (goal_pose.size() != 3) 
        {
            ROS_ERROR("Invalid nav_goal parameter.");
            return;
        }
        m_nav_goal = mrpt::math::TPose2D(goal_pose[0], goal_pose[1], goal_pose[2]);
        std::cout<<"***********************************nav goal received ="<< m_nav_goal.asString()<<std::endl;
        m_localn.param(
            "start_pose", start_pose_str, start_pose_str);
        std::vector<double>start_pose = processStringParam<double>(start_pose_str);
        if (start_pose.size() != 3) 
        {
            ROS_ERROR("Invalid start pose parameter.");
            return;
        }
        m_start_pose = mrpt::math::TPose2D(start_pose[0], start_pose[1], start_pose[2]);
        std::cout<<"***********************************start pose received ="<< m_start_pose.asString()<<std::endl;
        m_localn.param("start_vel", vel_str, vel_str);
        m_start_vel = mrpt::math::TTwist2D(std::stod(vel_str), 0.0, 0.0);
        std::cout<<"***************************** starting velocity ="<< m_start_vel.asString()<<std::endl;

        m_sub_map = m_nh.subscribe("map", 1, &TPS_Astar_Nav_Node::callbackMap, this);
        //m_sub_map_meta_data = m_nh.subscribe("map_metadata", 1, &TPS_Astar_Nav_Node::callbackMapMetaData, this);



    }

    template <typename T>
    std::vector<T> processStringParam(const std::string& param_str) 
    {
        std::string str = param_str;
        std::replace(str.begin(), str.end(), '[', ' ');
        std::replace(str.begin(), str.end(), ']', ' ');

        std::vector<T> result;
        std::istringstream iss(str);
        T value;

        while (iss >> value) {
            result.push_back(value);
            if (iss.peek() == ',') {
                iss.ignore();
            }
        }
        return result;
    }

    void callbackMap(const nav_msgs::OccupancyGrid& _map)
    {
        //ROS_INFO_STREAM("Navigator Map received for planning");
        std::call_once(m_map_received_flag,[this,_map]() {this->updateMap(_map);});
    }

    // void callbackMapMetaData(const nav_msgs::MapMetaData& _map_meta_data)
    // {
    //     ROS_INFO_STREAM("Map metadata callback received");
    // }

    void updateMap(const nav_msgs::OccupancyGrid& msg)
    {
        mrpt::maps::COccupancyGridMap2D grid;
	    //ASSERT_(grid.countMapsByClass<mrpt::maps::COccupancyGridMap2D>());
	    mrpt::ros1bridge::fromROS(msg, grid);
        auto obsPts = mrpt::maps::CSimplePointsMap::Create();
        grid.getAsPointCloud(*obsPts);
        ROS_INFO_STREAM("Setting gridmap for planning");
        m_grid_map = obsPts;
    }

    void do_path_plan()
    {
        auto obs =  selfdriving::ObstacleSource::FromStaticPointcloud(m_grid_map);
        selfdriving::PlannerInput planner_input;

        planner_input.stateStart.pose = m_start_pose;
        planner_input.stateStart.vel = m_start_vel;
        planner_input.stateGoal.state = m_nav_goal;
        planner_input.obstacles.emplace_back(obs);
        auto bbox = obs->obstacles()->boundingBox();

        std::cout<<"Map bounding box = "<<bbox.asString()<<std::endl;

    }
};


int main(int argc, char** argv)
{
	TPS_Astar_Nav_Node the_node(argc, argv);
    the_node.do_path_plan();
	ros::spin();
	return 0;
}