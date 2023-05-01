#include <ros/ros.h> 
#include <mutex>
#include "mrpt/system/CTimeLogger.h"
#include "mrpt/math/TPose2D.h"

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
    mrpt::math::TPose2D m_nav_goal;


    public:
    TPS_Astar_Nav_Node(int argc, char** argv):
                m_auxinit(argc, argv),
                m_nh(),
                m_localn("~")
    {
        std::vector<double> goal_pose;
        std::vector<double> default_goal_pose = {0.0, 0.0, 0.0};
        m_localn.param(
			"nav_goal", goal_pose, default_goal_pose);

        if (goal_pose.size() != 3) 
        {
            ROS_ERROR("Invalid nav_goal parameter.");
            return;
        }
        std::cout<<"Goal"<<goal_pose[0]<<"\t"<<goal_pose[1]<<"\t"<<goal_pose[2]<<std::endl;
        m_nav_goal = mrpt::math::TPose2D(goal_pose[0], goal_pose[1], goal_pose[2]);

        std::cout<<"nav goal received"<< m_nav_goal.asString()<<std::endl;

    }


};


int main(int argc, char** argv)
{
	TPS_Astar_Nav_Node the_node(argc, argv);
	ros::spin();
	return 0;
}