#include <tps_astar_nav_node/tps_astar_nav_node.h>


TPS_Astar_Nav_Node::TPS_Astar_Nav_Node(int argc, char** argv):
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
}

template <typename T>
std::vector<T> TPS_Astar_Nav_Node::processStringParam(const std::string& param_str) 
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

void TPS_Astar_Nav_Node::callbackMap(const nav_msgs::OccupancyGrid& _map)
{
    //ROS_INFO_STREAM("Navigator Map received for planning");
    std::call_once(m_map_received_flag,[this,_map]() {this->updateMap(_map);});
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
			mrpt::opengl::COpenGLScene::Ptr ptr_scene = m_win_3d->get3DSceneAndLock();

			ptr_scene->insert(plane);

			ptr_scene->enableFollowCamera(true);

			m_win_3d->unlockAccess3DScene();
		}
	}  // Show 3D?
}

// void TPS_Astar_Nav_Node::callbackMapMetaData(const nav_msgs::MapMetaData& _map_meta_data)
// {
//     ROS_INFO_STREAM("Map metadata callback received");
// }

void TPS_Astar_Nav_Node::updateMap(const nav_msgs::OccupancyGrid& msg)
{
    mrpt::maps::COccupancyGridMap2D grid;
    //ASSERT_(grid.countMapsByClass<mrpt::maps::COccupancyGridMap2D>());
    mrpt::ros1bridge::fromROS(msg, grid);
    auto obsPts = mrpt::maps::CSimplePointsMap::Create();
    grid.getAsPointCloud(*obsPts);
    ROS_INFO_STREAM("*****************************************Setting gridmap for planning");
    m_grid_map = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(obsPts);
    init3DDebug();
    do_path_plan();
}

void TPS_Astar_Nav_Node::do_path_plan()
{
    ROS_INFO_STREAM("************************ Do path planning");
    auto obs = selfdriving::ObstacleSource::FromStaticPointcloud(m_grid_map);
    selfdriving::PlannerInput planner_input;

    planner_input.stateStart.pose = m_start_pose;
    planner_input.stateStart.vel = m_start_vel;
    planner_input.stateGoal.state = m_nav_goal;
    planner_input.obstacles.emplace_back(obs);
    auto bbox = obs->obstacles()->boundingBox();

    {
        const auto bboxMargin = mrpt::math::TPoint3Df(2.0, 2.0, .0);
        const auto ptStart = mrpt::math::TPoint3Df(planner_input.stateStart.pose.x, 
                                                    planner_input.stateStart.pose.y, 0);
        const auto ptGoal = mrpt::math::TPoint3Df(planner_input.stateGoal.asSE2KinState().pose.x,
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
    std::cout << "World bbox : " << planner_input.worldBboxMin.asString() << " - "
              << planner_input.worldBboxMax.asString() << "\n";

    selfdriving::Planner::Ptr planner = selfdriving::TPS_Astar::Create();

    // Enable time profiler:
    planner->profiler_().enable(true);

    {
        // cost map:
        std::string costmap_param_file;

        m_localn.param(
            "global_costmap_parameters", costmap_param_file, costmap_param_file);

        ROS_ASSERT_MSG(
            mrpt::system::fileExists(costmap_param_file),
            "costmap params file not found: '%s'", costmap_param_file.c_str());

        const auto costMapParams =
            selfdriving::CostEvaluatorCostMap::Parameters::FromYAML(
                mrpt::containers::yaml::FromFile(costmap_param_file));

        auto costmap =
            selfdriving::CostEvaluatorCostMap::FromStaticPointObstacles(
                *m_grid_map, costMapParams, planner_input.stateStart.pose);

        ROS_INFO_STREAM("******************************* Costmap file read");

        planner->costEvaluators_.push_back(costmap);
    }

    // Preferred waypoints:
    auto wpParams = selfdriving::CostEvaluatorPreferredWaypoint::Parameters();
    {
        std::string wp_params_file;
        m_localn.param(
            "prefer_waypoints_parameters", wp_params_file, wp_params_file);

        ROS_ASSERT_MSG(
            mrpt::system::fileExists(wp_params_file),
            "Prefer waypoints params file not found: '%s'", wp_params_file.c_str());
        wpParams =
            selfdriving::CostEvaluatorPreferredWaypoint::Parameters::FromYAML(
                mrpt::containers::yaml::FromFile(wp_params_file));
    }

    auto costEval = selfdriving::CostEvaluatorPreferredWaypoint::Create();
    costEval->params_ = wpParams;
    planner->costEvaluators_.push_back(costEval);
    
    {
        std::string planner_parameters_file;
        m_localn.param(
            "planner_parameters", planner_parameters_file, planner_parameters_file);

        ROS_ASSERT_MSG(
            mrpt::system::fileExists(planner_parameters_file),
            "Planner params file not found: '%s'", planner_parameters_file.c_str());
        
        const auto c = mrpt::containers::yaml::FromFile(planner_parameters_file);
        planner->params_from_yaml(c);
        std::cout << "Loaded these planner params:\n";
        planner->params_as_yaml().printAsYAML();
    }

    // Insert custom progress callback:
    planner->progressCallback_ =
    [](const selfdriving::ProgressCallbackData& pcd) {
        std::cout << "[progressCallback] bestCostFromStart: "
                    << pcd.bestCostFromStart
                    << " bestCostToGoal: " << pcd.bestCostToGoal
                    << " bestPathLength: " << pcd.bestPath.size()
                    << std::endl;
    };

    {
        std::string ptg_ini_file;
        m_localn.param(
            "ptg_ini", ptg_ini_file, ptg_ini_file);

        ROS_ASSERT_MSG(
            mrpt::system::fileExists(ptg_ini_file),
            "PTG ini file not found: '%s'", ptg_ini_file.c_str());
        mrpt::config::CConfigFile cfg(ptg_ini_file);
        planner_input.ptgs.initFromConfigFile(cfg, "SelfDriving");

        ROS_INFO_STREAM("******************************* PTG ini");

    }

    const selfdriving::PlannerOutput plan = planner->plan(planner_input);

    std::cout << "\nDone.\n";
    std::cout << "Success: " << (plan.success ? "YES" : "NO") << "\n";
    std::cout << "Plan has " << plan.motionTree.edges_to_children.size()
            << " overall edges, " << plan.motionTree.nodes().size()
            << " nodes\n";

    if (!plan.bestNodeId.has_value())
    {
        std::cerr << "No bestNodeId in plan output.\n";
        return;
    }

    // backtrack:
    auto [plannedPath, pathEdges] =
        plan.motionTree.backtrack_path(*plan.bestNodeId);

    //selfdriving::refine_trajectory(plannedPath, pathEdges, planner_input.ptgs);

    ROS_INFO_STREAM("*******************************Planner Refined");

}



int main(int argc, char** argv)
{
	TPS_Astar_Nav_Node the_node(argc, argv);
    //the_node.do_path_plan();
	ros::spin();
	return 0;
}