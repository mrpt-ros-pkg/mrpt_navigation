#include <tps_astar_nav_node/tps_astar_nav_node.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>


TPS_Astar_Nav_Node::TPS_Astar_Nav_Node(int argc, char** argv):
                m_auxinit(argc, argv),
                m_nh(),
                m_localn("~"),
                m_nav_goal(mrpt::math::TPose2D(0.0, 0.0, 0.0)),
                m_start_pose(mrpt::math::TPose2D(0.0, 0.0, 0.0)),
                m_start_vel(mrpt::math::TTwist2D(0.0, 0.0, 0.0)),
                m_debug(true),
                m_gui_mrpt(true),
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
    m_localn.param(
        "nav_goal", nav_goal_str, nav_goal_str);
    std::vector<double>goal_pose = processStringParam<double>(nav_goal_str);
    if (goal_pose.size() != 3) 
    {
        ROS_ERROR("Invalid nav_goal parameter.");
        return;
    }
    m_nav_goal = mrpt::math::TPose2D(goal_pose[0], goal_pose[1], goal_pose[2]);
    ROS_INFO_STREAM("[TPS_Astar_Nav_Node]nav goal received ="<< m_nav_goal.asString());

    m_localn.param(
        "start_pose", start_pose_str, start_pose_str);
    std::vector<double>start_pose = processStringParam<double>(start_pose_str);
    if (start_pose.size() != 3) 
    {
        ROS_ERROR("Invalid start pose parameter.");
        return;
    }
    m_start_pose = mrpt::math::TPose2D(start_pose[0], start_pose[1], start_pose[2]);
    ROS_INFO_STREAM("[TPS_Astar_Nav_Node] start pose received ="<< m_start_pose.asString());

    m_localn.param("start_vel", vel_str, vel_str);
    m_start_vel = mrpt::math::TTwist2D(std::stod(vel_str), 0.0, 0.0);
    ROS_INFO_STREAM("[TPS_Astar_Nav_Node]starting velocity ="<< m_start_vel.asString());

    m_localn.param("topic_map_sub", m_sub_map_str, m_sub_map_str);
    m_sub_map = m_nh.subscribe(m_sub_map_str, 1, &TPS_Astar_Nav_Node::callbackMap, this);

    m_localn.param("topic_localization_sub", m_sub_localization_str, m_sub_localization_str);
    m_sub_localization_pose= m_nh.subscribe(m_sub_localization_str, 1, &TPS_Astar_Nav_Node::callbackLocalization, this);

    m_localn.param("topic_odometry_sub", m_sub_odometry_str, m_sub_odometry_str);
    m_sub_odometry = m_nh.subscribe(m_sub_odometry_str, 1, &TPS_Astar_Nav_Node::callbackOdometry, this);

    m_localn.param("topic_obstacles_sub", m_sub_obstacles_str, m_sub_obstacles_str);
    m_sub_obstacles = m_nh.subscribe(m_sub_obstacles_str, 1, &TPS_Astar_Nav_Node::callbackObstacles, this);

    m_localn.param("topic_cmd_vel_pub", m_pub_cmd_vel_str, m_pub_cmd_vel_str);
    m_pub_cmd_vel = m_nh.advertise<geometry_msgs::Twist>(m_pub_cmd_vel_str, 1);

    m_localn.param("topic_wp_seq_pub", m_pub_wp_seq_str, m_pub_wp_seq_str);
    m_pub_wp_seq = m_nh.advertise<mrpt_msgs::WaypointSequence>(m_pub_wp_seq_str,1);

    // Init timers:
    m_timer_run_nav = m_nh.createTimer(ros::Duration(m_nav_period), &TPS_Astar_Nav_Node::onDoNavigation, this);

    //Odometry publisher runs at 50Hz, so this functions runs at the same periodicity
    m_timer_enqueue = m_nh.createTimer(ros::Duration(m_enq_cmd_check_time), &TPS_Astar_Nav_Node::checkEnqueuedMotionCmds, this);

    if(m_nav_engine)
    {
        ROS_INFO_STREAM("TPS Astart Navigator already initialized, resetting nav engine");
        m_nav_engine.reset();
    }

    m_nav_engine = std::make_shared<selfdriving::TPS_Navigator>();

    m_jackal_robot = std::make_shared<Jackal_Interface>(*this);

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

void TPS_Astar_Nav_Node::callbackLocalization(const geometry_msgs::PoseWithCovarianceStamped& _pose)
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

void TPS_Astar_Nav_Node::publish_cmd_vel(const geometry_msgs::Twist& cmd_vel)
{
    ROS_INFO_STREAM("Publishing velocity command"<<cmd_vel);
    m_pub_cmd_vel.publish(cmd_vel);
}

void TPS_Astar_Nav_Node::publish_waypoint_sequence(const mrpt_msgs::WaypointSequence& wps)
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
			mrpt::opengl::COpenGLScene::Ptr ptr_scene = m_win_3d->get3DSceneAndLock();

			ptr_scene->insert(plane);

			ptr_scene->enableFollowCamera(true);

			m_win_3d->unlockAccess3DScene();
		}
	}  // Show 3D?
}

void TPS_Astar_Nav_Node::updateLocalization(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    tf2::Quaternion quat(msg.pose.pose.orientation.x, 
                         msg.pose.pose.orientation.y, 
                         msg.pose.pose.orientation.z, 
                         msg.pose.pose.orientation.w);
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
    //ROS_INFO_STREAM("Localization update complete");
}

void TPS_Astar_Nav_Node::updateOdom(const nav_msgs::Odometry& msg)
{
    tf2::Quaternion quat(msg.pose.pose.orientation.x, 
                         msg.pose.pose.orientation.y, 
                         msg.pose.pose.orientation.z, 
                         msg.pose.pose.orientation.w);
    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    std::lock_guard<std::mutex> csl(m_odometry_cs);
    m_odometry.odometry.x = msg.pose.pose.position.x;
    m_odometry.odometry.y = msg.pose.pose.position.y;
    m_odometry.odometry.phi = yaw;

    m_odometry.odometryVelocityLocal.vx= msg.twist.twist.linear.x;
    m_odometry.odometryVelocityLocal.vy= msg.twist.twist.linear.y;
    m_odometry.odometryVelocityLocal.omega= msg.twist.twist.angular.z;
    
    m_odometry.valid = true;
    m_odometry.timestamp = mrpt::system::now();
    /*TODO*/
    m_odometry.pendedActionExists = m_jackal_robot->enqeued_motion_pending();
    //ROS_INFO_STREAM("Odometry update complete");       
}

void TPS_Astar_Nav_Node::updateObstacles(const sensor_msgs::PointCloud& _pc)
{
    mrpt::maps::CSimplePointsMap point_cloud;
    if(!mrpt::ros1bridge::fromROS(_pc, point_cloud))
    {
        ROS_ERROR("Failed to convert Point Cloud to MRPT Points Map");
    }
    std::lock_guard<std::mutex> csl(m_obstacles_cs);
    m_obstacle_src = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(
                        std::make_shared<mrpt::maps::CSimplePointsMap>(point_cloud));
    
    //ROS_INFO_STREAM("Obstacles update complete"); 
}


void TPS_Astar_Nav_Node::updateMap(const nav_msgs::OccupancyGrid& msg)
{
    mrpt::maps::COccupancyGridMap2D grid;
    //ASSERT_(grid.countMapsByClass<mrpt::maps::COccupancyGridMap2D>());
    mrpt::ros1bridge::fromROS(msg, grid);
    auto obsPts = mrpt::maps::CSimplePointsMap::Create();
    grid.getAsPointCloud(*obsPts);
    ROS_INFO_STREAM("Setting gridmap for planning");
    m_grid_map = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(obsPts);

    m_path_plan_done = do_path_plan();
}

void TPS_Astar_Nav_Node::initializeNavigator()
{
    if(!m_nav_engine)
    {
        ROS_ERROR("TPS_AStar Not created!");
        return;
    }

    m_nav_engine->setMinLoggingLevel(mrpt::system::VerbosityLevel::LVL_DEBUG);
    m_nav_engine->config_.vehicleMotionInterface = std::dynamic_pointer_cast<selfdriving::VehicleMotionInterface>(
                                                        /*std::make_shared<Jackal_Interface>*/(m_jackal_robot));
    m_nav_engine->config_.vehicleMotionInterface->setMinLoggingLevel(mrpt::system::VerbosityLevel::LVL_DEBUG);
    m_nav_engine->config_.globalMapObstacleSource = selfdriving::ObstacleSource::FromStaticPointcloud(m_grid_map);
    m_nav_engine->config_.localSensedObstacleSource = selfdriving::ObstacleSource::FromStaticPointcloud(m_obstacle_src);

    {
        std::string ptg_ini_file;
        m_localn.param(
            "ptg_ini", ptg_ini_file, ptg_ini_file);

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
            "global_costmap_parameters", costmap_param_file, costmap_param_file);

        ROS_ASSERT_MSG(
            mrpt::system::fileExists(costmap_param_file),
            "costmap params file not found: '%s'", costmap_param_file.c_str());

        
        m_nav_engine->config_.globalCostParameters = 
                    selfdriving::CostEvaluatorCostMap::Parameters::FromYAML(
                            mrpt::containers::yaml::FromFile(costmap_param_file));

        m_nav_engine->config_.localCostParameters = 
                    selfdriving::CostEvaluatorCostMap::Parameters::FromYAML(
                            mrpt::containers::yaml::FromFile(costmap_param_file));
    }

    // Preferred waypoints:
    {
        std::string wp_params_file;
        m_localn.param(
            "prefer_waypoints_parameters", wp_params_file, wp_params_file);

        ROS_ASSERT_MSG(
            mrpt::system::fileExists(wp_params_file),
            "Prefer waypoints params file not found: '%s'", wp_params_file.c_str());
        m_nav_engine->config_.preferWaypointsParameters =
            selfdriving::CostEvaluatorPreferredWaypoint::Parameters::FromYAML(
                mrpt::containers::yaml::FromFile(wp_params_file));
    }
    
    {
        std::string planner_parameters_file;
        m_localn.param(
            "planner_parameters", planner_parameters_file, planner_parameters_file);

        ROS_ASSERT_MSG(
            mrpt::system::fileExists(planner_parameters_file),
            "Planner params file not found: '%s'", planner_parameters_file.c_str());
        
        m_nav_engine->config_.plannerParams = 
                    selfdriving::TPS_Astar_Parameters::FromYAML(
                        mrpt::containers::yaml::FromFile(planner_parameters_file));
    }

    {
        std::string nav_engine_parameters_file;
        m_localn.param(
            "nav_engine_parameters", nav_engine_parameters_file, nav_engine_parameters_file);

        ROS_ASSERT_MSG(
            mrpt::system::fileExists(nav_engine_parameters_file),
            "Planner params file not found: '%s'", nav_engine_parameters_file.c_str());
        
        m_nav_engine->config_.loadFrom(
                        mrpt::containers::yaml::FromFile(nav_engine_parameters_file));
    }

    m_nav_engine->initialize();

    ROS_INFO_STREAM("TPS_Astar Navigator intialized");

    if(m_path_plan_done)
    {
        navigateTo(m_nav_goal);
    }
    m_nav_engine_init = true;

}

void TPS_Astar_Nav_Node::navigateTo(const mrpt::math::TPose2D& target)
{
    // selfdriving::Waypoint waypoint(target.x, target.y, 1.0, false, 0.0, 0.0);

    // ROS_INFO_STREAM("[TPS_Astar_Nav_Node] navigateTo"<<waypoint.getAsText());

    // m_waypts.waypoints.push_back(waypoint);

    m_nav_engine->request_navigation(m_waypts);
}

bool TPS_Astar_Nav_Node::do_path_plan()
{
    ROS_INFO_STREAM("Do path planning");
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

        ROS_INFO_STREAM("PTG ini");

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
        return false;
    }

    if(plan.success)
    {
        m_activePlanOutput = plan;
        m_costEvaluators = planner->costEvaluators_;
    }

    // backtrack:
    auto [plannedPath, pathEdges] =
        plan.motionTree.backtrack_path(*plan.bestNodeId);

    selfdriving::refine_trajectory(plannedPath, pathEdges, planner_input.ptgs);

    m_wps_msg = mrpt_msgs::WaypointSequence();
    if(plan.success)
    {
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
        wp_msg.allow_skip = false;

        m_wps_msg.waypoints.push_back(wp_msg);

    }
    return plan.success;
}


selfdriving::VehicleLocalizationState TPS_Astar_Nav_Node::get_localization_state()
{ 	
    std::lock_guard<std::mutex> csl(m_localization_cs);
    return m_localization_pose;
}
selfdriving::VehicleOdometryState TPS_Astar_Nav_Node::get_odometry_state()
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
    if(m_path_plan_done)
    {
        publish_waypoint_sequence(m_wps_msg);
    }
    
    // if(m_obstacle_src && m_localization_pose.valid)
    // {
    //     std::call_once(m_init_nav_flag,[this]() {this->initializeNavigator();});
    // }

    // if(m_nav_engine_init)
    // {
    //     try
	//     {
	// 	    m_nav_engine->navigation_step();
	//     }
	//     catch (const std::exception& e)
	//     {
	// 	    std::cerr << "[TPS_Astar_Nav_Node] Exception:" << e.what() << std::endl;
	// 	    return;
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
    //     if(std::abs(odo.x - m_motion_trigger_pose.x) < m_motion_trigger_tolerance.x &&
    //        std::abs(odo.y - m_motion_trigger_pose.y) < m_motion_trigger_tolerance.y &&
    //        std::abs(odo.phi - m_motion_trigger_pose.phi) < m_motion_trigger_tolerance.phi)
    //     {
    //         std::lock_guard<std::mutex> csl(m_jackal_robot->m_enqueued_motion_mutex);
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
    //         ROS_INFO_STREAM("[TPS_Astar_Nav_Node] Enqueued motion timer = "<<m_enq_motion_timer);
    //         m_enq_motion_timer += m_enq_cmd_check_time;
    //         if(m_jackal_robot->enqeued_motion_pending() &&
    //           m_enq_motion_timer > m_motion_trigger_timeout)
    //         {
    //             std::lock_guard<std::mutex> csl(m_jackal_robot->m_enqueued_motion_mutex);
    //             on_enqueued_motion_timeout();
    //             m_enq_motion_timer = 0.0;
    //         }
    //     }
    // }
}


int main(int argc, char** argv)
{
	TPS_Astar_Nav_Node the_node(argc, argv);
    //the_node.do_path_plan();
	ros::spin();
	return 0;
}