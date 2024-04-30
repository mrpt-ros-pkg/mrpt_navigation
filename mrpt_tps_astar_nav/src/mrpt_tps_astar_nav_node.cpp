#include <mrpt_tps_astar_nav/mrpt_tps_astar_nav_node.hpp>
#include <cassert>

TPS_Astar_Nav_Node::TPS_Astar_Nav_Node() : rclcpp::Node(NODE_NAME)
{
	const auto qos = rclcpp::SystemDefaultsQoS();

	read_parameters();

	// Init ROS subs:
	// -----------------------
	sub_wp_seq_ = this->create_subscription<mrpt_msgs::msg::WaypointSequence>(
		topic_wp_seq_sub_, 1,
		[this](const mrpt_msgs::msg::WaypointSequence::SharedPtr msg) {
			this->callback_waypoint_seq(msg);
		}
	); 
	
	sub_localization_pose_ = 
		this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
			topic_localization_sub_, 1,
			[this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) 
			{
				this->update_localization(msg);
			}
	); 

	sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
		topic_odometry_sub_, 1,
		[this](const nav_msgs::msg::Odometry::SharedPtr msg) {
			this->update_odometry(msg);
		}
	);

	sub_obstacles_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
		topic_obstacles_sub_, 1, 
		[this](const sensor_msgs::msg::PointCloud2::SharedPtr msg){
			this->callback_obstacles(msg);
		}
	);

	// Init ROS publishers:
	// -----------------------
	pub_cmd_vel_ = 
		this->create_publisher<geometry_msgs::msg::Twist>(topic_cmd_vel_pub_, 1);

    // need publisher to say replan to planner or a service

	// Init timers:
	timer_run_nav_ = this->create_wall_timer(
		std::chrono::microseconds(mrpt::round(1e6 * nav_period_)),
		std::bind(&TPS_Astar_Nav_Node::on_do_navigation, this));

	// Odometry publisher runs at 50Hz, so this functions runs at the same
	// periodicity
	// timer_enqueue_ = this->create_wall_timer(
	// 	std::chrono::microseconds(mrpt::round(1e6 * enq_cmd_check_time_)),
	// 	std::bind(&TPS_Astar_Nav_Node::timer_callback, this));

	if (nav_engine_)
	{
		RCLCPP_INFO_STREAM(
			this->get_logger(),
			"TPS Astart Navigator already initialized, resetting nav "
			"engine");
		nav_engine_.reset();
	}

	nav_engine_ = std::make_shared<mpp::NavEngine>();

	robot_ = std::make_shared<RobotInterface>(*this);
}


void TPS_Astar_Nav_Node::read_parameters()
{
    std::vector<double> nav_goal;
	std::vector<double> start_pose;
	std::vector<double> start_vel;

	this->declare_parameter<std::vector<double>>("nav_goal", {0.0, 0.0, 0.0});
	this->get_parameter("nav_goal", nav_goal);
	if (nav_goal.size() != 3)
	{
		RCLCPP_ERROR(this->get_logger(), "Invalid nav_goal parameter");
		return;
	}
	nav_goal_ =
		mrpt::math::TPose2D(nav_goal[0], nav_goal[1], nav_goal[2]);

	RCLCPP_INFO_STREAM(
		this->get_logger(),
		"[TPS_Astar_Nav_Node] nav goal =" << nav_goal_.asString());
	

	this->declare_parameter<std::vector<double>>("start_pose", {0.0, 0.0, 0.0});
	this->get_parameter("start_pose", start_pose);
	if (start_pose.size() != 3)
	{
		RCLCPP_ERROR(this->get_logger(), "Invalid start pose parameter");
		return;
	}
	start_pose_ =
		mrpt::math::TPose2D(start_pose[0], start_pose[1], start_pose[2]);
	RCLCPP_INFO_STREAM(
		this->get_logger(), "[TPS_Astar_Planner_Node] start pose ="
								<< start_pose_.asString());

	this->declare_parameter<std::vector<double>>("start_vel",{0.0, 0.0, 0.0});
	this->get_parameter("start_vel", start_vel);
	if (start_vel.size() != 3)
	{
		RCLCPP_ERROR(this->get_logger(), "Invalid start velocity parameter");
		return;
	}
	start_vel_ = mrpt::math::TTwist2D(start_vel[0], start_vel[1], start_vel[2]);
	RCLCPP_INFO_STREAM(
		this->get_logger(),
		"[TPS_Astar_Planner_Node] starting velocity =" << start_vel_.asString());

	this->declare_parameter<bool>("mrpt_gui", false);
	this->get_parameter("mrpt_gui", gui_mrpt_);
	RCLCPP_INFO(
		this->get_logger(), "MRPT GUI Enabled: %s", gui_mrpt_ ? "true" : "false");
	
	
	this->declare_parameter<std::string>("topic_wp_seq_sub", "/waypoints");
	this->get_parameter("topic_wp_seq_sub", topic_wp_seq_sub_);
	RCLCPP_INFO(
		this->get_logger(), "topic_wp_seq_sub %s", topic_wp_seq_sub_.c_str());

	this->declare_parameter<std::string>("topic_localization_sub", "/mrpt_pose");
	this->get_parameter("topic_localization_sub", topic_localization_sub_);
	RCLCPP_INFO(
		this->get_logger(), "topic_localization_sub %s", topic_localization_sub_.c_str());

	this->declare_parameter<std::string>("topic_odometry_sub", "odom");
	this->get_parameter("topic_odometry_sub", topic_odometry_sub_);
	RCLCPP_INFO(
		this->get_logger(), "topic_odometry_sub %s", topic_odometry_sub_.c_str());

	this->declare_parameter<std::string>("topic_obstacles_sub", "/map_pointcloud");
	this->get_parameter("topic_obstacles_sub", topic_obstacles_sub_);
	RCLCPP_INFO(
		this->get_logger(), "topic_obstacles_sub %s", topic_obstacles_sub_.c_str());
	
	this->declare_parameter<std::string>("topic_cmd_vel_pub", "/enq_motion");
	this->get_parameter("topic_cmd_vel_pub", topic_cmd_vel_pub_);
	RCLCPP_INFO(
		this->get_logger(), "topic_cmd_vel_pub %s", topic_cmd_vel_pub_.c_str());


    this->declare_parameter<std::string>("ptg_ini", "");
    this->get_parameter("ptg_ini", ptg_ini_file_);
    RCLCPP_INFO(
        this->get_logger(), "ptg_ini_file %s", ptg_ini_file_.c_str());

    assert(mrpt::system::fileExists(ptg_ini_file_) &&
            "PTG ini file not found");

    this->declare_parameter<std::string>("global_costmap_parameters", "");
    this->get_parameter("global_costmap_parameters", costmap_params_file_);
    RCLCPP_INFO(
        this->get_logger(), "global_costmap_params_file %s", 
            costmap_params_file_.c_str());

    assert(mrpt::system::fileExists(costmap_params_file_) && 
            "costmap params file not found");

    this->declare_parameter<std::string>("prefer_waypoints_parameters", "");
    this->get_parameter("prefer_waypoints_parameters", wp_params_file_);
    RCLCPP_INFO(
        this->get_logger(), "prefer_waypoints_parameters_file %s", 
            wp_params_file_.c_str());

    assert(
        mrpt::system::fileExists(wp_params_file_) &&
        "Prefer waypoints params file not found");

    this->declare_parameter<std::string>("planner_parameters", "");
    this->get_parameter("planner_parameters", planner_params_file_);
    RCLCPP_INFO(
        this->get_logger(), "planner_parameters_file %s", 
            planner_params_file_.c_str());

    assert(
        mrpt::system::fileExists(planner_params_file_) &&
        "Planner params file not found");

    
    this->declare_parameter<std::string>("nav_engine_parameters", "");
    this->get_parameter("nav_engine_parameters", nav_engine_params_file_);
    RCLCPP_INFO(
        this->get_logger(), "nav_engine_parameters_file %s", 
            nav_engine_params_file_.c_str());

    assert(
        mrpt::system::fileExists(nav_engine_params_file_) &&
        "Planner params file not found");

}

void TPS_Astar_Nav_Node::callback_obstacles(
	const sensor_msgs::msg::PointCloud2::SharedPtr& _pc)
{
	update_obstacles(_pc);
}

void TPS_Astar_Nav_Node::publish_cmd_vel(
	const geometry_msgs::msg::Twist& cmd_vel)
{
	// RCLCPP_DEBUG_STREAM(
	// 	this->get_logger(), "Publishing velocity command" << cmd_vel);
	pub_cmd_vel_->publish(cmd_vel);
}

void TPS_Astar_Nav_Node::callback_waypoint_seq
    (const mrpt_msgs::msg::WaypointSequence::SharedPtr& _wps)
{

}

void TPS_Astar_Nav_Node::update_localization(
	const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& msg)
{
	tf2::Quaternion quat(
		msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	tf2::Matrix3x3 mat(quat);
	double roll, pitch, yaw;
	mat.getRPY(roll, pitch, yaw);
	std::lock_guard<std::mutex> csl(localization_cs_);
	localization_pose_.frame_id = msg->header.frame_id;
	localization_pose_.valid = true;
	localization_pose_.pose.x = msg->pose.pose.position.x;
	localization_pose_.pose.y = msg->pose.pose.position.y;
	localization_pose_.pose.phi = yaw;
	localization_pose_.timestamp =
		mrpt::ros2bridge::fromROS(msg->header.stamp);
	RCLCPP_DEBUG_STREAM(
		this->get_logger(), "Localization update complete");
}

void TPS_Astar_Nav_Node::update_odometry(const nav_msgs::msg::Odometry::SharedPtr& msg)
{
	tf2::Quaternion quat(
		msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	tf2::Matrix3x3 mat(quat);
	double roll, pitch, yaw;
	mat.getRPY(roll, pitch, yaw);
	std::lock_guard<std::mutex> csl(odometry_cs_);
	odometry_.odometry.x = msg->pose.pose.position.x;
	odometry_.odometry.y = msg->pose.pose.position.y;
	odometry_.odometry.phi = yaw;

	odometry_.odometryVelocityLocal.vx = msg->twist.twist.linear.x;
	odometry_.odometryVelocityLocal.vy = msg->twist.twist.linear.y;
	odometry_.odometryVelocityLocal.omega = msg->twist.twist.angular.z;

	odometry_.valid = true;
	odometry_.timestamp = mrpt::Clock::now();
	/*TODO*/
	odometry_.pendedActionExists = robot_->enqeued_motion_pending();
	RCLCPP_DEBUG_STREAM(this->get_logger(), "Odometry update complete");
}

void TPS_Astar_Nav_Node::update_obstacles
        (const sensor_msgs::msg::PointCloud2::SharedPtr& _pc)
{
	mrpt::maps::CSimplePointsMap point_cloud;
	if (!mrpt::ros2bridge::fromROS(*_pc, point_cloud))
	{
		RCLCPP_ERROR(
			this->get_logger(), "Failed to convert Point Cloud to MRPT Points Map");
	}
	std::lock_guard<std::mutex> csl(obstacles_cs_);
	obstacle_src_ = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(
		std::make_shared<mrpt::maps::CSimplePointsMap>(point_cloud));

	RCLCPP_DEBUG_STREAM(this->get_logger(), "Obstacles update complete");
}

void TPS_Astar_Nav_Node::initialize_navigator()
{
	if (!nav_engine_)
	{
		RCLCPP_ERROR(this->get_logger(), "TPS_AStar Not created!");
		return;
	}

	nav_engine_->setMinLoggingLevel(
		mrpt::system::VerbosityLevel::LVL_DEBUG);
	nav_engine_->config_.vehicleMotionInterface =
		std::dynamic_pointer_cast<mpp::VehicleMotionInterface>(
			/*std::make_shared<RobotInterface>*/ (robot_));
	nav_engine_->config_.vehicleMotionInterface->setMinLoggingLevel(
		mrpt::system::VerbosityLevel::LVL_DEBUG);
	nav_engine_->config_.globalMapObstacleSource =
		mpp::ObstacleSource::FromStaticPointcloud(grid_map_);
	nav_engine_->config_.localSensedObstacleSource =
		mpp::ObstacleSource::FromStaticPointcloud(obstacle_src_);

    mrpt::config::CConfigFile cfg(ptg_ini_file_);
    nav_engine_->config_.ptgs.initFromConfigFile(cfg, "SelfDriving");

    nav_engine_->config_.globalCostParameters =
        mpp::CostEvaluatorCostMap::Parameters::FromYAML(
            mrpt::containers::yaml::FromFile(costmap_params_file_));

    nav_engine_->config_.localCostParameters =
        mpp::CostEvaluatorCostMap::Parameters::FromYAML(
            mrpt::containers::yaml::FromFile(costmap_params_file_));

	// Preferred waypoints:
    nav_engine_->config_.preferWaypointsParameters =
        mpp::CostEvaluatorPreferredWaypoint::Parameters::FromYAML(
            mrpt::containers::yaml::FromFile(wp_params_file_));

    nav_engine_->config_.plannerParams =
        mpp::TPS_Astar_Parameters::FromYAML(
            mrpt::containers::yaml::FromFile(planner_params_file_));
    

	nav_engine_->config_.loadFrom(
		mrpt::containers::yaml::FromFile(nav_engine_params_file_));

	nav_engine_->initialize();

	RCLCPP_INFO_STREAM(
		this->get_logger(), "TPS_Astar Navigator intialized");

    // Make change such that if waypoint sequence is present then do navigate to
	if (true)
	{
		navigate_to(nav_goal_);
	}
	nav_engine_init_ = true;

}

// Not flushed out yet
void TPS_Astar_Nav_Node::navigate_to(const mrpt::math::TPose2D& target)
{
	// mpp::Waypoint waypoint(target.x, target.y, 1.0, false, 0.0, 0.0);

	// RCLCPP_INFO_STREAM(this->get_logger(), "[TPS_Astar_Planner_Node]
	// navigateTo"<<waypoint.getAsText());

	// waypts_.waypoints.push_back(waypoint);

	// nav_engine_->request_navigation(waypts_);
}


void TPS_Astar_Nav_Node::on_do_navigation()
{
	if(obstacle_src_ && localization_pose_.valid)
	{
	    std::call_once(init_nav_flag_,
        [this]() {
            this->initialize_navigator();
        }
        );
	}

	if(nav_engine_init_)
	{
	    try
	    {
		    nav_engine_->navigation_step();
	    }
	    catch (const std::exception& e)
	    {
		    std::cerr << "[TPS_Astar_Nav_Node] Exception:" << e.what() << std::endl;
            return;
	    }

	}
}

void TPS_Astar_Nav_Node::check_enqueued_motion_cmds()
{
	if(odometry_.valid) // && jackal_robot_->enqeued_motion_pending())
	{
	    auto& odo = odometry_.odometry;
	    RCLCPP_INFO_STREAM(this->get_logger(), "Odo: "<< odo.asString());
	    auto& pose = localization_pose_.pose;
	    RCLCPP_INFO_STREAM(this->get_logger(), "Pose: "<< pose.asString()); 
        RCLCPP_INFO_STREAM(this->get_logger(), "Trigger Pose "<< motion_trigger_pose_.asString()); 
        if(std::abs(odo.x - motion_trigger_pose_.x) < motion_trigger_tolerance_.x &&
	       std::abs(odo.y - motion_trigger_pose_.y) < motion_trigger_tolerance_.y && 
           std::abs(odo.phi - motion_trigger_pose_.phi) < motion_trigger_tolerance_.phi)
	    {
	        std::lock_guard<std::mutex>
	        csl(robot_->enqueued_motion_mutex_);
	        RCLCPP_INFO_STREAM(this->get_logger(), "Enqueued motion fired"); on_enqueued_motion_fired();
	        robot_->enqueued_motion_trigger_odom_ = odometry_;
	        RCLCPP_INFO_STREAM(this->get_logger(), "calling change speeds upon pend action fired");
	        //robot_->changeSpeeds(*next_cmd_);
	        RCLCPP_INFO_STREAM(this->get_logger(), "Change speeds complete"); 
            NOP_cmd_ = next_cmd_;
	        RCLCPP_INFO_STREAM(this->get_logger(), "Next NOP command set");
	    }
	    else
	    {
	        RCLCPP_INFO_STREAM(
                this->get_logger(), "[TPS_Astar_Nav_Node] Enqueued motion timer = "
                <<enq_motion_timer_); 
            enq_motion_timer_ += enq_cmd_check_time_;
	        if(robot_->enqeued_motion_pending() &&
	          enq_motion_timer_ > motion_trigger_timeout_)
	        {
	            std::lock_guard<std::mutex> csl(robot_->enqueued_motion_mutex_);
	            on_enqueued_motion_timeout();
	            enq_motion_timer_ = 0.0;
	        }
	    }
	}
}

mpp::VehicleLocalizationState 
        TPS_Astar_Nav_Node::get_localization_state()
{
    std::lock_guard<std::mutex> csl(localization_cs_);
	return localization_pose_;
}

mpp::VehicleOdometryState 
        TPS_Astar_Nav_Node::get_odometry_state()
{
    std::lock_guard<std::mutex> csl(odometry_cs_);
		return odometry_;
}

mrpt::maps::CPointsMap::Ptr 
        TPS_Astar_Nav_Node::get_current_obstacles()
{
    std::lock_guard<std::mutex> csl(obstacles_cs_);
	return obstacle_src_;
}


// --------------------------------------

RobotInterface::RobotInterface(TPS_Astar_Nav_Node & parent)
	: parent_(parent),
		enqueued_motion_pending_(false),
		enqueued_motion_timeout_(false)
{
	parent_.on_enqueued_motion_fired = [this]() {
		RCLCPP_INFO_STREAM(
			parent_.get_logger(), "[Jackal Robot] Enqueud motion fired");
		changeSpeeds(*parent_.next_cmd_);
		RCLCPP_INFO_STREAM(
			parent_.get_logger(), "[Jackal Robot] change speeds");
		{
			// std::lock_guard<std::mutex> csl(enqueued_motion_mutex_);
			enqueued_motion_pending_ = false;
			enqueued_motion_timeout_ = false;
		}
		RCLCPP_DEBUG_STREAM(parent_.get_logger(), "Mutex Release");
		return;
	};

	parent_.on_enqueued_motion_timeout = [this]() {
		RCLCPP_DEBUG_STREAM(
			parent_.get_logger(), "[Jackal Robot] Enqueud motion timedout");
		// std::lock_guard<std::mutex> csl(enqueued_motion_mutex_);
		enqueued_motion_pending_ = false;
		enqueued_motion_timeout_ = true;
		return;
	};
}

mpp::VehicleLocalizationState RobotInterface::get_localization()
{
	return parent_.get_localization_state();
}

mpp::VehicleOdometryState RobotInterface::get_odometry()
{
	return parent_.get_odometry_state();
}

bool RobotInterface::motion_execute(
	const std::optional<mrpt::kinematics::CVehicleVelCmd::Ptr>& immediate,
	const std::optional<mpp::EnqueuedMotionCmd>& next)
{
	if (immediate.has_value())
	{
		std::lock_guard<std::mutex> csl(enqueued_motion_mutex_);
		RCLCPP_INFO_STREAM(
			parent_.get_logger(), "[Jackal_Robot] Motion command received");
		mrpt::kinematics::CVehicleVelCmd& vel_cmd = *(immediate.value());
		changeSpeeds(vel_cmd);
		parent_.NOP_cmd_ = immediate.value();
		enqueued_motion_pending_ = false;
		enqueued_motion_timeout_ = false;
	}

	if (next.has_value())
	{
		// RCLCPP_INFO_STREAM(this->get_logger(), "[Jackal_Robot] Enqueued
		// command received");
		std::lock_guard<std::mutex> csl(enqueued_motion_mutex_);
		auto& enq_cmd = next.value();
		enqueued_motion_pending_ = true;
		enqueued_motion_timeout_ = false;
		parent_.motion_trigger_pose_ = enq_cmd.nextCondition.position;
		parent_.motion_trigger_tolerance_ = enq_cmd.nextCondition.tolerance;
		parent_.motion_trigger_timeout_ = enq_cmd.nextCondition.timeout;
		parent_.next_cmd_ = next->nextCmd;
		parent_.enq_motion_timer_ = 0.0;
		RCLCPP_INFO_STREAM(
			parent_.get_logger(),
			"[Jackal_Robot] Next cmd timeout = "
				<< parent_.motion_trigger_timeout_
				<< " Enq_Motion_timer = " << parent_.enq_motion_timer_);
	}

	if (!immediate.has_value() && !next.has_value())
	{
		std::lock_guard<std::mutex> csl(enqueued_motion_mutex_);
		RCLCPP_INFO_STREAM(
			parent_.get_logger(), "[Jackal_Robot] NOP command received");
		if (parent_.NOP_cmd_)
		{
			changeSpeeds(*parent_.NOP_cmd_);
			return true;
		}
	}

	return true;
}

bool RobotInterface::supports_enqeued_motions() const { return true; }

bool RobotInterface::enqeued_motion_pending() const
{
	auto lck = mrpt::lockHelper(enqueued_motion_mutex_);
	// RCLCPP_INFO_STREAM(this->get_logger(), "Enqueued motion command
	// pending
	// ?"<< 				(enqueued_motion_pending_?"True":"False"));
	return enqueued_motion_pending_;
}

bool RobotInterface::enqeued_motion_timed_out() const
{
	auto lck = mrpt::lockHelper(enqueued_motion_mutex_);
	// RCLCPP_INFO_STREAM(this->get_logger(), "Enqueued motion command timed
	// out
	// ?"<< 				(enqueued_motion_timeout_?"True":"False"));
	return enqueued_motion_timeout_;
}

std::optional<mpp::VehicleOdometryState>
	RobotInterface::enqued_motion_last_odom_when_triggered() const
{
	auto lck = mrpt::lockHelper(enqueued_motion_mutex_);
	return enqueued_motion_trigger_odom_;
}

bool RobotInterface::changeSpeeds(
	const mrpt::kinematics::CVehicleVelCmd& vel_cmd)
{
	// RCLCPP_INFO_STREAM(this->get_logger(), "[Jackal_Robot] change
	// speeds");
	using namespace mrpt::kinematics;
	const CVehicleVelCmd_DiffDriven* vel_cmd_diff_driven =
		dynamic_cast<const CVehicleVelCmd_DiffDriven*>(&vel_cmd);
	ASSERT_(vel_cmd_diff_driven);

	const double v = vel_cmd_diff_driven->lin_vel;
	const double w = vel_cmd_diff_driven->ang_vel;
	RCLCPP_INFO_STREAM(
		parent_.get_logger(),
		"changeSpeeds: v= " << v << "m/s and w=" << w * 180.0f / M_PI
							<< "deg/s");
	geometry_msgs::msg::Twist cmd;
	cmd.linear.x = v;
	cmd.angular.z = w;
	parent_.publish_cmd_vel(cmd);
	return true;
}

void RobotInterface::stop(const mpp::STOP_TYPE stopType)
{
	if (stopType == mpp::STOP_TYPE::EMERGENCY)
	{
		const auto cmd = getEmergencyStopCmd();
		enqueued_motion_pending_ = false;
		enqueued_motion_timeout_ = false;
		changeSpeeds(*cmd);
	}
	else
	{
		const auto cmd = getStopCmd();
		enqueued_motion_pending_ = false;
		enqueued_motion_timeout_ = false;
		changeSpeeds(*cmd);
	}
}

mrpt::kinematics::CVehicleVelCmd::Ptr RobotInterface::getStopCmd()
{
	mrpt::kinematics::CVehicleVelCmd::Ptr vel =
		mrpt::kinematics::CVehicleVelCmd::Ptr(
			new mrpt::kinematics::CVehicleVelCmd_DiffDriven);
	vel->setToStop();
	return vel;
}

mrpt::kinematics::CVehicleVelCmd::Ptr RobotInterface::getEmergencyStopCmd()
{
	return getStopCmd();
}

void RobotInterface::stop_watchdog()
{
	RCLCPP_INFO_STREAM(
		parent_.get_logger(), "TPS_Astar_Navigator watchdog timer stopped");
}

void RobotInterface::start_watchdog(const size_t periodMilliseconds)
{
	RCLCPP_INFO_STREAM(
		parent_.get_logger(), "TPS_Astar_Navigator start watchdog timer");
}

void RobotInterface::on_nav_end_due_to_error()
{
	RCLCPP_INFO_STREAM(
		parent_.get_logger(), "[TPS_Astar_Navigator] Nav End due to error");
	stop(mpp::STOP_TYPE::EMERGENCY);
}

void RobotInterface::on_nav_start()
{
	RCLCPP_INFO_STREAM(
		parent_.get_logger(), "[TPS_Astar_Navigator] Nav starting to Goal ="
									<< parent_.nav_goal_.asString());
}

void RobotInterface::on_nav_end()
{
	RCLCPP_INFO_STREAM(
		parent_.get_logger(), "[TPS_Astar_Navigator] Nav End");
	stop(mpp::STOP_TYPE::REGULAR);
}

void RobotInterface::on_path_seems_blocked()
{
	RCLCPP_INFO_STREAM(
		parent_.get_logger(), "[TPS_Astar_Navigator] Path Seems blocked");
	stop(mpp::STOP_TYPE::REGULAR);
}

void RobotInterface::on_apparent_collision()
{
	RCLCPP_INFO_STREAM(
		parent_.get_logger(), "[TPS_Astar_Navigator] Apparent collision");
	// stop(mpp::STOP_TYPE::REGULAR);
}

mrpt::maps::CPointsMap::Ptr RobotInterface::obstacles(
	mrpt::system::TTimeStamp t)
{
	return parent_.get_current_obstacles();
}


// ------------------------------------
int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<TPS_Astar_Nav_Node>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}