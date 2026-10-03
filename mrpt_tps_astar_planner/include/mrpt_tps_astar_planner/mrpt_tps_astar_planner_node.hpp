/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mpp/algos/CostEvaluatorCostMap.h>
#include <mpp/algos/CostEvaluatorPreferredWaypoint.h>
#include <mpp/algos/CostEvaluatorReverseMotion.h>
#include <mpp/algos/NavEngine.h>
#include <mpp/algos/TPS_Astar.h>
#include <mpp/algos/edge_interpolated_path.h>
#include <mpp/algos/refine_trajectory.h>
#include <mpp/algos/trajectories.h>
#include <mpp/algos/viz.h>
#include <mpp/data/EnqueuedMotionCmd.h>
#include <mpp/data/MotionPrimitivesTree.h>
#include <mpp/data/PlannerOutput.h>
#include <mpp/data/robot_shape_sampling.h>
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
#include <mrpt/poses/CPose3D.h>
#include <mrpt/ros2bridge/map.h>
#include <mrpt/ros2bridge/point_cloud2.h>
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/time.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <chrono>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mrpt_msgs/msg/waypoint.hpp>
#include <mrpt_msgs/msg/waypoint_sequence.hpp>
#include <mrpt_nav_interfaces/srv/make_plan_from_to.hpp>
#include <mrpt_nav_interfaces/srv/make_plan_to.hpp>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <set>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <type_traits>
#include <utility>

namespace mrpt_tps_astar_planner
{
/**
 * The main ROS2 node class.
 */
class TPS_Astar_Planner_Node : public rclcpp::Node
{
   public:
	explicit TPS_Astar_Planner_Node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
	virtual ~TPS_Astar_Planner_Node()
	{
		if (ptgs_init_thread_.joinable())
		{
			ptgs_init_thread_.join();
		}
	}

   private:
	/// CTimeLogger instance for profiling
	mrpt::system::CTimeLogger profiler_;

	/// Subscriber to Goal position
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;

	/// Mutex for gridmaps_ & obstacle_points_
	std::mutex obstacles_cs_;

	/// Subscribers to gridmaps
	struct InfoPerGridMapSource
	{
		rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub;
		mrpt::maps::COccupancyGridMap2D::Ptr grid;
		mrpt::maps::CSimplePointsMap::Ptr grid_obstacles;
	};
	std::deque<InfoPerGridMapSource> gridmaps_;

	/// Subscriber to obstacle points
	struct InfoPerPointMapSource
	{
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
		mrpt::maps::CPointsMap::Ptr obstacle_points;
	};

	std::deque<InfoPerPointMapSource> obstacle_points_;

	/// Publisher for waypoint sequence
	rclcpp::Publisher<mrpt_msgs::msg::WaypointSequence>::SharedPtr pub_wp_seq_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_wp_path_seq_;
	std::vector<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr> pub_costmaps_;

	/// Robot footprint used for planning (latched), so other nodes can
	/// check their configuration is consistent with it:
	rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_robot_shape_;
	std::mutex pub_costmaps_cs_;

	// tf2 buffer and listener
	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	/// Flag for MRPT GUI
	bool gui_mrpt_ = false;

	/// Counter of planning requests shown in the debug GUI window title
	unsigned int gui_plan_request_counter_ = 0;

	/// frame_id for "map"
	std::string frame_id_map_ = "map";

	/// frame_id for the robot
	std::string frame_id_robot_ = "base_link";

	/// goal topic subscriber name
	std::string topic_goal_sub_ = "tps_astar_nav_goal";

	/// map topic subscriber name(s) (multiple if separated by ',')
	std::string topic_gridmap_sub_ = "/map";

	/// obstacles topic subscriber name(s) (multiple if separated by ',')
	std::string topic_obstacle_points_sub_ = "";

	/// topics (from topic_gridmap_sub_, topic_obstacle_points_sub_) that shall
	/// be subscribed with transient QoS (normally, all static maps) (multiple
	/// if separated by ',')
	std::string topic_static_maps_ = "/map";

	/// waypoint sequence topic publisher name
	std::string topic_wp_seq_pub_;

	/// costmaps topic publisher name prefix
	std::string topic_costmaps_pub_ = "/costmap";

	/// Parameter file for PTGs
	std::string ptg_ini_file_ = "ptgs.ini";

	/// Parameters file for Costmap evaluator
	std::string costmap_params_file_ = "global-costmap-params.yaml";

	/// Parameters file for waypoints preferences
	std::string wp_params_file_ = "waypoints-params.yaml";

	/// Parameters file for planner
	std::string planner_params_file_ = "planner-params.yaml";

	float problem_world_bbox_margin_ = 2.0f;
	bool problem_world_bbox_ignore_obstacles_ = false;

	bool astar_skip_refine_ = false;

	/// Extra cost per second of reverse motion (0: reversing costs the same as
	/// driving forward)
	double reverse_motion_cost_factor_ = 1.0;

	/// Waypoint parameters
	double mid_waypoints_allowed_distance_ = 0.5;
	double final_waypoint_allowed_distance_ = 0.4;

	bool mid_waypoints_allow_skip_ = true;
	bool final_waypoint_allow_skip_ = false;

	bool mid_waypoints_ignore_heading_ = false;
	bool final_waypoint_ignore_heading_ = false;

	/// Planner params loaded once at startup, reused to init per-call local planner instances
	mrpt::containers::yaml planner_params_yaml_;

	mpp::TrajectoriesAndRobotShape ptgs_;

	/// Building the PTG lookup tables can take a long time, so it runs in a
	/// background thread to keep the node responsive. Planning requests are
	/// rejected (with a warning) until this flag is set.
	std::atomic<bool> ptgs_ready_{false};
	std::atomic<bool> ptgs_failed_{false};
	std::thread ptgs_init_thread_;

	// ptgs_ holds shared_ptr<ptg_t> entries that are reused (not cloned) by
	// every do_path_plan() call via pi.ptgs = ptgs_. The PTG implementations
	// mutate internal scratch state while evaluating a plan, so with the
	// reentrant callback group + MultiThreadedExecutor below, concurrent
	// service calls can run plan() on the same PTG objects at once and
	// corrupt each other's search. Serialize the actual planning call with
	// this mutex instead of trying to make every PTG implementation
	// thread-safe.
	std::mutex planning_cs_;

	/// Parameters for the cost evaluator
	mpp::CostEvaluatorCostMap::Parameters costMapParams_;

	/// Reentrant callback group so service calls can run concurrently
	rclcpp::CallbackGroup::SharedPtr srv_cbg_;

   private:
	/**
	 * @brief wait for transform between map frame and the robot frame
	 *
	 * @param des position of the robot with respect to map frame
	 * @param target_frame the robot tf frame
	 * @param source_frame the map tf frame
	 * @param timeout_milliseconds timeout for transform wait
	 *
	 * @return true if there is transform from map to the robot
	 */
	[[nodiscard]] bool wait_for_transform(
		mrpt::poses::CPose3D& des, const std::string& target_frame, const std::string& source_frame,
		const int timeout_milliseconds = 50);

	/**
	 * @brief Reads a parameter from the node's parameter server.
	 *
	 * This function attempts to retrieve parameters and assign it to class
	 * member vars.
	 */
	void read_parameters();

	/**
	 * @brief Initialize A* planner with required params
	 */
	void initialize_planner();

	/// Builds the PTGs (slow), then publishes the robot footprint. Run in a background thread.
	void initialize_ptgs_and_publish_shape();

	/// Warns that a request cannot be served yet. Returns true if PTGs are ready.
	[[nodiscard]] bool check_ptgs_ready(const char* requestKind);

	/**
	 * @brief Callback function when a new goal location is received
	 * @param _goal is a PoseStamped object pointer
	 */
	void callback_goal(const geometry_msgs::msg::PoseStamped& goal);

	/**
	 * @brief Callback function when a new map is received
	 * @param _map is an occupancy grid object pointer
	 */
	void callback_map(const nav_msgs::msg::OccupancyGrid::SharedPtr& m, InfoPerGridMapSource& e);

	/**
	 * @brief Callback function to update the obstacles around the Robot in case
	 * of replan
	 * @param _pc pointcloud object pointer from sensors
	 */
	void callback_obstacles(
		const sensor_msgs::msg::PointCloud2::SharedPtr& pc, InfoPerPointMapSource& e);

	/**
	 * @brief Callback function to prompt for a replan
	 * @param _pose current localization location of the robot on the map
	 */
	void callback_replan(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& _pose);

	/**
	 * @brief Mutex locked method to update the map when new one is received
	 * @param _map is an occupancy grid object pointer
	 */
	void update_map(const nav_msgs::msg::OccupancyGrid::SharedPtr& _msg, InfoPerGridMapSource& e);

	/**
	 * @brief Mutex locked method to update local obstacle map
	 * @param _pc PointCloud2 object
	 */
	void update_obstacles(
		const sensor_msgs::msg::PointCloud2::SharedPtr& _pc, InfoPerPointMapSource& e);

	struct PlanResult
	{
		PlanResult() = default;

		bool valid = false;
		mpp::PlannerOutput plan_output;
		mrpt_msgs::msg::WaypointSequence wps{};
	};

	/**
	 * @brief Method to perform the path plan
	 * @param start robot initial pose
	 * @param goal  robot goal pose
	 * @return the plan results
	 */
	PlanResult do_path_plan(const mrpt::math::TPose2D& start, const mrpt::math::TPose2D& goal);

	void srv_make_plan_to(
		const std::shared_ptr<mrpt_nav_interfaces::srv::MakePlanTo::Request> req,
		std::shared_ptr<mrpt_nav_interfaces::srv::MakePlanTo::Response> resp);

	rclcpp::Service<mrpt_nav_interfaces::srv::MakePlanTo>::SharedPtr srvMakePlanTo_;

	void srv_make_plan_from_to(
		const std::shared_ptr<mrpt_nav_interfaces::srv::MakePlanFromTo::Request> req,
		std::shared_ptr<mrpt_nav_interfaces::srv::MakePlanFromTo::Response> resp);

	rclcpp::Service<mrpt_nav_interfaces::srv::MakePlanFromTo>::SharedPtr srvMakePlanFromTo_;

	/**
	 * @brief Returns the planner instance for the calling thread, initializing
	 * it on first use. Each executor thread keeps its own instance so
	 * concurrent service calls never share mutable planner state.
	 */
	mpp::Planner& get_thread_planner();

	/**
	 * @brief Debug method to visualize the planning
	 */
	/**
	 * @brief Publisher method to publish waypoint sequence
	 * @param wps Waypoint sequence object
	 */
	void publish_waypoint_sequence(const mrpt_msgs::msg::WaypointSequence& wps);
};

}  // namespace mrpt_tps_astar_planner
