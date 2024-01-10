/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt_pf_localization/mrpt_pf_localization_core.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <cstring>	// size_t
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mrpt_msgs/msg/observation_range_beacon.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define MRPT_LOCALIZATION_NODE_DEFAULT_PARAMETER_UPDATE_SKIP 1
#define MRPT_LOCALIZATION_NODE_DEFAULT_PARTICLECLOUD_UPDATE_SKIP 5
#define MRPT_LOCALIZATION_NODE_DEFAULT_MAP_UPDATE_SKIP 2

using mrpt::obs::CObservationOdometry;

/// ROS Node
class PFLocalizationNode : public rclcpp::Node
{
   public:
	PFLocalizationNode(
		const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
	~PFLocalizationNode();

	struct NodeParameters
	{
		NodeParameters() = default;
		~NodeParameters() = default;

		double rate_hz = 10.0;	//!< Execution rate in Hz

		/// projection into the future added to the published tf to extend its
		/// validity
		double transform_tolerance = 0.1;

		/// maximum time without updating we keep using filter time instead of
		/// Time::now
		double no_update_tolerance = 1.0;

		/// maximum time without any observation before start complaining
		double no_inputs_tolerance;

		///
		int parameter_update_skip;

		int particlecloud_update_skip;

		int map_update_skip;

		std::string base_footprint_frame_id = "base_footprint";
		std::string odom_frame_id = "odom";
		std::string global_frame_id = "map";

		std::string topic_initialpose = "/initialpose";
		std::string topic_gridmap = "/map";

		std::string topic_odometry = "/odom";

		/// Comma "," separated list of topics to subscribe for LaserScan msgs
		std::string topic_sensors_2d_scan;

		/// Comma "," separated list of topics to subscribe for PointCloud2 msgs
		std::string topic_sensors_point_clouds;

		bool update_while_stopped;
		bool update_sensor_pose;
		bool pose_broadcast;
		bool tf_broadcast;
		bool use_map_topic;
		bool first_map_only;
	};

	NodeParameters nodeParams_;

   private:
	PFLocalizationCore core_;

	rclcpp::TimerBase::SharedPtr timer_;

	///
	void reload_params_from_ros();

	void loop();
	void callbackLaser(
		const sensor_msgs::msg::LaserScan& msg, const std::string& topicName);
	void callbackPointCloud(
		const sensor_msgs::msg::PointCloud2& msg, const std::string& topicName);

	void callbackBeacon(const mrpt_msgs::msg::ObservationRangeBeacon&);
	void callbackRobotPose(
		const geometry_msgs::msg::PoseWithCovarianceStamped&);
	void odometryForCallback(
		CObservationOdometry::Ptr&, const std_msgs::msg::Header&);
	void callbackInitialpose(
		const geometry_msgs::msg::PoseWithCovarianceStamped& msg);
	void callbackOdometry(const nav_msgs::msg::Odometry&);
	void callbackMap(const nav_msgs::msg::OccupancyGrid&);
	void updateMap(const nav_msgs::msg::OccupancyGrid&);
	void publishTF();
	void publishPose();

	// nav_msgs::srv::GetMap::Response resp_;

	/** Sub for /initialpose */
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::
		SharedPtr sub_init_pose_;
	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_gridmap_;

	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;

	// Sensors:
	std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr>
		subs_2dlaser_;
	std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>
		subs_point_clouds_;

	// rclcpp::Subscriber sub_map_;
	// rclcpp::ServiceClient client_map_;

	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_particles_;

	// Pubs for ROS standard occupancy grid map:
	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_;
	rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr pub_metadata_;

	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
		pub_pose_;

	// rclcpp::ServiceServer service_map_;

	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

	std::map<std::string, mrpt::poses::CPose3D> laser_poses_;
	std::map<std::string, mrpt::poses::CPose3D> beacon_poses_;

	// methods
	void update();
	void updateSensorPose(std::string frame_id);

	void publishParticles();
	void useROSLogLevel();

	bool waitForTransform(
		mrpt::poses::CPose3D& des, const std::string& target_frame,
		const std::string& source_frame, const int timeoutMilliseconds = 50);
	bool mapCallback(
		nav_msgs::srv::GetMap::Request& req,
		nav_msgs::srv::GetMap::Response& res);
	void publishMap();
};
