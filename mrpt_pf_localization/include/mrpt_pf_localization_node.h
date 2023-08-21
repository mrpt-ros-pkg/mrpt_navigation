/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2023, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/obs/CObservationOdometry.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <cstring>	// size_t
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
#include <std_msgs/msg/header.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "mrpt_pf_localization/mrpt_pf_localization_core.h"

#define MRPT_LOCALIZATION_NODE_DEFAULT_RATE 10.0
#define MRPT_LOCALIZATION_NODE_DEFAULT_PARAMETER_UPDATE_SKIP 1
#define MRPT_LOCALIZATION_NODE_DEFAULT_PARTICLECLOUD_UPDATE_SKIP 5
#define MRPT_LOCALIZATION_NODE_DEFAULT_MAP_UPDATE_SKIP 2

using mrpt::obs::CObservationOdometry;

/// ROS Node
class PFLocalizationNode : public PFLocalization, public rclcpp::Node
{
   public:
	struct NodeParameters
	{
		// static const int MOTION_MODEL_GAUSSIAN = 0;
		// static const int MOTION_MODEL_THRUN = 1;
		NodeParameters();

		void update(const unsigned long& loop_count);

		double rate;

		/// projection into the future added to the published tf to extend its
		/// validity
		double transform_tolerance;

		/// maximum time without updating we keep using filter time instead of
		/// Time::now
		double no_update_tolerance;

		/// maximum time without any observation before start complaining
		double no_inputs_tolerance;

		///
		int parameter_update_skip;

		int particlecloud_update_skip;

		int map_update_skip;

		std::string base_frame_id;

		std::string odom_frame_id;

		std::string global_frame_id;

		bool update_while_stopped;
		bool update_sensor_pose;
		bool pose_broadcast;
		bool tf_broadcast;
		bool use_map_topic;
		bool first_map_only;
	};

	PFLocalizationNode(rclcpp::Node& n);
	virtual ~PFLocalizationNode();
	void init();
	void loop();
	void callbackLaser(const sensor_msgs::msg::LaserScan&);
	void callbackBeacon(const mrpt_msgs::msg::ObservationRangeBeacon&);
	void callbackRobotPose(
		const geometry_msgs::msg::PoseWithCovarianceStamped&);
	void odometryForCallback(
		CObservationOdometry::Ptr&, const std_msgs::msg::Header&);
	void callbackInitialpose(
		const geometry_msgs::msg::PoseWithCovarianceStamped&);
	void callbackOdometry(const nav_msgs::msg::Odometry&);
	void callbackMap(const nav_msgs::msg::OccupancyGrid&);
	void updateMap(const nav_msgs::msg::OccupancyGrid&);
	void publishTF();
	void publishPose();

   private:
	rclcpp::Node nh_;

	bool first_map_received_ = false;

	rclcpp::Time time_last_input_;

	unsigned long long loop_count_ = 0;

	nav_msgs::srv::GetMap::Response resp_;

	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::
		SharedPtr sub_init_pose_;
	// rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_pub_cmd_vel;

	rclcpp::Subscriber sub_odometry_;
	std::vector<rclcpp::Subscriber> sub_sensors_;
	rclcpp::Subscriber sub_map_;
	rclcpp::ServiceClient client_map_;
	rclcpp::Publisher pub_particles_;
	rclcpp::Publisher pub_map_;
	rclcpp::Publisher pub_metadata_;
	rclcpp::Publisher pub_pose_;
	rclcpp::ServiceServer service_map_;

	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_{tf_buffer_};

	tf2_ros::TransformBroadcaster tf_broadcaster_;

	std::map<std::string, mrpt::poses::CPose3D> laser_poses_;
	std::map<std::string, mrpt::poses::CPose3D> beacon_poses_;

	// methods
	void update();
	void updateSensorPose(std::string frame_id);

	void publishParticles();
	void useROSLogLevel();

	bool waitForTransform(
		mrpt::poses::CPose3D& des, const std::string& target_frame,
		const std::string& source_frame, const ros::Time& time,
		const int& timeoutMilliseconds,
		const ros::Duration& polling_sleep_duration = ros::Duration(0.01));
	bool mapCallback(
		nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res);
	void publishMap();
	virtual bool waitForMap();
};
