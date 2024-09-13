/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/TTwist3D.h>
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
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "mrpt_msgs/msg/generic_object.hpp"

#define MRPT_LOCALIZATION_NODE_DEFAULT_PARAMETER_UPDATE_SKIP 1
#define MRPT_LOCALIZATION_NODE_DEFAULT_PARTICLECLOUD_UPDATE_SKIP 5
#define MRPT_LOCALIZATION_NODE_DEFAULT_MAP_UPDATE_SKIP 2

using mrpt::obs::CObservationOdometry;

/// ROS Node
class PFLocalizationNode : public rclcpp::Node
{
   public:
	PFLocalizationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
	~PFLocalizationNode();

	struct NodeParameters
	{
		NodeParameters() = default;
		~NodeParameters() = default;

		void loadFrom(const mrpt::containers::yaml& cfg);

		double rate_hz = 2.0;  //!< Execution rate in Hz

		/// projection into the future added to the published tf to extend its
		/// validity. /tf will be re-published with half this period to ensure
		/// that it is always valid in the /tf tree.
		double transform_tolerance = 0.1;

		/// maximum time without updating we keep using filter time instead of
		/// Time::now
		double no_update_tolerance = 1.0;

		/// maximum time without any observation before start complaining
		double no_inputs_tolerance = 2.0;

		std::string base_link_frame_id = "base_link";
		std::string odom_frame_id = "odom";
		std::string global_frame_id = "map";

		std::string topic_map = "/mrpt_map/metric_map";
		std::string topic_initialpose = "/initialpose";
		std::string topic_odometry = "/odom";

		std::string pub_topic_particles = "/particlecloud";
		std::string pub_topic_pose = "/pf_pose";

		/// Comma "," separated list of topics to subscribe for LaserScan msgs
		std::string topic_sensors_2d_scan;

		/// Comma "," separated list of topics to subscribe for PointCloud2 msgs
		std::string topic_sensors_point_clouds;

		/// Topic name to subscribe for GNSS msgs:
		std::string topic_gnss = "/gps";
	};

	NodeParameters nodeParams_;

   private:
	PFLocalizationCore core_;

	int loopCount_ = 0;	 //!< For decimation purposes only

	bool isTimeFor(int decimation) const
	{
		return decimation <= 1 || (loopCount_ % decimation == 0);
	}

	rclcpp::TimerBase::SharedPtr timer_, timerPubTF_;

	///
	void reload_params_from_ros();

	void loop();
	void callbackLaser(const sensor_msgs::msg::LaserScan& msg, const std::string& topicName);
	void callbackPointCloud(const sensor_msgs::msg::PointCloud2& msg, const std::string& topicName);

	void callbackGNSS(const sensor_msgs::msg::NavSatFix& msg);

	void callbackBeacon(const mrpt_msgs::msg::ObservationRangeBeacon&);
	void callbackRobotPose(const geometry_msgs::msg::PoseWithCovarianceStamped&);
	void callbackInitialpose(const geometry_msgs::msg::PoseWithCovarianceStamped& msg);
	void callbackOdometry(const nav_msgs::msg::Odometry&);

	void callbackMap(const mrpt_msgs::msg::GenericObject& obj);

	/// Publish the PF output mean to /tf
	void publishTF();
	/// Publish the PF output as a PoseArray & PoseWithCovarianceStamped msg
	void publishParticlesAndStampedPose();

	void updateEstimatedTwist();
	void createOdometryFromTwist();

	// These two are used in updateEstimatedTwist()
	std::optional<mrpt::poses::CPose3DPDFParticles> prevParts_;
	std::optional<mrpt::Clock::time_point> prevStamp_;
	std::optional<mrpt::math::TTwist3D> estimated_twist_;

	/** Sub for /initialpose */
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_init_pose_;

	rclcpp::Subscription<mrpt_msgs::msg::GenericObject>::SharedPtr subMap_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry_;

	// Sensors:
	std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> subs_2dlaser_;
	std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subs_point_clouds_;
	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subGNSS_;

	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pubParticles_;

	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pubPose_;

	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

	std::optional<mrpt::Clock::time_point> last_sensor_stamp_;

	void useROSLogLevel();

	[[nodiscard]] bool waitForTransform(
		mrpt::poses::CPose3D& des, const std::string& target_frame, const std::string& source_frame,
		const int timeoutMilliseconds = 50);

	void update_tf_pub_data();
	std::optional<geometry_msgs::msg::TransformStamped> tfMapOdomToPublish_;
	std::mutex tfMapOdomToPublishMtx_;
};
