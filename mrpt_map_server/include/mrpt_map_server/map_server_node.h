/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <mrpt_msgs/msg/generic_object.hpp>
#include <mrpt_nav_interfaces/msg/georeferencing_metadata.hpp>
#include <mrpt_nav_interfaces/srv/get_gridmap_layer.hpp>
#include <mrpt_nav_interfaces/srv/get_layers.hpp>
#include <mrpt_nav_interfaces/srv/get_pointmap_layer.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class MapServer : public rclcpp::Node
{
   public:
	MapServer();
	~MapServer();
	void init();
	void loop();

   private:
	// params that come from launch file
	std::string pub_mm_topic_ = "map_server";

	std::string service_map_str_ = "static_map";

	std::string frame_id_ = "map";

	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

	/// metric map: will be used whatever is the incoming map format.
	mp2p_icp::metric_map_t theMap_;
	std::mutex theMapMtx_;

	/// (re)publish each map layer, creating the publisher the first time.
	/// If the number of subscriber is detected to have changed, msgs will be
	/// re-published.
	void publish_map();

	// ------ publishers --------

	template <typename msg_t>
	struct PerTopicData
	{
		typename rclcpp::Publisher<msg_t>::SharedPtr pub;

#if 0  // disabled
		size_t subscribers = 0;
		double lastPublishTime = 0;

		bool new_subscribers(
			const rclcpp::Time& now, double forceRepublishPeriodSeconds)
		{
			const auto N = pub->get_subscription_count();
			bool ret = N > subscribers;
			subscribers = N;

			if (forceRepublishPeriodSeconds > 0)
			{
				if (now.seconds() - lastPublishTime >
					forceRepublishPeriodSeconds)
					ret = true;
			}

			if (ret) lastPublishTime = now.seconds();
			return ret;
		}
#endif
	};

	// for the top-level mm metric map:
	PerTopicData<mrpt_msgs::msg::GenericObject> pubMM_;

	PerTopicData<mrpt_msgs::msg::GenericObject> pubGeoRef_;
	PerTopicData<mrpt_nav_interfaces::msg::GeoreferencingMetadata> pubGeoRefMsg_;

	// clang-format off
	// Binary form of each layer:
	std::map<std::string, PerTopicData<mrpt_msgs::msg::GenericObject>> pubLayers_;
	
	// for grid map layers:
	std::map<std::string, PerTopicData<nav_msgs::msg::OccupancyGrid>> pubGridMaps_;
	std::map<std::string, PerTopicData<nav_msgs::msg::MapMetaData>> pubGridMapsMetaData_;
	// for points layers:
	std::map<std::string,PerTopicData<sensor_msgs::msg::PointCloud2>> pubPointMaps_;

	// clang-format on

	sensor_msgs::msg::PointCloud2 pointmap_layer_to_msg(const mrpt::maps::CPointsMap::Ptr& pts);

	// Services:
	rclcpp::Service<mrpt_nav_interfaces::srv::GetLayers>::SharedPtr srvMapLayers_;

	void srv_map_layers(
		const std::shared_ptr<mrpt_nav_interfaces::srv::GetLayers::Request> req,
		std::shared_ptr<mrpt_nav_interfaces::srv::GetLayers::Response> resp);

	rclcpp::Service<mrpt_nav_interfaces::srv::GetGridmapLayer>::SharedPtr srvGetGrid_;

	void srv_get_gridmap(
		const std::shared_ptr<mrpt_nav_interfaces::srv::GetGridmapLayer::Request> req,
		std::shared_ptr<mrpt_nav_interfaces::srv::GetGridmapLayer::Response> resp);

	rclcpp::Service<mrpt_nav_interfaces::srv::GetPointmapLayer>::SharedPtr srvGetPoints_;

	void srv_get_pointmap(
		const std::shared_ptr<mrpt_nav_interfaces::srv::GetPointmapLayer::Request> req,
		std::shared_ptr<mrpt_nav_interfaces::srv::GetPointmapLayer::Response> resp);
};
