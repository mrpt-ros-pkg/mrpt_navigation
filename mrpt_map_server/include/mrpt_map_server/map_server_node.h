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

#include "mrpt_msgs/msg/generic_object.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class MapServer : public rclcpp::Node
{
   public:
	MapServer();
	~MapServer();
	void init();
	void loop();

   private:
	// member variables
	double m_frequency = 1.0;  //!< rate at which the ros map is published

	// params that come from launch file
	std::string pub_mm_topic_ = "map_server";

	std::string m_service_map_str = "static_map";

	rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr
		m_service_map;	//!< service for map server
	nav_msgs::srv::GetMap::Response
		m_response_ros;	 //!< response from the map server

	/// metric map: will be used whatever is the incoming map format.
	mp2p_icp::metric_map_t theMap_;

	/// (re)publish each map layer, creating the publisher the first time.
	/// If the number of subscriber is detected to have changed, msgs will be
	/// re-published.
	void publish_map();

	// ------ publishers --------
	// clang-format off
	// for the top-level mm metric map:
	rclcpp::Publisher<mrpt_msgs::msg::GenericObject>::SharedPtr pubMM_;
	size_t pubMM_subscribers_ = 0;

	// for grid map layers:
	std::map<std::string, rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr> pubGridMaps_;
	size_t pubGridMaps_subscribers_ = 0;
	
	std::map<std::string, rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr> pubGridMapsMetaData_;
	size_t pubGridMapsMetaData_subscribers_ = 0;

	// for points layers:
	std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2::SharedPtr>> pubPointMaps_;
	size_t pubPointMaps_subscribers_ = 0;

	// clang-format on

	bool map_callback(
		const std::shared_ptr<nav_msgs::srv::GetMap::Request> req,
		const std::shared_ptr<nav_msgs::srv::GetMap::Response> res);
};
