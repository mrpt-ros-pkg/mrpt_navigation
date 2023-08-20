/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2023, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/config/CConfigFile.h>
#include <mrpt/maps/CMultiMetricMap.h>

#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "rclcpp/rclcpp.hpp"

class MapServer : public rclcpp::Node
{
   public:
	MapServer();
	~MapServer();
	void init();
	void loop();

   private:
	// member variables
	double m_frequency{0};	//!< rate at which the ros map is published
	bool m_debug{true};	 //!< boolean flag for debugging
	// params that come from launch file
	std::string m_pub_metadata_str;	 //!< param name for map metadata publisher
	std::string m_pub_map_ros_str;	//!< param name for the map publisher
	std::string m_service_map_str;	//!< param name for the map service
	// publishers and services
	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
		m_pub_map_ros;	//!< publisher for map in ros format
	rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr
		m_pub_metadata;	 //!< publisher for map metadata
	rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr
		m_service_map;	//!< service for map server
	nav_msgs::srv::GetMap::Response
		m_response_ros;	 //!< response from the map server

	mrpt::maps::CMultiMetricMap::Ptr
		m_metric_map;  //!< DS to hold the map in MRPT world

	void publish_map();

	bool map_callback(
		const std::shared_ptr<nav_msgs::srv::GetMap::Request> req,
		const std::shared_ptr<nav_msgs::srv::GetMap::Response> res);
};