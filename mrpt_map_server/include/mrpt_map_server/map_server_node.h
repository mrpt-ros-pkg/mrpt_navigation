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
	double frequency_ = 1.0;  //!< rate at which the ros map is published

	double force_republish_period_ = 0;	 //!< [s] (0:disabled)

	// params that come from launch file
	std::string pub_mm_topic_ = "map_server";

	std::string service_map_str_ = "static_map";

	std::string frame_id_ = "map";

#if 0
	rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr
		m_service_map;	//!< service for map server
	nav_msgs::srv::GetMap::Response m_response_ros;	 //!< response from the map server
#endif

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
	};

	// for the top-level mm metric map:
	PerTopicData<mrpt_msgs::msg::GenericObject> pubMM_;

	// clang-format off
	// Binary form of each layer:
	std::map<std::string, PerTopicData<mrpt_msgs::msg::GenericObject>> pubLayers_;
	
	// for grid map layers:
	std::map<std::string, PerTopicData<nav_msgs::msg::OccupancyGrid>> pubGridMaps_;
	std::map<std::string, PerTopicData<nav_msgs::msg::MapMetaData>> pubGridMapsMetaData_;
	// for points layers:
	std::map<std::string,PerTopicData<sensor_msgs::msg::PointCloud2>> pubPointMaps_;

	// clang-format on

	bool map_callback(
		const std::shared_ptr<nav_msgs::srv::GetMap::Request> req,
		const std::shared_ptr<nav_msgs::srv::GetMap::Response> res);
};
