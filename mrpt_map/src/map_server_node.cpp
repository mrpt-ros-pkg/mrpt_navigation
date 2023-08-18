/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2023, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include "mrpt_map/map_server_node.hpp"

#include <mrpt/config/CConfigFile.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/ros2bridge/map.h>
#include <mrpt/system/filesystem.h>	 // ASSERT_FILE_EXISTS_()

#include <chrono>
#include <nav_msgs/msg/occupancy_grid.hpp>

using namespace mrpt::config;
using mrpt::maps::CMultiMetricMap;
using mrpt::maps::COccupancyGridMap2D;

MapServer::MapServer() : Node("mrpt_map_server") {}

MapServer::~MapServer() {}

void MapServer::init()
{
	this->declare_parameter<bool>("debug", false);
	this->get_parameter("debug", m_debug);
	RCLCPP_INFO(this->get_logger(), "debug: %s", m_debug ? "true" : "false");

	mrpt::maps::COccupancyGridMap2D::Ptr grid;
	std::string map_yaml_file;
	this->declare_parameter<std::string>("map_yaml_file", "");
	this->get_parameter("map_yaml_file", map_yaml_file);
	RCLCPP_INFO(
		this->get_logger(), "map_yaml_file name: %s", map_yaml_file.c_str());

	if (!map_yaml_file.empty())
	{
		grid = mrpt::maps::COccupancyGridMap2D::Create();
		grid->loadFromROSMapServerYAML(map_yaml_file);
	}
	else
	{
		std::string ini_file;
		std::string map_file;

		this->declare_parameter<std::string>("ini_file", "map.ini");
		this->get_parameter("ini_file", ini_file);
		RCLCPP_INFO(
			this->get_logger(), "map_ini_file name: %s", ini_file.c_str());

		this->declare_parameter<std::string>("map_file", "map.simplemap");
		this->get_parameter("map_file", map_file);
		RCLCPP_INFO(this->get_logger(), "map_file name: %s", map_file.c_str());

		ASSERT_FILE_EXISTS_(ini_file);
		ASSERT_FILE_EXISTS_(map_file);
		CConfigFile config_file;
		config_file.setFileName(ini_file);

		m_metric_map = CMultiMetricMap::Create();

		mrpt::ros2bridge::MapHdl::loadMap(
			*m_metric_map, config_file, map_file, "metricMap", m_debug);

		grid = m_metric_map->mapByClass<COccupancyGridMap2D>();
	}

	ASSERT_(grid);

	this->declare_parameter<std::string>("frame_id", "map");
	this->get_parameter("frame_id", m_response_ros.map.header.frame_id);
	RCLCPP_INFO(
		this->get_logger(), "frame_id: %s",
		m_response_ros.map.header.frame_id.c_str());

	this->declare_parameter<double>("frequency", 0.1);
	this->get_parameter("frequency", m_frequency);
	RCLCPP_INFO(this->get_logger(), "frequency: %f", m_frequency);

	this->declare_parameter<std::string>("pub_map_ros", "map");
	this->get_parameter("pub_map_ros", m_pub_map_ros_str);
	RCLCPP_INFO(
		this->get_logger(), "pub_map_ros: %s", m_pub_map_ros_str.c_str());

	this->declare_parameter<std::string>("pub_metadata", "map_metadata");
	this->get_parameter("pub_metadata", m_pub_metadata_str);
	RCLCPP_INFO(
		this->get_logger(), "pub_metadata: %s", m_pub_metadata_str.c_str());

	this->declare_parameter<std::string>("service_map", "static_map");
	this->get_parameter("service_map", m_service_map_str);
	RCLCPP_INFO(
		this->get_logger(), "service_map: %s", m_service_map_str.c_str());

	m_pub_map_ros = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
		m_pub_map_ros_str, 1);
	m_pub_metadata = this->create_publisher<nav_msgs::msg::MapMetaData>(
		m_pub_metadata_str, 1);
	m_service_map = this->create_service<nav_msgs::srv::GetMap>(
		m_service_map_str,
		[this](
			const nav_msgs::srv::GetMap::Request::SharedPtr req,
			const nav_msgs::srv::GetMap::Response::SharedPtr res) {
			this->map_callback(req, res);
		});

	if (m_debug)
	{
		RCLCPP_INFO(
			this->get_logger(),
			"gridMap[0]:  %i x %i @ %4.3fm/p, %4.3f, %4.3f, %4.3f, %4.3f\n",
			grid->getSizeX(), grid->getSizeY(), grid->getResolution(),
			grid->getXMin(), grid->getYMin(), grid->getXMax(), grid->getYMax());
	}

	mrpt::ros2bridge::toROS(*grid, m_response_ros.map);

	if (m_debug)
	{
		RCLCPP_INFO(
			this->get_logger(),
			"msg:         %i x %i @ %4.3fm/p, %4.3f, %4.3f, %4.3f, %4.3f\n",
			m_response_ros.map.info.width, m_response_ros.map.info.height,
			m_response_ros.map.info.resolution,
			m_response_ros.map.info.origin.position.x,
			m_response_ros.map.info.origin.position.y,
			m_response_ros.map.info.width * m_response_ros.map.info.resolution +
				m_response_ros.map.info.origin.position.x,
			m_response_ros.map.info.height *
					m_response_ros.map.info.resolution +
				m_response_ros.map.info.origin.position.y);
	}
}

bool MapServer::map_callback(
	const std::shared_ptr<nav_msgs::srv::GetMap::Request> req,
	const std::shared_ptr<nav_msgs::srv::GetMap::Response> res)
{
	RCLCPP_INFO(this->get_logger(), "mapCallback: service requested");
	*res = m_response_ros;
	return true;
}

void MapServer::publish_map()
{
	auto now = std::chrono::system_clock::now();

	m_response_ros.map.header.stamp =
		rclcpp::Time(now.time_since_epoch().count(), RCL_ROS_TIME);
	if (m_pub_map_ros->get_subscription_count() > 0)
	{
		m_pub_map_ros->publish(m_response_ros.map);
	}
	if (m_pub_metadata->get_subscription_count() > 0)
	{
		m_pub_metadata->publish(m_response_ros.map.info);
	}
}

void MapServer::loop()
{
	if (m_frequency > 0)
	{
		rclcpp::Rate rate(m_frequency);
		while (rclcpp::ok())
		{
			publish_map();
			rclcpp::spin_some(shared_from_this());
			rate.sleep();
		}
	}
	else
	{
		publish_map();
		rclcpp::spin(shared_from_this());
	}
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<MapServer>();
	node->init();
	node->loop();
	rclcpp::shutdown();
	return 0;
}
