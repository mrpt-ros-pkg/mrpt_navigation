/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include "mrpt_map_server/map_server_node.h"

#include <mrpt/config/CConfigFile.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/ros2bridge/map.h>
#include <mrpt/serialization/CArchive.h>
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
	// See:
	// https://github.com/mrpt-ros-pkg/mrpt_navigation/blob/ros2/mrpt_map_server/README.md

	// MAP FORMAT 2: "legacy" ROS1 grid maps:
	// -------------------------------------------
	std::string map_yaml_file;
	this->declare_parameter<std::string>("map_yaml_file", "");
	this->get_parameter("map_yaml_file", map_yaml_file);
	RCLCPP_INFO(
		this->get_logger(), "map_yaml_file name: '%s'", map_yaml_file.c_str());

	std::string mrpt_metricmap_file;
	this->declare_parameter<std::string>("mrpt_metricmap_file", "");
	this->get_parameter("mrpt_metricmap_file", mrpt_metricmap_file);
	RCLCPP_INFO(
		this->get_logger(), "mrpt_metricmap_file name: '%s'",
		mrpt_metricmap_file.c_str());

	if (!map_yaml_file.empty())
	{
		auto grid = mrpt::maps::COccupancyGridMap2D::Create();
		grid->loadFromROSMapServerYAML(map_yaml_file);

		// store as the unique map layer named "map":
		theMap_.layers["map"] = grid;
	}
	else if (!mrpt_metricmap_file.empty())
	{
		ASSERT_FILE_EXISTS_(mrpt_metricmap_file);
		mrpt::io::CFileGZInputStream f(mrpt_metricmap_file);

		auto a = mrpt::serialization::archiveFrom(f);

		mrpt::serialization::CSerializable::Ptr obj = a.ReadObject();
		ASSERT_(obj);
		auto map = std::dynamic_pointer_cast<mrpt::maps::CMetricMap>(obj);
		ASSERTMSG_(map, "Object read from input stream is not a CMetricMap");

		// store as the unique map layer named "map":
		theMap_.layers["map"] = map;
	}
	else
	{
		// MAP FORMAT 1: MP2P_ICP "*.mm" metric maps
		// -------------------------------------------
		std::string mm_file;

		this->declare_parameter<std::string>("mm_file", "map.mm");
		this->get_parameter("mm_file", mm_file);
		RCLCPP_INFO(this->get_logger(), "mm_file: '%s'", mm_file.c_str());

		ASSERT_FILE_EXISTS_(mm_file);

		RCLCPP_INFO_STREAM(
			this->get_logger(),
			"Loading metric_map_t map from '" << mm_file << "' ...");

		bool mapReadOk = theMap_.load_from_file(mm_file);
		ASSERT_(mapReadOk);

		RCLCPP_INFO_STREAM(
			this->get_logger(),
			"Loaded map contents: " << theMap_.contents_summary());
	}

	this->declare_parameter<std::string>("frame_id", "map");
	this->get_parameter("frame_id", m_response_ros.map.header.frame_id);
	RCLCPP_INFO(
		this->get_logger(), "frame_id: '%s'",
		m_response_ros.map.header.frame_id.c_str());

	this->declare_parameter<double>("frequency", m_frequency);
	this->get_parameter("frequency", m_frequency);
	RCLCPP_INFO(this->get_logger(), "frequency: %f", m_frequency);

	this->declare_parameter<std::string>("pub_mm_topic", pub_mm_topic_);
	this->get_parameter("pub_mm_topic", pub_mm_topic_);
	RCLCPP_INFO(
		this->get_logger(), "pub_mm_topic: '%s'", pub_mm_topic_.c_str());

	this->declare_parameter<std::string>("service_map", m_service_map_str);
	this->get_parameter("service_map", m_service_map_str);
	RCLCPP_INFO(
		this->get_logger(), "service_map: '%s'", m_service_map_str.c_str());

	m_service_map = this->create_service<nav_msgs::srv::GetMap>(
		m_service_map_str,
		[this](
			const nav_msgs::srv::GetMap::Request::SharedPtr req,
			const nav_msgs::srv::GetMap::Response::SharedPtr res) {
			this->map_callback(req, res);
		});

	//	mrpt::ros2bridge::toROS(*grid, m_response_ros.map);
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
	using namespace std::string_literals;

	auto now = std::chrono::system_clock::now();

	m_response_ros.map.header.stamp =
		rclcpp::Time(now.time_since_epoch().count(), RCL_ROS_TIME);

	// 1st: top-level MM map:
	if (!pubMM_)
	{
		pubMM_ = this->create_publisher<mrpt_msgs::msg::GenericObject>(
			pub_mm_topic_ + "/metric_map"s, 1);
	}
	if (pubMM_->get_subscription_count() > pubMM_subscribers_)
	{
		// xxx
	}
	pubMM_subscribers_ = pubMM_->get_subscription_count();

	// 2nd: each layer:

#if 0
	m_pub_map_ros = m_pub_metadata =
		this->create_publisher<nav_msgs::msg::MapMetaData>(
			m_pub_metadata_str, 1);

	if (m_pub_map_ros->get_subscription_count() > 0)
	{
		m_pub_map_ros->publish(m_response_ros.map);
	}
	if (m_pub_metadata->get_subscription_count() > 0)
	{
		m_pub_metadata->publish(m_response_ros.map.info);
	}
#endif
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
