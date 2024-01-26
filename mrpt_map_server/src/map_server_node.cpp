/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include "mrpt_map_server/map_server_node.h"

#include <mrpt/config/CConfigFile.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/maps/CPointsMapXYZIRT.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CVoxelMap.h>
#include <mrpt/ros2bridge/map.h>
#include <mrpt/ros2bridge/point_cloud2.h>
#include <mrpt/ros2bridge/time.h>
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

	auto lck = mrpt::lockHelper(theMapMtx_);

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

	this->declare_parameter<std::string>("frame_id", frame_id_);
	this->get_parameter("frame_id", frame_id_);
	RCLCPP_INFO(this->get_logger(), "frame_id: '%s'", frame_id_.c_str());

	this->declare_parameter<double>("frequency", frequency_);
	this->get_parameter("frequency", frequency_);
	RCLCPP_INFO(this->get_logger(), "frequency: %f", frequency_);

	this->declare_parameter<std::string>("pub_mm_topic", pub_mm_topic_);
	this->get_parameter("pub_mm_topic", pub_mm_topic_);
	RCLCPP_INFO(
		this->get_logger(), "pub_mm_topic: '%s'", pub_mm_topic_.c_str());

	this->declare_parameter<std::string>("service_map", service_map_str_);
	this->get_parameter("service_map", service_map_str_);
	RCLCPP_INFO(
		this->get_logger(), "service_map: '%s'", service_map_str_.c_str());

#if 0
	m_service_map = this->create_service<nav_msgs::srv::GetMap>(
		m_service_map_str,
		[this](
			const nav_msgs::srv::GetMap::Request::SharedPtr req,
			const nav_msgs::srv::GetMap::Response::SharedPtr res) {
			this->map_callback(req, res);
		});
#endif
}

bool MapServer::map_callback(
	const std::shared_ptr<nav_msgs::srv::GetMap::Request> req,
	const std::shared_ptr<nav_msgs::srv::GetMap::Response> res)
{
	RCLCPP_INFO(this->get_logger(), "mapCallback: service requested");
	//	*res = m_response_ros;
	return true;
}

void MapServer::publish_map()
{
	using namespace std::string_literals;
	auto lck = mrpt::lockHelper(theMapMtx_);

	// 1st: top-level MM map:
	if (!pubMM_.pub)
	{
		pubMM_.pub = this->create_publisher<mrpt_msgs::msg::GenericObject>(
			pub_mm_topic_ + "/metric_map"s, 1);
	}
	if (pubMM_.new_subscribers())
	{
		mrpt_msgs::msg::GenericObject msg;
		mrpt::serialization::ObjectToOctetVector(&theMap_, msg.data);
		pubMM_.pub->publish(msg);
	}

	std_msgs::msg::Header msg_header;
	msg_header.stamp = this->get_clock()->now();
	msg_header.frame_id = frame_id_;

	// 2nd: each layer:
	for (const auto& [layerName, layerMap] : theMap_.layers)
	{
		// 2.1) for any map, publish it in mrpt binary form:
		if (pubLayers_.count(layerName) == 0)
		{
			pubLayers_[layerName].pub =
				this->create_publisher<mrpt_msgs::msg::GenericObject>(
					pub_mm_topic_ + "/"s + layerName, 1);
		}
		if (pubLayers_[layerName].new_subscribers())
		{
			mrpt_msgs::msg::GenericObject msg;
			mrpt::serialization::ObjectToOctetVector(layerMap.get(), msg.data);
			pubLayers_[layerName].pub->publish(msg);
		}

		// 2.2) publish as ROS standard msgs, if applicable too:
		// Is it a pointcloud?
		if (auto pts =
				std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(layerMap);
			pts)
		{
			if (pubPointMaps_.count(layerName) == 0)
			{
				pubPointMaps_[layerName].pub =
					this->create_publisher<sensor_msgs::msg::PointCloud2>(
						pub_mm_topic_ + "/"s + layerName + "_points"s, 1);
			}
			if (pubPointMaps_[layerName].new_subscribers())
			{
				sensor_msgs::msg::PointCloud2 msg_pts;

				if (auto* xyzirt =
						dynamic_cast<const mrpt::maps::CPointsMapXYZIRT*>(
							pts.get());
					xyzirt)
				{
					mrpt::ros2bridge::toROS(*xyzirt, msg_header, msg_pts);
				}
				else if (auto* xyzi =
							 dynamic_cast<const mrpt::maps::CPointsMapXYZI*>(
								 pts.get());
						 xyzi)
				{
					mrpt::ros2bridge::toROS(*xyzi, msg_header, msg_pts);
				}
				else if (auto* sPts =
							 dynamic_cast<const mrpt::maps::CSimplePointsMap*>(
								 pts.get());
						 sPts)
				{
					mrpt::ros2bridge::toROS(*sPts, msg_header, msg_pts);
				}
				else
				{
					THROW_EXCEPTION_FMT(
						"Unexpected point cloud class: '%s'",
						pts->GetRuntimeClass()->className);
				}
				pubPointMaps_[layerName].pub->publish(msg_pts);
			}
		}

		// Is it a voxelmap?
		if (auto vxl =
				std::dynamic_pointer_cast<mrpt::maps::CVoxelMap>(layerMap);
			vxl)
		{
			// Render in RVIZ as occupied voxels=points:
			mrpt::maps::CSimplePointsMap::Ptr pts = vxl->getOccupiedVoxels();

			if (pubPointMaps_.count(layerName) == 0)
			{
				pubPointMaps_[layerName].pub =
					this->create_publisher<sensor_msgs::msg::PointCloud2>(
						pub_mm_topic_ + "/"s + layerName + "_points"s, 1);
			}
			if (pubPointMaps_[layerName].new_subscribers())
			{
				sensor_msgs::msg::PointCloud2 msg_pts;
				mrpt::ros2bridge::toROS(*pts, msg_header, msg_pts);
				pubPointMaps_[layerName].pub->publish(msg_pts);
			}
		}

		// Is it a grid map?
		if (auto grid =
				std::dynamic_pointer_cast<mrpt::maps::COccupancyGridMap2D>(
					layerMap);
			grid)
		{
			if (pubGridMaps_.count(layerName) == 0)
			{
				pubGridMaps_[layerName].pub =
					this->create_publisher<nav_msgs::msg::OccupancyGrid>(
						pub_mm_topic_ + "/"s + layerName + "_gridmap"s, 1);

				pubGridMapsMetaData_[layerName].pub =
					this->create_publisher<nav_msgs::msg::MapMetaData>(
						pub_mm_topic_ + "/"s + layerName + "_gridmap_metadata"s,
						1);
			}
			// Note: DONT merge this into a single (..||..) to avoid
			// shortcircuit logic, since we want both calls to be evaluated for
			// their side effects.
			const bool b1 = pubGridMaps_[layerName].new_subscribers();
			const bool b2 = pubGridMapsMetaData_[layerName].new_subscribers();
			if (b1 || b2)
			{
				nav_msgs::msg::OccupancyGrid gridMsg;
				mrpt::ros2bridge::toROS(*grid, gridMsg, msg_header);
				pubGridMaps_[layerName].pub->publish(gridMsg);
				pubGridMapsMetaData_[layerName].pub->publish(gridMsg.info);
			}
		}

	}  // end for each layer
}

void MapServer::loop()
{
	if (frequency_ > 0)
	{
		rclcpp::Rate rate(frequency_);
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
