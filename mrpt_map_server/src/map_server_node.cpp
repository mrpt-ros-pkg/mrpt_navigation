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

#include <nav_msgs/msg/occupancy_grid.hpp>

using namespace mrpt::config;
using mrpt::maps::CMultiMetricMap;
using mrpt::maps::COccupancyGridMap2D;

MapServer::MapServer() : Node("mrpt_map_server") {}

MapServer::~MapServer() {}

void MapServer::init()
{
	using namespace std::string_literals;

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

	this->declare_parameter<std::string>("pub_mm_topic", pub_mm_topic_);
	this->get_parameter("pub_mm_topic", pub_mm_topic_);
	RCLCPP_INFO(
		this->get_logger(), "pub_mm_topic: '%s'", pub_mm_topic_.c_str());

	this->declare_parameter<std::string>("service_map", service_map_str_);
	this->get_parameter("service_map", service_map_str_);
	RCLCPP_INFO(
		this->get_logger(), "service_map: '%s'", service_map_str_.c_str());

	srvMapLayers_ = this->create_service<mrpt_nav_interfaces::srv::GetLayers>(
		this->get_fully_qualified_name() + "/get_layers"s,
		[this](
			const mrpt_nav_interfaces::srv::GetLayers::Request::SharedPtr req,
			mrpt_nav_interfaces::srv::GetLayers::Response::SharedPtr res)
		{ srv_map_layers(req, res); });

	srvGetGrid_ = this->create_service<
		mrpt_nav_interfaces::srv::GetGridmapLayer>(
		this->get_fully_qualified_name() + "/get_grid_layer"s,
		[this](
			const mrpt_nav_interfaces::srv::GetGridmapLayer::Request::SharedPtr
				req,
			mrpt_nav_interfaces::srv::GetGridmapLayer::Response::SharedPtr res)
		{ srv_get_gridmap(req, res); });

	srvGetPoints_ = this->create_service<
		mrpt_nav_interfaces::srv::GetPointmapLayer>(
		this->get_fully_qualified_name() + "/get_pointcloud_layer"s,
		[this](
			const mrpt_nav_interfaces::srv::GetPointmapLayer::Request::SharedPtr
				req,
			mrpt_nav_interfaces::srv::GetPointmapLayer::Response::SharedPtr res)
		{ srv_get_pointmap(req, res); });
}

void MapServer::publish_map()
{
	using namespace std::string_literals;

	// REP-2003: https://ros.org/reps/rep-2003.html
	// Maps:  reliable transient-local
	auto QoS = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

	auto lck = mrpt::lockHelper(theMapMtx_);

	// 1st: top-level MM map:
	if (!pubMM_.pub)
	{
		pubMM_.pub = this->create_publisher<mrpt_msgs::msg::GenericObject>(
			pub_mm_topic_ + "/metric_map"s, QoS);
	}

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
					pub_mm_topic_ + "/"s + layerName, QoS);
		}

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
						pub_mm_topic_ + "/"s + layerName + "_points"s, QoS);
			}

			{
				const sensor_msgs::msg::PointCloud2 msg_pts =
					pointmap_layer_to_msg(pts);

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
						pub_mm_topic_ + "/"s + layerName + "_points"s, QoS);
			}

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
						pub_mm_topic_ + "/"s + layerName + "_gridmap"s, QoS);

				pubGridMapsMetaData_[layerName].pub =
					this->create_publisher<nav_msgs::msg::MapMetaData>(
						pub_mm_topic_ + "/"s + layerName + "_gridmap_metadata"s,
						QoS);
			}

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
	publish_map();
	rclcpp::spin(shared_from_this());
}

void MapServer::srv_map_layers(
	const std::shared_ptr<mrpt_nav_interfaces::srv::GetLayers::Request> req,
	const std::shared_ptr<mrpt_nav_interfaces::srv::GetLayers::Response> resp)
{
	auto lck = mrpt::lockHelper(theMapMtx_);

	// req: empty
	(void)req;

	resp->layers.clear();
	for (const auto& [layerName, _] : theMap_.layers)
		resp->layers.push_back(layerName);
}

void MapServer::srv_get_gridmap(
	const std::shared_ptr<mrpt_nav_interfaces::srv::GetGridmapLayer::Request>
		req,
	const std::shared_ptr<mrpt_nav_interfaces::srv::GetGridmapLayer::Response>
		resp)
{
	auto lck = mrpt::lockHelper(theMapMtx_);

	resp->valid = false;

	if (theMap_.layers.count(req->layer_name) == 0) return;

	auto m = theMap_.layers.at(req->layer_name);
	auto grid = std::dynamic_pointer_cast<mrpt::maps::COccupancyGridMap2D>(m);
	if (!grid) return;	// it's not a gridmap

	std_msgs::msg::Header msg_header;
	msg_header.stamp = this->get_clock()->now();
	msg_header.frame_id = frame_id_;

	mrpt::ros2bridge::toROS(*grid, resp->grid, msg_header);

	resp->valid = true;
}

void MapServer::srv_get_pointmap(
	const std::shared_ptr<mrpt_nav_interfaces::srv::GetPointmapLayer::Request>
		req,
	std::shared_ptr<mrpt_nav_interfaces::srv::GetPointmapLayer::Response> resp)
{
	auto lck = mrpt::lockHelper(theMapMtx_);

	resp->valid = false;

	if (theMap_.layers.count(req->layer_name) == 0) return;

	auto m = theMap_.layers.at(req->layer_name);
	auto pts = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(m);
	if (!pts) return;  // it's not a point cloud

	std_msgs::msg::Header msg_header;
	msg_header.stamp = this->get_clock()->now();
	msg_header.frame_id = frame_id_;

	resp->points = pointmap_layer_to_msg(pts);

	resp->valid = true;
}

sensor_msgs::msg::PointCloud2 MapServer::pointmap_layer_to_msg(
	const mrpt::maps::CPointsMap::Ptr& pts)
{
	std_msgs::msg::Header msg_header;
	msg_header.stamp = this->get_clock()->now();
	msg_header.frame_id = frame_id_;

	sensor_msgs::msg::PointCloud2 msg_pts;

	if (auto* xyzirt =
			dynamic_cast<const mrpt::maps::CPointsMapXYZIRT*>(pts.get());
		xyzirt)
	{
		mrpt::ros2bridge::toROS(*xyzirt, msg_header, msg_pts);
	}
	else if (auto* xyzi =
				 dynamic_cast<const mrpt::maps::CPointsMapXYZI*>(pts.get());
			 xyzi)
	{
		mrpt::ros2bridge::toROS(*xyzi, msg_header, msg_pts);
	}
	else if (auto* sPts =
				 dynamic_cast<const mrpt::maps::CSimplePointsMap*>(pts.get());
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

	return msg_pts;
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
