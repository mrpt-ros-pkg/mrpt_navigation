/***********************************************************************************
 * Revised BSD License *
 * Copyright (c) 2014-2023, Markus Bader <markus.bader@tuwien.ac.at> *
 * All rights reserved. *
 *                                                                                 *
 * Redistribution and use in source and binary forms, with or without *
 * modification, are permitted provided that the following conditions are met: *
 *     * Redistributions of source code must retain the above copyright *
 *       notice, this list of conditions and the following disclaimer. *
 *     * Redistributions in binary form must reproduce the above copyright *
 *       notice, this list of conditions and the following disclaimer in the *
 *       documentation and/or other materials provided with the distribution. *
 *     * Neither the name of the Vienna University of Technology nor the *
 *       names of its contributors may be used to endorse or promote products *
 *       derived from this software without specific prior written permission. *
 *                                                                                 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND *
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 **
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE *
 * DISCLAIMED. IN NO EVENT SHALL Markus Bader BE LIABLE FOR ANY *
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES *
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 **
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND *
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 **
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. *
 ***********************************************************************************/

#include "mrpt_map/map_server_node.hpp"
#include <mrpt/config/CConfigFile.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/ros2bridge/map.h>
#include <mrpt/system/filesystem.h> // ASSERT_FILE_EXISTS_()
#include <nav_msgs/msg/occupancy_grid.hpp>

using namespace mrpt::config;
using mrpt::maps::CMultiMetricMap;
using mrpt::maps::COccupancyGridMap2D;

MapServer::MapServer() : Node("mrpt_map_server")
{
}

MapServer::~MapServer() {}

void MapServer::init()
{
	this->declare_parameter<bool>("debug", false);
	this->get_parameter("debug", m_debug);
  	RCLCPP_INFO(this->get_logger(), "debug: %s",  m_debug?"true":"false");

	mrpt::maps::COccupancyGridMap2D::Ptr grid;
	std::string map_yaml_file;
	this->declare_parameter<std::string>("map_yaml_file", "");
	this->get_parameter("map_yaml_file", map_yaml_file);
  	RCLCPP_INFO(this->get_logger(), "map_yaml_file name: %s", map_yaml_file.c_str());

	if(!map_yaml_file.empty())
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
  		RCLCPP_INFO(this->get_logger(), "map_ini_file name: %s", ini_file.c_str());

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
  	RCLCPP_INFO(this->get_logger(), "frame_id: %s", m_response_ros.map.header.frame_id.c_str());

	this->declare_parameter<double>("frequency", 0.1);
	this->get_parameter("frequency", m_frequency);
  	RCLCPP_INFO(this->get_logger(), "frequency: %f", m_frequency);

	this->declare_parameter<std::string>("pub_map_ros", "map");
	this->get_parameter("pub_map_ros", m_pub_map_ros_str);
  	RCLCPP_INFO(this->get_logger(), "pub_map_ros: %s", m_pub_map_ros_str.c_str());

	this->declare_parameter<std::string>("pub_metadata", "map_metadata");
	this->get_parameter("pub_metadata", m_pub_metadata_str);
  	RCLCPP_INFO(this->get_logger(), "pub_metadata: %s", m_pub_metadata_str.c_str());


}

bool MapServer::map_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav_msgs::srv::GetMap::Request> req,
    const std::shared_ptr<nav_msgs::srv::GetMap::Response> res)
{
  RCLCPP_INFO(this->get_logger(), "mapCallback: service requested");
  *res = m_response_ros;
  return true;
}

void MapServer::publish_map()
{
  
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

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapServer>();
  node->init();
  node->loop();
  rclcpp::shutdown();
  return 0;
}
