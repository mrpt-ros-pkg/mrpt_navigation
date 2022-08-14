/***********************************************************************************
 * Revised BSD License *
 * Copyright (c) 2014, Markus Bader <markus.bader@tuwien.ac.at> *
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

#include <map_server_node.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/ros1bridge/map.h>
#include <mrpt/system/filesystem.h>	 // ASSERT_FILE_EXISTS_()

using namespace mrpt::config;
using mrpt::maps::CMultiMetricMap;
using mrpt::maps::COccupancyGridMap2D;

MapServer::MapServer(ros::NodeHandle& n) : n_(n) {}

MapServer::~MapServer() {}

void MapServer::init()
{
	n_param_.param<bool>("debug", debug_, true);
	ROS_INFO("debug: %s", (debug_ ? "true" : "false"));

	mrpt::maps::COccupancyGridMap2D::Ptr grid;

	if (auto yamlFile = n_param_.param<std::string>("map_yaml_file", "");
		!yamlFile.empty())
	{
		ROS_INFO("map_yaml_file: %s", yamlFile.c_str());

		grid = mrpt::maps::COccupancyGridMap2D::Create();
		grid->loadFromROSMapServerYAML(yamlFile);
	}
	else
	{
		std::string ini_file;
		std::string map_file;
		n_param_.param<std::string>("ini_file", ini_file, "map.ini");

		n_param_.param<std::string>("map_file", map_file, "map.simplemap");

		ROS_INFO("ini_file: %s", ini_file.c_str());
		ROS_INFO("map_file: %s", map_file.c_str());

		ASSERT_FILE_EXISTS_(ini_file);
		ASSERT_FILE_EXISTS_(map_file);
		CConfigFile config_file;
		config_file.setFileName(ini_file);

		metric_map_ = CMultiMetricMap::Create();

		mrpt::ros1bridge::MapHdl::loadMap(
			*metric_map_, config_file, map_file, "metricMap", debug_);

		grid = metric_map_->mapByClass<COccupancyGridMap2D>();
	}

	ASSERT_(grid);

	n_param_.param<std::string>(
		"frame_id", resp_ros_.map.header.frame_id, "map");
	ROS_INFO("frame_id: %s", resp_ros_.map.header.frame_id.c_str());
	n_param_.param<double>("frequency", frequency_, 0.1);
	ROS_INFO("frequency: %f", frequency_);

	pub_map_ros_ = n_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
	pub_metadata_ =
		n_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
	service_map_ =
		n_.advertiseService("static_map", &MapServer::mapCallback, this);

	if (debug_)
		printf(
			"gridMap[0]:  %i x %i @ %4.3fm/p, %4.3f, %4.3f, %4.3f, %4.3f\n",
			grid->getSizeX(), grid->getSizeY(), grid->getResolution(),
			grid->getXMin(), grid->getYMin(), grid->getXMax(), grid->getYMax());

	mrpt::ros1bridge::toROS(*grid, resp_ros_.map);

	if (debug_)
		printf(
			"msg:         %i x %i @ %4.3fm/p, %4.3f, %4.3f, %4.3f, %4.3f\n",
			resp_ros_.map.info.width, resp_ros_.map.info.height,
			resp_ros_.map.info.resolution, resp_ros_.map.info.origin.position.x,
			resp_ros_.map.info.origin.position.y,
			resp_ros_.map.info.width * resp_ros_.map.info.resolution +
				resp_ros_.map.info.origin.position.x,
			resp_ros_.map.info.height * resp_ros_.map.info.resolution +
				resp_ros_.map.info.origin.position.y);
}

bool MapServer::mapCallback(
	nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res)
{
	ROS_INFO("mapCallback: service requested!\n");
	res = resp_ros_;
	return true;
}

void MapServer::publishMap()
{
	resp_ros_.map.header.stamp = ros::Time::now();
	resp_ros_.map.header.seq = loop_count_;
	if (pub_map_ros_.getNumSubscribers() > 0)
	{
		pub_map_ros_.publish(resp_ros_.map);
	}
	if (pub_metadata_.getNumSubscribers() > 0)
	{
		pub_metadata_.publish(resp_ros_.map.info);
	}
}

void MapServer::loop()
{
	if (frequency_ > 0)
	{
		ros::Rate rate(frequency_);
		for (loop_count_ = 0; ros::ok(); loop_count_++)
		{
			publishMap();
			ros::spinOnce();
			rate.sleep();
		}
	}
	else
	{
		publishMap();
		ros::spin();
	}
}

int main(int argc, char** argv)
{
	// Initialize ROS
	ros::init(argc, argv, "mrpt_map_server");
	ros::NodeHandle node;
	MapServer my_node(node);
	my_node.init();
	my_node.loop();
	return 0;
}
