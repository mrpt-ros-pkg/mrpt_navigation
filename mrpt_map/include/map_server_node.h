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
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **                       *
 ***********************************************************************************/

#ifndef MRPT_MAP_SERVER_NODE_H
#define MRPT_MAP_SERVER_NODE_H

#include "ros/ros.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/GetMap.h"
#include "boost/smart_ptr.hpp"

#include <mrpt/version.h>
#if MRPT_VERSION >= 0x199
#include <mrpt/config/CConfigFile.h>
using namespace mrpt::config;
#else
#include <mrpt/utils/CConfigFile.h>
using namespace mrpt::utils;
#endif

#include <mrpt/maps/CMultiMetricMap.h>
using mrpt::maps::CMultiMetricMap;

class MapServer
{
   public:
	MapServer(ros::NodeHandle& n);
	~MapServer();
	void init();
	void loop();

   private:
	ros::NodeHandle n_;
	ros::NodeHandle n_param_{"~"};
	double frequency_{0};
	unsigned long loop_count_{0};
	bool debug_{true};
	ros::Publisher pub_map_ros_;
	ros::Publisher pub_metadata_;
	ros::ServiceServer service_map_;
	nav_msgs::GetMap::Response resp_ros_;
	boost::shared_ptr<CMultiMetricMap> metric_map_;
	void publishMap();
	bool mapCallback(
		nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res);
};

#endif  // MRPT_MAP_SERVER_NODE_H
