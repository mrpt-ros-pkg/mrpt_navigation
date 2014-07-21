/***********************************************************************************
 * Revised BSD License                                                             *
 * Copyright (c) 2014, Markus Bader <markus.bader@tuwien.ac.at>                    *
 * All rights reserved.                                                            *
 *                                                                                 *
 * Redistribution and use in source and binary forms, with or without              *
 * modification, are permitted provided that the following conditions are met:     *
 *     * Redistributions of source code must retain the above copyright            *
 *       notice, this list of conditions and the following disclaimer.             *
 *     * Redistributions in binary form must reproduce the above copyright         *
 *       notice, this list of conditions and the following disclaimer in the       *
 *       documentation and/or other materials provided with the distribution.      *
 *     * Neither the name of the Vienna University of Technology nor the           *
 *       names of its contributors may be used to endorse or promote products      *
 *       derived from this software without specific prior written permission.     *
 *                                                                                 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND *
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED   *
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE          *
 * DISCLAIMED. IN NO EVENT SHALL Markus Bader BE LIABLE FOR ANY                    *
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES      *
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;    *
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND     *
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT      *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS   *
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                    * 
 ***********************************************************************************/

#include <map_server_node.h>
#include <mrpt/base.h>
#include <mrpt/slam.h>

MapServer::MapServer(ros::NodeHandle &n)
    : n_(n)
    , n_param_("~")
    , loop_count_(0)
    , debug_(true) {
}
MapServer::~MapServer() {
}
void MapServer::init() {
    std::string init_file;
    std::string map_file;
    n_param_.getParam("debug", debug_);
    ROS_INFO("debug: %s", (debug_?"true":"false"));
    n_param_.getParam("init_file", init_file);
    ROS_INFO("init_file: %s", init_file.c_str());
    n_param_.getParam("map_file", map_file);
    ROS_INFO("map_file: %s", map_file.c_str());

    //ASSERT_FILE_EXISTS_(init_file);
    mrpt::utils::CConfigFile config_file;
    config_file.setFileName(init_file);
    mrpt::slam::CMultiMetricMap metric_map;
    loadMap(config_file, map_file, metric_map);
}
void MapServer::loop() {
    ros::Rate rate(10);
    while (ros::ok())
    {

    }
}

bool MapServer::loadMap(const mrpt::utils::CConfigFile &_config_file, const std::string &_map_file, mrpt::slam::CMultiMetricMap &_metric_map) {
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "mrpt_map_server");
    ros::NodeHandle node;
    MapServer my_node(node);
    my_node.loop();
    return 0;
}
