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
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                    *                       *
 ***********************************************************************************/

#include "mrpt_localization_node.h"
#include "mrpt_localization_node_defaults.h"

PFLocalizationNode::ParametersNode::ParametersNode()
    : Parameters(), node("~") {
    node.param<double>("rate", rate, MRPT_LOCALIZATION_NODE_DEFAULT_RATE);
    ROS_INFO("rate: %f", rate);
    node.param<int>("parameter_update_skip", parameter_update_skip, MRPT_LOCALIZATION_NODE_DEFAULT_PARAMETER_UPDATE_SKIP);
    ROS_INFO("parameter_update_skip: %i", parameter_update_skip);
    node.getParam("ini_file", iniFile);
    ROS_INFO("ini_file: %s", iniFile.c_str());
    node.getParam("rawlog_file", rawlogFile);
    ROS_INFO("rawlog_file: %s", rawlogFile.c_str());
    node.getParam("map_file", mapFile);
    ROS_INFO("map_file: %s", mapFile.c_str());
    
    reconfigureFnc_ = boost::bind(&PFLocalizationNode::ParametersNode::callbackParameters, this ,  _1, _2);
    reconfigureServer_.setCallback(reconfigureFnc_);
}

void PFLocalizationNode::ParametersNode::update(const unsigned long &loop_count) {
    if(loop_count % parameter_update_skip) return;
    node.getParam("debug", debug);
    if(loop_count == 0) ROS_INFO("debug:  %s", (debug ? "true" : "false"));
}

void PFLocalizationNode::ParametersNode::callbackParameters (mrpt_localization::LocalizationConfig &config, uint32_t level ) {
    
}