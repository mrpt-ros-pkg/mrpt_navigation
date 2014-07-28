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

PFLocalizationNode::Parameters::Parameters()
    : PFLocalization::Parameters(), node("~") {
    node.param<double>("rate", rate, MRPT_LOCALIZATION_NODE_DEFAULT_RATE);
    ROS_INFO("rate: %f", rate);
    node.param<int>("parameter_update_skip", parameter_update_skip, MRPT_LOCALIZATION_NODE_DEFAULT_PARAMETER_UPDATE_SKIP);
    ROS_INFO("parameter_update_skip: %i", parameter_update_skip);
    node.getParam("ini_file", iniFile);
    ROS_INFO("ini_file: %s", iniFile.c_str());
    node.getParam("map_file", mapFile);
    ROS_INFO("map_file: %s", mapFile.c_str());
    node.param<std::string>("global_frame_id", global_frame_id, "map");
    ROS_INFO("global_frame_id: %s", global_frame_id.c_str());
    node.param<std::string>("odom_frame_id", odom_frame_id, "odom");
    ROS_INFO("odom_frame_id: %s", odom_frame_id.c_str());

    reconfigureFnc_ = boost::bind(&PFLocalizationNode::Parameters::callbackParameters, this ,  _1, _2);
    reconfigureServer_.setCallback(reconfigureFnc_);
}

void PFLocalizationNode::Parameters::update(const unsigned long &loop_count) {
    if(loop_count % parameter_update_skip) return;
    node.getParam("debug", debug);
    if(loop_count == 0) ROS_INFO("debug:  %s", (debug ? "true" : "false"));
    {
        int v = particlecloud_update_skip;
        node.param<int>("particlecloud_update_skip", particlecloud_update_skip, MRPT_LOCALIZATION_NODE_DEFAULT_PARTICLECLOUD_UPDATE_SKIP);
        if(v != particlecloud_update_skip) ROS_INFO("particlecloud_update_skip: %i", particlecloud_update_skip);
    }
    {
        int v = map_update_skip;
        node.param<int>("map_update_skip", map_update_skip, MRPT_LOCALIZATION_NODE_DEFAULT_MAP_UPDATE_SKIP);
        if(v != map_update_skip) ROS_INFO("map_update_skip: %i", map_update_skip);
    }
}

void PFLocalizationNode::Parameters::callbackParameters (mrpt_localization::MotionConfig &config, uint32_t level ) {
    if(config.motion_noise_type == MOTION_MODEL_GAUSSIAN) {
        motionModelOptions.modelSelection = mrpt::slam::CActionRobotMovement2D::mmGaussian;
        motionModelOptions.gausianModel.a1 = config.gaussian_alpha_1;
        motionModelOptions.gausianModel.a2 = config.gaussian_alpha_2;
        motionModelOptions.gausianModel.a3 = config.gaussian_alpha_3;
        motionModelOptions.gausianModel.a4 = config.gaussian_alpha_4;
        motionModelOptions.gausianModel.minStdXY  = config.gaussian_alpha_xy;
        motionModelOptions.gausianModel.minStdPHI = config.gaussian_alpha_phi;
        ROS_INFO("gausianModel.a1: %f", motionModelOptions.gausianModel.a1);
        ROS_INFO("gausianModel.a2: %f", motionModelOptions.gausianModel.a2);
        ROS_INFO("gausianModel.a3: %f", motionModelOptions.gausianModel.a3);
        ROS_INFO("gausianModel.a4: %f", motionModelOptions.gausianModel.a4);
        ROS_INFO("gausianModel.minStdXY: %f", motionModelOptions.gausianModel.minStdXY);
        ROS_INFO("gausianModel.minStdPHI: %f", motionModelOptions.gausianModel.minStdPHI);
    }
}
