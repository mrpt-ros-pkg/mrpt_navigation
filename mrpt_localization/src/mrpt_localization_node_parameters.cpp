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

PFLocalizationNode::Parameters::Parameters(PFLocalizationNode *p) :
    PFLocalization::Parameters(p), node("~")
{
  node.param<double>("transform_tolerance", transform_tolerance, 0.1);
  ROS_INFO("transform_tolerance: %f", transform_tolerance);
  node.param<double>("no_update_tolerance", no_update_tolerance, 1.0);
  ROS_INFO("no_update_tolerance: %f", no_update_tolerance);
  node.param<double>("no_inputs_tolerance", no_inputs_tolerance, std::numeric_limits<double>::infinity());
  ROS_INFO("no_inputs_tolerance: %f", no_inputs_tolerance);  // disabled by default
  node.param<double>("rate", rate, MRPT_LOCALIZATION_NODE_DEFAULT_RATE);
  ROS_INFO("rate: %f", rate);
  node.getParam("gui_mrpt", gui_mrpt);
  ROS_INFO("gui_mrpt: %s", gui_mrpt ? "true" : "false");
  node.param<int>("parameter_update_skip", parameter_update_skip, MRPT_LOCALIZATION_NODE_DEFAULT_PARAMETER_UPDATE_SKIP);
  ROS_INFO("parameter_update_skip: %i", parameter_update_skip);
  node.getParam("ini_file", ini_file);
  ROS_INFO("ini_file: %s", ini_file.c_str());
  node.getParam("map_file", map_file);
  ROS_INFO("map_file: %s", map_file.c_str());
  node.getParam("sensor_sources", sensor_sources);
  ROS_INFO("sensor_sources: %s", sensor_sources.c_str());
  node.param<std::string>("tf_prefix", tf_prefix, "");
  ROS_INFO("tf_prefix: %s", tf_prefix.c_str());
  node.param<std::string>("global_frame_id", global_frame_id, "map");
  ROS_INFO("global_frame_id: %s", global_frame_id.c_str());
  node.param<std::string>("odom_frame_id", odom_frame_id, "odom");
  ROS_INFO("odom_frame_id: %s", odom_frame_id.c_str());
  node.param<std::string>("base_frame_id", base_frame_id, "base_link");
  ROS_INFO("base_frame_id: %s", base_frame_id.c_str());
  node.param<bool>("pose_broadcast", pose_broadcast, false);
  ROS_INFO("pose_broadcast: %s", pose_broadcast ? "true" : "false");
  node.param<bool>("tf_broadcast", tf_broadcast, true);
  ROS_INFO("tf_broadcast: %s", tf_broadcast ? "true" : "false");
  node.param<bool>("debug", debug, true);
  ROS_INFO("debug: %s", debug? "true" : "false");

  reconfigure_cb_ = boost::bind(&PFLocalizationNode::Parameters::callbackParameters, this, _1, _2);
  reconfigure_server_.setCallback(reconfigure_cb_);
}

void PFLocalizationNode::Parameters::update(const unsigned long &loop_count)
{
  if (loop_count % parameter_update_skip)
    return;
  node.getParam("debug", debug);
  if (loop_count == 0)
    ROS_INFO("debug: %s", debug ? "true" : "false");
  {
    int v = particlecloud_update_skip;
    node.param<int>("particlecloud_update_skip", particlecloud_update_skip,
                    MRPT_LOCALIZATION_NODE_DEFAULT_PARTICLECLOUD_UPDATE_SKIP);
    if (v != particlecloud_update_skip)
      ROS_INFO("particlecloud_update_skip: %i", particlecloud_update_skip);
  }
  {
    int v = map_update_skip;
    node.param<int>("map_update_skip", map_update_skip, MRPT_LOCALIZATION_NODE_DEFAULT_MAP_UPDATE_SKIP);
    if (v != map_update_skip)
      ROS_INFO("map_update_skip: %i", map_update_skip);
  }
}

void PFLocalizationNode::Parameters::callbackParameters(mrpt_localization::MotionConfig &config, uint32_t level)
{
  if (config.motion_noise_type == MOTION_MODEL_GAUSSIAN)
  {
    motion_model_options->modelSelection = CActionRobotMovement2D::mmGaussian;
#if MRPT_VERSION>=0x150
#define gausianModel gaussianModel    // a typo was fixed in 1.5.0
#endif
	motion_model_options->gausianModel.a1 = config.gaussian_alpha_1;
	motion_model_options->gausianModel.a2 = config.gaussian_alpha_2;
    motion_model_options->gausianModel.a3 = config.gaussian_alpha_3;
    motion_model_options->gausianModel.a4 = config.gaussian_alpha_4;
    motion_model_options->gausianModel.minStdXY = config.gaussian_alpha_xy;
    motion_model_options->gausianModel.minStdPHI = config.gaussian_alpha_phi;
	ROS_INFO("gaussianModel.type: gaussian");
	ROS_INFO("gaussianModel.a1: %f", motion_model_options->gausianModel.a1);
	ROS_INFO("gaussianModel.a2: %f", motion_model_options->gausianModel.a2);
	ROS_INFO("gaussianModel.a3: %f", motion_model_options->gausianModel.a3);
	ROS_INFO("gaussianModel.a4: %f", motion_model_options->gausianModel.a4);
	ROS_INFO("gaussianModel.minStdXY: %f", motion_model_options->gausianModel.minStdXY);
	ROS_INFO("gaussianModel.minStdPHI: %f", motion_model_options->gausianModel.minStdPHI);
  }
  else
  {
    ROS_INFO("We support at the moment only gaussian motion models");
  }
  *use_motion_model_default_options = config.use_default_motion;
  ROS_INFO("use_motion_model_default_options: %s", use_motion_model_default_options ? "true" : "false");
  motion_model_default_options->gausianModel.minStdXY = config.default_noise_xy;
  ROS_INFO("default_noise_xy: %f", motion_model_default_options->gausianModel.minStdXY);
  motion_model_default_options->gausianModel.minStdPHI = config.default_noise_phi;
  ROS_INFO("default_noise_phi: %f", motion_model_default_options->gausianModel.minStdPHI);
  update_while_stopped = config.update_while_stopped;
  ROS_INFO("update_while_stopped: %s", update_while_stopped ? "true" : "false");
}
