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

#include "rawlog_play_node.h"
#include "rawlog_play_node_defaults.h"

RawlogPlayNode::ParametersNode::ParametersNode() : Parameters(), node("~")
{
	node.param<double>("rate", rate, RAWLOG_PLAY_NODE_DEFAULT_RATE);
	ROS_INFO("rate: %f", rate);
	node.param<int>(
		"parameter_update_skip", parameter_update_skip,
		RAWLOG_PLAY_NODE_DEFAULT_PARAMETER_UPDATE_SKIP);
	ROS_INFO("parameter_update_skip: %i", parameter_update_skip);
	node.getParam("rawlog_file", rawlog_file);
	ROS_INFO("rawlog_file: %s", rawlog_file.c_str());
	node.param<std::string>("odom_frame", odom_frame, "odom");
	ROS_INFO("odom_frame: %s", odom_frame.c_str());
	node.param<std::string>("base_frame", base_frame, "base_link");
	ROS_INFO("base_frame: %s", base_frame.c_str());
	node.param<std::string>("tf_prefix", tf_prefix, "");
	ROS_INFO("tf_prefix: %s", tf_prefix.c_str());
	reconfigureFnc_ = boost::bind(
		&RawlogPlayNode::ParametersNode::callbackParameters, this, _1, _2);
	reconfigureServer_.setCallback(reconfigureFnc_);
}

void RawlogPlayNode::ParametersNode::update(const unsigned long& loop_count)
{
	if (loop_count % parameter_update_skip) return;
	node.getParam("debug", debug);
	if (loop_count == 0) ROS_INFO("debug:  %s", (debug ? "true" : "false"));
}

void RawlogPlayNode::ParametersNode::callbackParameters(
	mrpt_rawlog::RawLogRecordConfig& config, uint32_t level)
{
	if (config.motion_noise_type == MOTION_MODEL_GAUSSIAN)
	{
		motionModelOptions.modelSelection = CActionRobotMovement2D::mmGaussian;
		motionModelOptions.gaussianModel.a1 = config.motion_gaussian_alpha_1;
		motionModelOptions.gaussianModel.a2 = config.motion_gaussian_alpha_2;
		motionModelOptions.gaussianModel.a3 = config.motion_gaussian_alpha_3;
		motionModelOptions.gaussianModel.a4 = config.motion_gaussian_alpha_4;
		motionModelOptions.gaussianModel.minStdXY = config.motion_gaussian_alpha_xy;
		motionModelOptions.gaussianModel.minStdPHI = config.motion_gaussian_alpha_phi;
		ROS_INFO("gaussianModel.a1: %f", motionModelOptions.gaussianModel.a1);
		ROS_INFO("gaussianModel.a2: %f", motionModelOptions.gaussianModel.a2);
		ROS_INFO("gaussianModel.a3: %f", motionModelOptions.gaussianModel.a3);
		ROS_INFO("gaussianModel.a4: %f", motionModelOptions.gaussianModel.a4);
		ROS_INFO(
			"gaussianModel.minStdXY: %f",
			motionModelOptions.gaussianModel.minStdXY);
		ROS_INFO(
			"gaussianModel.minStdPHI: %f",
			motionModelOptions.gaussianModel.minStdPHI);
	}
}
