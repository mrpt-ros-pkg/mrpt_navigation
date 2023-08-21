/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2023, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include "mrpt_localization_node_defaults.h"
#include "mrpt_pf_localization_node.h"

PFLocalizationNode::Parameters::Parameters(PFLocalizationNode* p)
	: PFLocalization::Parameters(p)
{
	node.param<double>("transform_tolerance", transform_tolerance, 0.1);
	MRPT_LOG_INFO_FMT("transform_tolerance: %f", transform_tolerance);
	node.param<double>("no_update_tolerance", no_update_tolerance, 1.0);
	MRPT_LOG_INFO_FMT("no_update_tolerance: %f", no_update_tolerance);
	node.param<double>(
		"no_inputs_tolerance", no_inputs_tolerance,
		std::numeric_limits<double>::infinity());
	MRPT_LOG_INFO_FMT(
		"no_inputs_tolerance: %f", no_inputs_tolerance);  // disabled by default
	node.param<double>("rate", rate, MRPT_LOCALIZATION_NODE_DEFAULT_RATE);
	MRPT_LOG_INFO_FMT("rate: %f", rate);
	node.getParam("gui_mrpt", gui_mrpt);
	MRPT_LOG_INFO_FMT("gui_mrpt: %s", gui_mrpt ? "true" : "false");
	node.param<int>(
		"parameter_update_skip", parameter_update_skip,
		MRPT_LOCALIZATION_NODE_DEFAULT_PARAMETER_UPDATE_SKIP);
	MRPT_LOG_INFO_FMT("parameter_update_skip: %i", parameter_update_skip);
	node.getParam("ini_file", ini_file);
	MRPT_LOG_INFO_FMT("ini_file: %s", ini_file.c_str());
	node.getParam("map_file", map_file);
	MRPT_LOG_INFO_FMT("map_file: %s", map_file.c_str());
	node.getParam("sensor_sources", sensor_sources);
	MRPT_LOG_INFO_FMT("sensor_sources: %s", sensor_sources.c_str());
	node.param<std::string>("global_frame_id", global_frame_id, "map");
	MRPT_LOG_INFO_FMT("global_frame_id: %s", global_frame_id.c_str());
	node.param<std::string>("odom_frame_id", odom_frame_id, "odom");
	MRPT_LOG_INFO_FMT("odom_frame_id: %s", odom_frame_id.c_str());
	node.param<std::string>("base_frame_id", base_frame_id, "base_link");
	MRPT_LOG_INFO_FMT("base_frame_id: %s", base_frame_id.c_str());
	node.param<bool>("pose_broadcast", pose_broadcast, false);
	MRPT_LOG_INFO_FMT("pose_broadcast: %s", pose_broadcast ? "true" : "false");
	node.param<bool>("tf_broadcast", tf_broadcast, true);
	MRPT_LOG_INFO_FMT("tf_broadcast: %s", tf_broadcast ? "true" : "false");
	node.param<bool>("use_map_topic", use_map_topic, false);
	MRPT_LOG_INFO_FMT("use_map_topic: %s", use_map_topic ? "true" : "false");
	node.param<bool>("first_map_only", first_map_only, false);
	MRPT_LOG_INFO_FMT("first_map_only: %s", first_map_only ? "true" : "false");
	node.param<bool>("debug", debug, true);
	MRPT_LOG_INFO_FMT("debug: %s", debug ? "true" : "false");

	reconfigure_cb_ = boost::bind(
		&PFLocalizationNode::Parameters::callbackParameters, this, _1, _2);
	reconfigure_server_.setCallback(reconfigure_cb_);
}

void PFLocalizationNode::Parameters::update(const unsigned long& loop_count)
{
	if (loop_count % parameter_update_skip) return;
	node.getParam("debug", debug);
	if (loop_count == 0)
		MRPT_LOG_INFO_FMT("debug: %s", debug ? "true" : "false");
	{
		int v = particlecloud_update_skip;
		node.param<int>(
			"particlecloud_update_skip", particlecloud_update_skip,
			MRPT_LOCALIZATION_NODE_DEFAULT_PARTICLECLOUD_UPDATE_SKIP);
		if (v != particlecloud_update_skip)
			MRPT_LOG_INFO_FMT(
				"particlecloud_update_skip: %i", particlecloud_update_skip);
	}
	{
		int v = map_update_skip;
		node.param<int>(
			"map_update_skip", map_update_skip,
			MRPT_LOCALIZATION_NODE_DEFAULT_MAP_UPDATE_SKIP);
		if (v != map_update_skip)
			MRPT_LOG_INFO_FMT("map_update_skip: %i", map_update_skip);
	}
}

void PFLocalizationNode::Parameters::callbackParameters(
	mrpt_pf_localization::MotionConfig& config, uint32_t level)
{
	if (config.motion_noise_type == MOTION_MODEL_GAUSSIAN)
	{
		motion_model_options->modelSelection =
			CActionRobotMovement2D::mmGaussian;

		motion_model_options->gaussianModel.a1 = config.gaussian_alpha_1;
		motion_model_options->gaussianModel.a2 = config.gaussian_alpha_2;
		motion_model_options->gaussianModel.a3 = config.gaussian_alpha_3;
		motion_model_options->gaussianModel.a4 = config.gaussian_alpha_4;
		motion_model_options->gaussianModel.minStdXY = config.gaussian_alpha_xy;
		motion_model_options->gaussianModel.minStdPHI =
			config.gaussian_alpha_phi;
		MRPT_LOG_INFO_FMT("gaussianModel.type: gaussian");
		MRPT_LOG_INFO_FMT(
			"gaussianModel.a1: %f", motion_model_options->gaussianModel.a1);
		MRPT_LOG_INFO_FMT(
			"gaussianModel.a2: %f", motion_model_options->gaussianModel.a2);
		MRPT_LOG_INFO_FMT(
			"gaussianModel.a3: %f", motion_model_options->gaussianModel.a3);
		MRPT_LOG_INFO_FMT(
			"gaussianModel.a4: %f", motion_model_options->gaussianModel.a4);
		MRPT_LOG_INFO_FMT(
			"gaussianModel.minStdXY: %f",
			motion_model_options->gaussianModel.minStdXY);
		MRPT_LOG_INFO_FMT(
			"gaussianModel.minStdPHI: %f",
			motion_model_options->gaussianModel.minStdPHI);
	}
	else
	{
		MRPT_LOG_INFO_FMT(
			"We support at the moment only gaussian motion models");
	}
	*use_motion_model_default_options = config.use_default_motion;
	MRPT_LOG_INFO_FMT(
		"use_motion_model_default_options: %s",
		use_motion_model_default_options ? "true" : "false");
	motion_model_default_options->gaussianModel.minStdXY =
		config.default_noise_xy;
	MRPT_LOG_INFO_FMT(
		"default_noise_xy: %f",
		motion_model_default_options->gaussianModel.minStdXY);
	motion_model_default_options->gaussianModel.minStdPHI =
		config.default_noise_phi;
	MRPT_LOG_INFO_FMT(
		"default_noise_phi: %f",
		motion_model_default_options->gaussianModel.minStdPHI);
	update_while_stopped = config.update_while_stopped;
	MRPT_LOG_INFO_FMT(
		"update_while_stopped: %s", update_while_stopped ? "true" : "false");
	update_sensor_pose = config.update_sensor_pose;
	MRPT_LOG_INFO_FMT(
		"update_sensor_pose: %s", update_sensor_pose ? "true" : "false");
}
