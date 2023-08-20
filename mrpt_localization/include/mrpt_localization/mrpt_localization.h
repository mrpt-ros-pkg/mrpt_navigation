/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2023, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/config/CConfigFile.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt_localization/mrpt_localization_core.h>
#include <stdint.h>

#include <iostream>

class PFLocalization : public PFLocalizationCore
{
   public:
	struct Parameters
	{
		Parameters(PFLocalization* p);
		bool debug;
		bool gui_mrpt;
		std::string ini_file;
		std::string map_file;
		std::string sensor_sources;	 //!< A list of topics (e.g. laser scanners)
		//! to subscribe to for sensory data. Split
		//! with "," (e.g. "laser1,laser2")
		bool* use_motion_model_default_options;
		mrpt::obs::CActionRobotMovement2D::TMotionModelOptions* motion_model_options;
		mrpt::obs::CActionRobotMovement2D::TMotionModelOptions*
			motion_model_default_options;
	};
	PFLocalization(Parameters* parm);
	virtual ~PFLocalization();

   protected:
	Parameters* param_;
	void init();
	void init3DDebug();
	void show3DDebug(mrpt::obs::CSensoryFrame::Ptr _observations);
	void configureFilter(const mrpt::config::CConfigFile& _configFile);
	virtual bool waitForMap() { return false; }
	mrpt::gui::CDisplayWindow3D::Ptr win3D_;
	mrpt::opengl::COpenGLScene scene_;

	int SCENE3D_FREQ_;
	bool SCENE3D_FOLLOW_;
	bool SHOW_PROGRESS_3D_REAL_TIME_;
	int SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS_;
};
