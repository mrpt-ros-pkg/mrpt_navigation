/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <cstdint>
#include <iostream>

class RawlogPlay
{
   public:
	struct Parameters
	{
		Parameters();
		bool debug;
		std::string rawlog_file;
		mrpt::obs::CActionRobotMovement2D::TMotionModelOptions
			motionModelOptions;
	};

	RawlogPlay(Parameters* parm);
	~RawlogPlay();

   protected:
	Parameters* param_;
	mrpt::io::CFileGZInputStream rawlog_stream_;
	mrpt::poses::CPose3DPDFGaussian robotPose;
	size_t entry_ = 0;
};
