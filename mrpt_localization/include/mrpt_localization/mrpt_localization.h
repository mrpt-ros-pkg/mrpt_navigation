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

#include <iostream>
#include <stdint.h>
#include <mrpt/base.h>
#include <mrpt/slam.h>
#include <mrpt/gui.h>

#ifndef MRPT_LOCALIZATION_H
#define MRPT_LOCALIZATION_H


class PFLocalization {
public:
	struct Parameters{
		Parameters();
	    bool debug;
        std::string iniFile;
        std::string rawlogFile;
        std::string mapFile;
        mrpt::slam::CActionRobotMovement2D::TMotionModelOptions motionModelOptions;
	};
    PFLocalization (Parameters *parm);
    ~PFLocalization();
protected:
    Parameters *param_; 
    mrpt::slam::CRawlog *pRawLog;
    void init();
    void update();
    void summary();
    void incommingLaserData(mrpt::slam::CObservation2DRangeScanPtr  laser);
    void incommingOdomData( mrpt::slam::CObservationOdometryPtr odometry);
    mrpt::utils::CConfigFile iniFile_;


    mrpt::utils::CTicTac tictac_;
    mrpt::utils::CTicTac tictacGlobal_;
    mrpt::slam::CMultiMetricMap metricMap_;
    mrpt::gui::CDisplayWindow3DPtr win3D_;
    mrpt::opengl::COpenGLScene scene_;
    mrpt::slam::COccupancyGridMap2D::TEntropyInfo gridInfo_;

    mrpt::utils::CPose2D pdfEstimation_;
    mrpt::utils::CPose2D odometryEstimation_;
    mrpt::utils::CMatrixDouble covEstimation_;
    mrpt::slam::CMonteCarloLocalization2D  pdf_;
};

#endif // MRPT_LOCALIZATION_H
