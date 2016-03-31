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


#include <mrpt_slam/mrpt_slam.h>
#include <mrpt_slam/mrpt_slam_defaults.h>

// JLB: I really can't explain this, but if this header is not included here (though unneeded!)
// the behavior of the particle filter does not converge as expected (WTF!!!) (Verified with MRPT 1.0.2 & 1.3.0)
#define MRPT_NO_WARN_BIG_HDR
#include <mrpt/base.h>


#include <mrpt_bridge/map.h>

#include <mrpt/system/filesystem.h>
#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/random.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace mrpt::bayes;
using namespace mrpt::poses;
using namespace std;
using namespace mrpt::random;

#include <mrpt/version.h>
#if MRPT_VERSION>=0x130
using namespace mrpt::maps;
using namespace mrpt::obs;
#endif


RBPFSlam::~RBPFSlam()
{
}

RBPFSlam::RBPFSlam(Parameters *param)
    : RBPFSlamCore(), param_(param) {
}

void RBPFSlam::init() {
    printf("iniFile ready %s\n", param_->iniFile.c_str());
    ASSERT_FILE_EXISTS_(param_->iniFile);
    printf("ASSERT_FILE_EXISTS_ %s\n", param_->iniFile.c_str());
    mrpt::utils::CConfigFile iniFile;
    iniFile.setFileName(param_->iniFile);
    printf("CConfigFile %s\n", param_->iniFile.c_str());

    // Load configuration:
    // -----------------------------------------
    string iniSectionName ( "MappingApplication" );
    update_counter_ = 0;

    // Non-mandatory entries:
    SCENE3D_FREQ_        = iniFile.read_int(iniSectionName,"LOG_FREQUENCY",10);
    SCENE3D_FOLLOW_      = iniFile.read_bool(iniSectionName,"CAMERA_3DSCENE_FOLLOWS_ROBOT",true);

    SHOW_PROGRESS_3D_REAL_TIME_ = iniFile.read_bool(iniSectionName,"SHOW_PROGRESS_IN_WINDOW",false);
    SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS_ = iniFile.read_int(iniSectionName,"SHOW_PROGRESS_IN_WINDOW_DELAY_MS",1);

#if !MRPT_HAS_WXWIDGETS
    SHOW_PROGRESS_3D_REAL_TIME_ = false;
#endif

    // Default odometry uncertainty parameters in "odom_params_default_" depending on how fast the robot moves, etc...
    //  Only used for observations-only rawlogs:
    motion_model_default_options_.modelSelection = CActionRobotMovement2D::mmGaussian;
    motion_model_default_options_.gausianModel.minStdXY  = iniFile.read_double("DummyOdometryParams","minStdXY",0.04);
    motion_model_default_options_.gausianModel.minStdPHI = DEG2RAD(iniFile.read_double("DefaultOdometryParams","minStdPHI", 2.0));

	rbpfMappingOptions.loadFromConfigFile(iniFile,"MappingApplication");
	rbpfMappingOptions.dumpToConsole();

	mrpt::slam::CMetricMapBuilderRBPF mapBuilderInit(rbpfMappingOptions); 
	mapBuilderRBPF = mapBuilderInit;
    configureMapBuilder(iniFile);

    if(param_->gui_mrpt) init3DDebug();

}

void RBPFSlam::configureMapBuilder(const mrpt::utils::CConfigFile &_configFile) {
	mapBuilderRBPF.options.verbose					= true;
	mapBuilderRBPF.options.enableMapUpdating		= true;
    mapBuilderRBPF.options.debugForceInsertion		= false;

	randomGenerator.randomize();

}

void RBPFSlam::init3DDebug() {
    log_info("init3DDebug");
	
    if (!SHOW_PROGRESS_3D_REAL_TIME_) return;
    if(!win3D_) {
	
        win3D_ = CDisplayWindow3D::Create("rbpf-slam - The MRPT project", 1000, 600);
        win3D_->setCameraZoom(40);
        win3D_->setCameraAzimuthDeg(-50);
		win3D_->setCameraElevationDeg(70);
        //win3D_->waitForKey();

        // Create the 3D scene and get the map only once, later we'll modify only the particles, etc..
    }
    if(param_->debug) printf(" --------------------------- init3DDebug done \n");
    if(param_->debug) fflush(stdout);
}

void RBPFSlam::show3DDebug(CSensoryFramePtr _observations) {
    // Create 3D window if requested:
    if (SHOW_PROGRESS_3D_REAL_TIME_)
    {
		
        TTimeStamp cur_obs_timestamp;
        if (_observations->size()>0)
            cur_obs_timestamp = _observations->getObservationByIndex(0)->timestamp;
		
		tictac_.Tic();	
		scene_ = COpenGLScene::Create();
       	
		COpenGLScenePtr & ptrScene = win3D_->get3DSceneAndLock();
		ptrScene = scene_;
		curPDFptr = mapBuilderRBPF.getCurrentPoseEstimation();
		
		if ( IS_CLASS( curPDFptr, CPose3DPDFParticles ) )
		{
			CPose3DPDFParticlesPtr pp= CPose3DPDFParticlesPtr(curPDFptr);
			curPDF = *pp;
		}
		
		if (SCENE3D_FOLLOW_) {
			mrpt::opengl::CCameraPtr objCam = mrpt::opengl::CCamera::Create();
			CPose3D		robotPose;
			curPDF.getMean(robotPose);

			objCam->setPointingAt(robotPose);
			objCam->setAzimuthDegrees(-30);
			objCam->setElevationDegrees(30);
			scene_->insert( objCam );
		}
		
		if(objs)	scene_->insert( objs ); // adds the beacon particles
		
		size_t		M = mapBuilderRBPF.mapPDF.particlesCount(); // number of robot particles, set by ini file
		
		mrpt::opengl::CSetOfLinesPtr objLines = mrpt::opengl::CSetOfLines::Create();
		objLines->setColor(0,1,1);

		for (size_t i=0;i<M;i++)
		{
			std::deque<TPose3D>		path;
			mapBuilderRBPF.mapPDF.getPath(i,path);

			float	x0=0,y0=0,z0=0;
			for (size_t k=0;k<path.size();k++)
			{
				objLines->appendLine(
					x0, y0, z0+0.001,
					path[k].x, path[k].y, path[k].z+0.001 ); 
				x0=path[k].x;
				y0=path[k].y;
				z0=path[k].z;
			}
		}
		scene_->insert( objLines ); // adds the robot path as lines
		CPose3D			lastMeanPose;
		float			minDistBtwPoses=-1;
		std::deque<TPose3D>		dummyPath;
		mapBuilderRBPF.mapPDF.getPath(0,dummyPath);
		
		for (int k=(int)dummyPath.size()-1;k>=0;k--)
		{
			CPose3DPDFParticles	poseParts;
			mapBuilderRBPF.mapPDF.getEstimatedPosePDFAtTime(k,poseParts);

			CPose3D		meanPose;
			CMatrixDouble66 COV;
			poseParts.getCovarianceAndMean(COV,meanPose);

			if ( meanPose.distanceTo(lastMeanPose)>minDistBtwPoses )
			{
				CMatrixDouble33 COV3 = COV.block(0,0,3,3);

				minDistBtwPoses = 6 * sqrt(COV3(0,0)+COV3(1,1));

				opengl::CEllipsoidPtr objEllip = opengl::CEllipsoid::Create();
				objEllip->setLocation(meanPose.x(), meanPose.y(), meanPose.z() + 0.001 );
				objEllip->setCovMatrix(COV3, COV3(2,2)==0 ? 2:3 );

				objEllip->setColor(0,0,1);
				objEllip->enableDrawSolid3D(false);
				scene_->insert( objEllip ); //inserts the average position of the robot as an ellipse

				lastMeanPose = meanPose;
			}
		}

        win3D_->unlockAccess3DScene();

        // Move camera:
        //win3D_->setCameraPointingToPoint( curRobotPose.x, curRobotPose.y, curRobotPose.z );

        // Update:
        win3D_->forceRepaint();
		int add_delay = SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS_ - t_exec*1000;
		if (add_delay>0)
			sleep(add_delay);
    }
}

