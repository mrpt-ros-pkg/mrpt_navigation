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

#include <mrpt_localization/mrpt_localization.h>
#include <mrpt_localization/mrpt_localization_defaults.h>

// JLB: I really can't explain this, but if this header is not included here (though unneeded!)
// the behavior of the particle filter does not converge as expected (WTF!!!) (Verified with MRPT 1.0.2 & 1.3.0)
#define MRPT_NO_WARN_BIG_HDR
#include <mrpt/base.h>

#include <mrpt_bridge/map.h>

#include <mrpt/system/filesystem.h>
#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/opengl/CPointCloud.h>

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

#include <mrpt/version.h>
#if MRPT_VERSION>=0x130
  using namespace mrpt::maps;
  using namespace mrpt::obs;
#endif

PFLocalization::~PFLocalization()
{
}

PFLocalization::PFLocalization(Parameters *param) :
    PFLocalizationCore(), param_(param)
{
}

void PFLocalization::init()
{
  printf("iniFile ready %s\n", param_->iniFile.c_str());
  ASSERT_FILE_EXISTS_(param_->iniFile);
  printf("ASSERT_FILE_EXISTS_ %s\n", param_->iniFile.c_str());
  mrpt::utils::CConfigFile iniFile;
  iniFile.setFileName(param_->iniFile);
  printf("CConfigFile %s\n", param_->iniFile.c_str());

  vector_int particles_count;    // Number of initial particles (if size>1, run the experiments N times)

  // Load configuration:
  // -----------------------------------------
  string iniSectionName("LocalizationExperiment");
  update_counter_ = 0;

  // Mandatory entries:
  iniFile.read_vector(iniSectionName, "particles_count", vector_int(1, 0), particles_count, /*Fail if not found*/true);

  if (param_->mapFile.empty())
  {
    param_->mapFile = iniFile.read_string(iniSectionName, "map_file", "");
  }

  // Non-mandatory entries:
  SCENE3D_FREQ_ = iniFile.read_int(iniSectionName, "3DSceneFrequency", 10);
  SCENE3D_FOLLOW_ = iniFile.read_bool(iniSectionName, "3DSceneFollowRobot", true);

  SHOW_PROGRESS_3D_REAL_TIME_ = iniFile.read_bool(iniSectionName, "SHOW_PROGRESS_3D_REAL_TIME", false);
  SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS_ = iniFile.read_int(iniSectionName, "SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS", 1);

#if !MRPT_HAS_WXWIDGETS
  SHOW_PROGRESS_3D_REAL_TIME_ = false;
#endif

  // Default odometry uncertainty parameters in "odom_params_default_" depending on how fast the robot moves, etc...
  //  Only used for observations-only rawlogs:
  motion_model_default_options_.modelSelection = CActionRobotMovement2D::mmGaussian;
  motion_model_default_options_.gausianModel.minStdXY = iniFile.read_double("DummyOdometryParams", "minStdXY", 0.04);
  motion_model_default_options_.gausianModel.minStdPHI =
      DEG2RAD(iniFile.read_double("DefaultOdometryParams", "minStdPHI", 2.0));

  if (!iniFile.read_bool(iniSectionName, "init_PDF_mode", false, /*Fail if not found*/true))
  {
    init_PDF_min_x = iniFile.read_float(iniSectionName, "init_PDF_min_x", 0, true);
    init_PDF_max_x = iniFile.read_float(iniSectionName, "init_PDF_max_x", 0, true);
    init_PDF_min_y = iniFile.read_float(iniSectionName, "init_PDF_min_y", 0, true);
    init_PDF_max_y = iniFile.read_float(iniSectionName, "init_PDF_max_y", 0, true);
    float min_phi = DEG2RAD(iniFile.read_float(iniSectionName, "init_PDF_min_phi_deg", -180));
    float max_phi = DEG2RAD(iniFile.read_float(iniSectionName, "init_PDF_max_phi_deg", 180));
    mrpt::poses::CPose2D p;
    mrpt::math::CMatrixDouble33 cov;
    cov(0, 0) = fabs(init_PDF_max_x - init_PDF_min_x);
    cov(1, 1) = fabs(init_PDF_max_x - init_PDF_min_x);
    cov(2, 2) = min_phi < max_phi ? max_phi - min_phi : (max_phi + 2 * M_PI) - min_phi;
    p.x() = init_PDF_min_x + cov(0, 0) / 2.0;
    p.y() = init_PDF_min_y + cov(1, 1) / 2.0;
    p.phi() = min_phi + cov(2, 2) / 2.0;
    printf("----------- phi: %4.3f: %4.3f <-> %4.3f, %4.3f\n", p.phi(), min_phi, max_phi, cov(2, 2));
    initialPose_ = mrpt::poses::CPosePDFGaussian(p, cov);
    state_ = INIT;
  }
  else
  {
    log_error("no initial pose");
  }

  configureFilter(iniFile);
  // Metric map options:

  if (!mrpt_bridge::MapHdl::loadMap(metric_map_, iniFile, param_->mapFile, "metricMap", param_->debug))
  {
    waitForMap();
  }

  initialParticleCount_ = *particles_count.begin();

  if (param_->gui_mrpt)
    init3DDebug();

}

void PFLocalization::configureFilter(const mrpt::utils::CConfigFile &_configFile)
{
  // PF-algorithm Options:
  // ---------------------------
  CParticleFilter::TParticleFilterOptions pfOptions;
  pfOptions.loadFromConfigFile(_configFile, "PF_options");
  pfOptions.dumpToConsole();

  // PDF Options:
  // ------------------
  TMonteCarloLocalizationParams pdfPredictionOptions;
  pdfPredictionOptions.KLD_params.loadFromConfigFile(_configFile, "KLD_options");

  pdf_.clear();

  // PDF Options:
  pdf_.options = pdfPredictionOptions;

  pdf_.options.metricMap = &metric_map_;

  // Create the PF object:
  pf_.m_options = pfOptions;
}

void PFLocalization::init3DDebug()
{
  log_info("init3DDebug");
  if (!SHOW_PROGRESS_3D_REAL_TIME_)
    return;
  if (!win3D_)
  {
    win3D_ = CDisplayWindow3D::Create("pf-localization - The MRPT project", 1000, 600);
    win3D_->setCameraZoom(20);
    win3D_->setCameraAzimuthDeg(-45);
    //win3D_->waitForKey();

    // Create the 3D scene and get the map only once, later we'll modify only the particles, etc..
    COccupancyGridMap2D::TEntropyInfo gridInfo;
    // The gridmap:
    if (metric_map_.m_gridMaps.size())
    {
      metric_map_.m_gridMaps[0]->computeEntropy(gridInfo);
    }
    else
    {
      gridInfo.effectiveMappedArea = (init_PDF_max_x - init_PDF_min_x) * (init_PDF_max_y - init_PDF_min_y);
    }
    printf("The gridmap has %.04fm2 observed area, %u observed cells\n", gridInfo.effectiveMappedArea,
           (unsigned)gridInfo.effectiveMappedCells);
    printf("Initial PDF: %f particles/m2\n", initialParticleCount_ / gridInfo.effectiveMappedArea);

    CSetOfObjectsPtr plane = CSetOfObjects::Create();
    metric_map_.getAs3DObject(plane);
    scene_.insert(plane);

    if (SHOW_PROGRESS_3D_REAL_TIME_)
    {
      COpenGLScenePtr ptrScene = win3D_->get3DSceneAndLock();

      ptrScene->insert(plane);

      ptrScene->enableFollowCamera(true);

      win3D_->unlockAccess3DScene();
    }
  } // Show 3D?
  if (param_->debug)
    printf(" --------------------------- init3DDebug done \n");
  if (param_->debug)
    fflush(stdout);
}

void PFLocalization::show3DDebug(CSensoryFramePtr _observations)
{
  // Create 3D window if requested:
  if (SHOW_PROGRESS_3D_REAL_TIME_)
  {
    TTimeStamp cur_obs_timestamp;
    if (_observations->size() > 0)
      cur_obs_timestamp = _observations->getObservationByIndex(0)->timestamp;

    CPose2D meanPose;
    CMatrixDouble33 cov;
    pdf_.getCovarianceAndMean(cov, meanPose);

    COpenGLScenePtr ptrScene = win3D_->get3DSceneAndLock();

    win3D_->setCameraPointingToPoint(meanPose.x(), meanPose.y(), 0);
    win3D_->addTextMessage(
        10, 10, mrpt::format("timestamp: %s", mrpt::system::dateTimeLocalToString(cur_obs_timestamp).c_str()),
        mrpt::utils::TColorf(.8f, .8f, .8f), "mono", 15, mrpt::opengl::NICE, 6001);

    win3D_->addTextMessage(10, 33, mrpt::format("#particles= %7u", static_cast<unsigned int>(pdf_.size())),
                           mrpt::utils::TColorf(.8f, .8f, .8f), "mono", 15, mrpt::opengl::NICE, 6002);

    win3D_->addTextMessage(10, 55, mrpt::format("mean pose (x y phi_deg)= %s", meanPose.asString().c_str()),
                           mrpt::utils::TColorf(.8f, .8f, .8f), "mono", 15, mrpt::opengl::NICE, 6003);

    // The particles:
    {
      CRenderizablePtr parts = ptrScene->getByName("particles");
      if (parts)
        ptrScene->removeObject(parts);

      CSetOfObjectsPtr p = pdf_.getAs3DObject<CSetOfObjectsPtr>();
      p->setName("particles");
      ptrScene->insert(p);
    }

    // The particles' cov:
    {
      CRenderizablePtr ellip = ptrScene->getByName("parts_cov");
      if (!ellip)
      {
        ellip = CEllipsoid::Create();
        ellip->setName("parts_cov");
        ellip->setColor(1, 0, 0, 0.6);

        getAs<CEllipsoid>(ellip)->setLineWidth(2);
        getAs<CEllipsoid>(ellip)->setQuantiles(3);
        getAs<CEllipsoid>(ellip)->set2DsegmentsCount(60);
        ptrScene->insert(ellip);
      }
      ellip->setLocation(meanPose.x(), meanPose.y(), 0.05);

      getAs<CEllipsoid>(ellip)->setCovMatrix(cov, 2);
    }

    // The laser scan:
    {
      CRenderizablePtr scanPts = ptrScene->getByName("scan");
      if (!scanPts)
      {
        scanPts = CPointCloud::Create();
        scanPts->setName("scan");
        scanPts->setColor(1, 0, 0, 0.9);
        getAs<CPointCloud>(scanPts)->enableColorFromZ(false);
        getAs<CPointCloud>(scanPts)->setPointSize(4);
        ptrScene->insert(scanPts);
      }

      CSimplePointsMap map;
      static CSimplePointsMap last_map;

      CPose3D robotPose3D(meanPose);

      map.clear();
      _observations->insertObservationsInto(&map);

      getAs<CPointCloud>(scanPts)->loadFromPointsMap(&last_map);
      getAs<CPointCloud>(scanPts)->setPose(robotPose3D);
      last_map = map;
    }

    // The camera:
    ptrScene->enableFollowCamera(true);

    // Views:
    COpenGLViewportPtr view1 = ptrScene->getViewport("main");
    {
      CCamera &cam = view1->getCamera();
      cam.setAzimuthDegrees(-90);
      cam.setElevationDegrees(90);
      cam.setPointingAt(meanPose);
      cam.setZoomDistance(5);
      cam.setOrthogonal();
    }

    win3D_->unlockAccess3DScene();

    // Move camera:
    //win3D_->setCameraPointingToPoint( curRobotPose.x, curRobotPose.y, curRobotPose.z );

    // Update:
    win3D_->forceRepaint();

    sleep(SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS_);
  }
}
