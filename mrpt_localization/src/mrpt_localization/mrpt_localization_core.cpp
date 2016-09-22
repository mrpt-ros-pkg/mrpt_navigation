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

#include <mrpt_localization/mrpt_localization_core.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace std;

PFLocalizationCore::~PFLocalizationCore()
{
}

PFLocalizationCore::PFLocalizationCore() :
    state_(NA)
{
}

void PFLocalizationCore::init()
{
  mrpt::math::CMatrixDouble33 cov;
  cov(0, 0) = 1, cov(1, 1) = 1, cov(2, 2) = 2 * M_PI;
  initialPose_ = mrpt::poses::CPosePDFGaussian(mrpt::poses::CPose2D(0, 0, 0), cov);
  initialParticleCount_ = 1000;
  motion_model_default_options_.modelSelection = CActionRobotMovement2D::mmGaussian;
  motion_model_default_options_.gausianModel.minStdXY = 0.10;
  motion_model_default_options_.gausianModel.minStdPHI = 2.0;
}

void PFLocalizationCore::initializeFilter()
{
  mrpt::math::CMatrixDouble33 cov;
  mrpt::poses::CPose2D mean_point;
  log_info("InitializeFilter: %4.3fm, %4.3fm, %4.3frad ", mean_point.x(), mean_point.y(), mean_point.phi());
  initialPose_.getCovarianceAndMean(cov, mean_point);
  float min_x = mean_point.x() - cov(0, 0);
  float max_x = mean_point.x() + cov(0, 0);
  float min_y = mean_point.y() - cov(1, 1);
  float max_y = mean_point.y() + cov(1, 1);
  float min_phi = mean_point.phi() - cov(2, 2);
  float max_phi = mean_point.phi() + cov(2, 2);
  if (metric_map_.m_gridMaps.size())
  {
    pdf_.resetUniformFreeSpace(metric_map_.m_gridMaps[0].pointer(), 0.7f, initialParticleCount_, min_x, max_x, min_y,
                               max_y, min_phi, max_phi);
  }
  else if (metric_map_.m_landmarksMap)
  {
    pdf_.resetUniform(min_x, max_x, min_y, max_y, min_phi, max_phi, initialParticleCount_);
  }
  state_ = RUN;
}

void PFLocalizationCore::updateFilter(CActionCollectionPtr _action, CSensoryFramePtr _sf)
{
  if (state_ == INIT)
    initializeFilter();
  tictac_.Tic();
  pf_.executeOn(pdf_, _action.pointer(), _sf.pointer(), &pf_stats_);
  timeLastUpdate_ = _sf->getObservationByIndex(0)->timestamp;
  update_counter_++;
}

void PFLocalizationCore::observation(CSensoryFramePtr _sf, CObservationOdometryPtr _odometry)
{
  CActionCollectionPtr action = CActionCollection::Create();
  CActionRobotMovement2D odom_move;
  odom_move.timestamp = _sf->getObservationByIndex(0)->timestamp;
  if (_odometry)
  {
    if (odomLastObservation_.empty())
    {
      odomLastObservation_ = _odometry->odometry;
    }
    mrpt::poses::CPose2D incOdoPose = _odometry->odometry - odomLastObservation_;
    odomLastObservation_ = _odometry->odometry;
    odom_move.computeFromOdometry(incOdoPose, motion_model_options_);
    action->insert(odom_move);
    updateFilter(action, _sf);
  }
  else
  {
    if (use_motion_model_default_options_)
    {
      log_info("No odometry at update %4i -> using dummy", update_counter_);
      odom_move.computeFromOdometry(mrpt::poses::CPose2D(0, 0, 0), motion_model_default_options_);
      action->insert(odom_move);
      updateFilter(action, _sf);
    }
    else
    {
      log_info("No odometry at update %4i -> skipping observation", update_counter_);
    }
  }
}
