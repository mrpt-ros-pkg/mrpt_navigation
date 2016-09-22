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

#ifndef MRPT_LOCALIZATION_CORE_H
#define MRPT_LOCALIZATION_CORE_H

#include <iostream>
#include <stdint.h>

#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/slam/CMonteCarloLocalization2D.h>
#include <mrpt_bridge/mrpt_log_macros.h>

#include <mrpt/version.h>
#if MRPT_VERSION>=0x130
  #include <mrpt/obs/CActionRobotMovement2D.h>
  #include <mrpt/obs/CActionCollection.h>
  #include <mrpt/obs/CObservationOdometry.h>
  #include <mrpt/obs/CSensoryFrame.h>
  #include <mrpt/maps/CMultiMetricMap.h>
  using namespace mrpt::maps;
  using namespace mrpt::obs;
#else
  #include <mrpt/slam/CActionRobotMovement2D.h>
  #include <mrpt/slam/CActionCollection.h>
  #include <mrpt/slam/CObservationOdometry.h>
  #include <mrpt/slam/CSensoryFrame.h>
  #include <mrpt/slam/CMultiMetricMap.h>
  using namespace mrpt::slam;
#endif

class PFLocalizationCore
{
  MRPT_VIRTUAL_LOG_MACROS

public:
  enum PFStates
  {
    NA, INIT, RUN
  };

  PFLocalizationCore();
  ~PFLocalizationCore();

  /**
   * Initializes the parameter with common values to acive a working filter out of the box
   **/
  void init();
  /**
   * preprocesses an observation and calls the update PFLocalizationCore::updateFilter
   * If the odom data is null the function will assume a dummy odometry distribution around the last pose
   * @param _sf sensor observation
   * @param _odometry the odom data can also be NULL
   **/
  void observation(CSensoryFramePtr _sf, CObservationOdometryPtr _odometry);
protected:

  bool use_motion_model_default_options_; /// used default odom_params
  CActionRobotMovement2D::TMotionModelOptions motion_model_default_options_; /// used if there are is not odom
  CActionRobotMovement2D::TMotionModelOptions motion_model_options_;         /// used with odom value motion noise
  CMultiMetricMap metric_map_;                 /// map
  mrpt::bayes::CParticleFilter pf_;            /// common interface for particle filters
  mrpt::bayes::CParticleFilter::TParticleFilterStats pf_stats_; /// filter statistics
  mrpt::slam::CMonteCarloLocalization2D pdf_;  /// the filter
  mrpt::poses::CPosePDFGaussian initialPose_;  /// initial posed used in initializeFilter()
  int initialParticleCount_;                   /// number of particles for initialization
  mrpt::system::TTimeStamp timeLastUpdate_;    /// time of the last update
  mrpt::utils::CTicTac tictac_;                /// timer to measure performance
  size_t update_counter_;                      /// internal counter to count the number of filter updates
  PFStates state_;                             /// filter states to perform things like init on the corret time
  mrpt::poses::CPose2D odomLastObservation_;   /// pose at the last overvation

private:
  /**
   * Initilizes the filter at pose PFLocalizationCore::initialPose_ with PFLocalizationCore::initialParticleCount_
   * it is called by the PFLocalizationCore::updateFilter if the state_ == INIT
   **/
  void initializeFilter();

  void updateFilter(CActionCollectionPtr _action, CSensoryFramePtr _sf);
};

#endif // MRPT_LOCALIZATION_CORE_H

