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
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/maps/CMultiMetricMap.h>
using namespace mrpt::maps;
using namespace mrpt::obs;

class PFLocalizationCore
{
	MRPT_VIRTUAL_LOG_MACROS

   public:
	enum PFStates
	{
		NA,
		INIT,
		RUN,
		IDLE
	};

	PFLocalizationCore();
	~PFLocalizationCore();

	/**
	 * Initializes the parameter with common values to acive a working filter
	 *out of the box
	 **/
	void init();
	/**
	 * preprocesses an observation and calls the update
	 *PFLocalizationCore::updateFilter
	 * If the odom data is null the function will assume a dummy odometry
	 *distribution around the last pose
	 * @param _sf sensor observation
	 * @param _odometry the odom data can also be NULL
	 **/
	void observation(
		CSensoryFrame::Ptr _sf, CObservationOdometry::Ptr _odometry);

   protected:
	bool use_motion_model_default_options_;  ///< used default odom_params
	CActionRobotMovement2D::TMotionModelOptions
		motion_model_default_options_;  ///< used if there are is not odom
	CActionRobotMovement2D::TMotionModelOptions
		motion_model_options_;  ///< used with odom value motion noise
	CMultiMetricMap metric_map_;  ///< map
	mrpt::bayes::CParticleFilter
		pf_;  ///< common interface for particle filters
	mrpt::bayes::CParticleFilter::TParticleFilterStats
		pf_stats_;  ///< filter statistics
	mrpt::slam::CMonteCarloLocalization2D pdf_;  ///< the filter
	mrpt::poses::CPosePDFGaussian
		initial_pose_;  ///< initial posed used in initializeFilter()
	int initial_particle_count_;  ///< number of particles for initialization
	mrpt::system::TTimeStamp time_last_update_;  ///< time of the last update
	mrpt::utils::CTicTac tictac_;  ///< timer to measure performance
	size_t update_counter_;  ///< internal counter to count the number of filter
	/// updates
	PFStates state_;  ///< filter states to perform things like init on the
	/// correct time
	mrpt::poses::CPose2D
		odom_last_observation_;  ///< pose at the last observation
	bool init_PDF_mode;  ///< Initial PDF mode: 0 for free space cells, 1 for
	/// any cell
	float init_PDF_min_x;  ///< Initial PDF boundaries
	float init_PDF_max_x;
	float init_PDF_min_y;
	float init_PDF_max_y;

   private:
	/**
	 * Initializes the filter at pose PFLocalizationCore::initial_pose_ with
	 *PFLocalizationCore::initial_particle_count_
	 * it is called by the PFLocalizationCore::updateFilter if the state_ ==
	 *INIT
	 **/
	void initializeFilter();

	void updateFilter(CActionCollection::Ptr _action, CSensoryFrame::Ptr _sf);
};

#endif  // MRPT_LOCALIZATION_CORE_H
