/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2023, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/core/Clock.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/slam/CMonteCarloLocalization2D.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTicTac.h>
#include <stdint.h>

#include <iostream>

class PFLocalizationCore : public mrpt::system::COutputLogger
{
   public:
	enum PFStates
	{
		NA,
		INIT,
		RUN,
		IDLE
	};

	PFLocalizationCore() = default;
	~PFLocalizationCore() = default;

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
		mrpt::obs::CSensoryFrame::Ptr _sf,
		mrpt::obs::CObservationOdometry::Ptr _odometry);

   protected:
	bool use_motion_model_default_options_;	 ///< used default odom_params
	mrpt::obs::CActionRobotMovement2D::TMotionModelOptions
		motion_model_default_options_;	///< used if there are is not odom
	mrpt::obs::CActionRobotMovement2D::TMotionModelOptions
		motion_model_options_;	///< used with odom value motion noise
	mrpt::maps::CMultiMetricMap::Ptr metric_map_ =
		mrpt::maps::CMultiMetricMap::Create();
	mrpt::bayes::CParticleFilter pf_;  ///< interface for particle filters
	mrpt::bayes::CParticleFilter::TParticleFilterStats pf_stats_;
	mrpt::slam::CMonteCarloLocalization2D pdf_;	 ///< the filter

	/// initial posed used in initializeFilter()
	mrpt::poses::CPosePDFGaussian initial_pose_;
	/// number of particles for initialization
	int initial_particle_count_;

	mrpt::Clock::time_point time_last_update_;	///< time of the last update
	mrpt::system::CTicTac tictac_;	///< timer to measure performance

	/// internal counter to count the number of filter updates
	size_t update_counter_;

	/// filter states to perform things like init on the correct time
	PFStates state_ = NA;

	/// pose at the last observation
	mrpt::poses::CPose2D odom_last_observation_;

	/// Initial PDF mode: 0 for free space cells, 1 for any cell
	bool init_PDF_mode;
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

	void updateFilter(
		mrpt::obs::CActionCollection::Ptr _action,
		mrpt::obs::CSensoryFrame::Ptr _sf);
};
