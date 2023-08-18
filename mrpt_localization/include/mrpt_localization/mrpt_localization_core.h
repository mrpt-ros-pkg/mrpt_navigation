/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2023, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/slam/CMonteCarloLocalization2D.h>
#include <mrpt/system/CTicTac.h>
#include <stdint.h>

#include <iostream>
using namespace mrpt::maps;
using namespace mrpt::obs;

class PFLocalizationCore
{
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
	bool use_motion_model_default_options_;	 ///< used default odom_params
	CActionRobotMovement2D::TMotionModelOptions
		motion_model_default_options_;	///< used if there are is not odom
	CActionRobotMovement2D::TMotionModelOptions
		motion_model_options_;	///< used with odom value motion noise
	CMultiMetricMap::Ptr metric_map_ = CMultiMetricMap::Create();  ///< map
	mrpt::bayes::CParticleFilter
		pf_;  ///< common interface for particle filters
	mrpt::bayes::CParticleFilter::TParticleFilterStats
		pf_stats_;	///< filter statistics
	mrpt::slam::CMonteCarloLocalization2D pdf_;	 ///< the filter
	mrpt::poses::CPosePDFGaussian
		initial_pose_;	///< initial posed used in initializeFilter()
	int initial_particle_count_;  ///< number of particles for initialization
	mrpt::system::TTimeStamp time_last_update_;	 ///< time of the last update
	mrpt::system::CTicTac tictac_;	///< timer to measure performance
	size_t update_counter_;	 ///< internal counter to count the number of filter
	/// updates
	PFStates state_;  ///< filter states to perform things like init on the
	/// correct time
	mrpt::poses::CPose2D
		odom_last_observation_;	 ///< pose at the last observation
	bool init_PDF_mode;	 ///< Initial PDF mode: 0 for free space cells, 1 for
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
