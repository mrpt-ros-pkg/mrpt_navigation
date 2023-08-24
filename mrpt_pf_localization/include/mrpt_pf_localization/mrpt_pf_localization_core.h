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
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/slam/CMonteCarloLocalization2D.h>
#include <mrpt/slam/CMonteCarloLocalization3D.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>

#include <mutex>
#include <optional>

/**
 * The core C++ non-ROS part of the particle filter localization algorithm.
 * It features a 2D SE(2) and 3D SE(3) particle filter, all in one, depending on
 * parameters.
 *
 */
class PFLocalizationCore : public mrpt::system::COutputLogger
{
   public:
	PFLocalizationCore() = default;
	virtual ~PFLocalizationCore() = default;

	/** Parameters the filter will use to initialize or to run.
	 *  The ROS node will overwrite here when reading params from YAML file,
	 *  CLI, or service-based changes.
	 */
	struct Parameters
	{
		Parameters();
		~Parameters() = default;

		/** Shows a custom MRPT GUI with the PF and map state
		 *  Can be changed at any moment.
		 */
		bool gui_enable = true;

		/** If false (default), will use 2D (SE(2)) particle filter. Otherwise,
		 * the 3D mode (SE(3)) is enabled.
		 * It is read upon filter initialization.
		 */
		bool use_se3_pf = false;

		/** If gui_enable==true, makes the camera to follow the mean of the
		 * particles.
		 * Can be changed at any moment.
		 */
		bool gui_camera_follow_robot = true;

		/** Uncertainty motion model for regular odometry-based motion:
		 * Can be changed at any moment.
		 */
		mrpt::obs::CActionRobotMovement2D::TMotionModelOptions
			motion_model_options;

		/** Uncertainty motion model to use when NO odometry has been received
		 * Can be changed at any moment.
		 */
		mrpt::obs::CActionRobotMovement2D::TMotionModelOptions
			motion_model_default_options;

		/** initial pose used to intialize the filter */
		mrpt::poses::CPose3DPDFGaussian initial_pose;

		/** All the PF parameters: algorithm, number of samples, dynamic
		 * samples, etc.
		 * It is read upon filter initialization.
		 */
		mrpt::bayes::CParticleFilter::TParticleFilterOptions pf_options;

		/** Dynamic sampling-related parameters.
		 * Can be changed at any moment.
		 */
		mrpt::slam::TKLDParams kld_options;
	};

	/// The state of the particle filter. The "loop()" method will look at this
	/// to know what to do.
	enum class State : uint8_t
	{
		/// The filter has been neither initialized nor parameters/map loaded.
		UNINITIALIZED,
		/// Next iteration must create samples at the initial pose PDF. All
		/// parameters and map(s) are correctly loaded.
		TO_BE_INITIALIZED,
		/// Running as usual, the robot is moving.
		RUNNING_MOVING,
		/// Special case: the robot is known to be stopped, do not update the
		/// particles.
		RUNNING_STILL
	};

	/** Must be called for each new observation that arrives from the robot:
	 *  odometry, 2D or 3D lidar, GPS, etc.
	 */
	void on_observation(const mrpt::obs::CObservation::Ptr& obs);

	/** The main API call: executes one PF step, taking into account all the
	 * parameters and observations gathered so far, updates the optional GUI,
	 * etc.
	 *
	 * Must be called at a timely fashion. Upon return, the PF state can be
	 * grabbed via the public methods below.
	 */
	void step();

	/** Reset the object to the initial state as if created from scratch */
	void reset();

	// TODO: Getters

   protected:
	Parameters params_;

	struct InternalState
	{
		InternalState() = default;

		State fsm_state = State::UNINITIALIZED;

		mrpt::maps::CMultiMetricMap::Ptr metric_map =
			mrpt::maps::CMultiMetricMap::Create();

		mrpt::bayes::CParticleFilter pf_;  ///< interface for particle filters

		mrpt::bayes::CParticleFilter::TParticleFilterStats pf_stats_;

		/// The filter:
		std::optional<mrpt::slam::CMonteCarloLocalization2D> pdf2d_;
		std::optional<mrpt::slam::CMonteCarloLocalization3D> pdf3d_;

		// mrpt::Clock::time_point time_last_update_;	///< time of the last
		// update

		/// internal counter to count the number of filter updates
		// size_t update_counter_;

		/** Observations in the queue since the last run.
		 *  This field is protected by an independent mutex.
		 */
		std::vector<mrpt::obs::CObservation::Ptr> pendingObs;
	};

   private:
	InternalState state_;
	std::mutex stateMtx_;
	std::mutex pendingObsMtx_;

	mrpt::system::CTimeLogger profiler_{
		true /*enabled*/, "mrpt_pf_localization" /*name*/};

	mrpt::gui::CDisplayWindow3D::Ptr win3D_;

	void init3DDebug();
	void show3DDebug(mrpt::obs::CSensoryFrame::Ptr _observations);

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

	/** To be called only
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
