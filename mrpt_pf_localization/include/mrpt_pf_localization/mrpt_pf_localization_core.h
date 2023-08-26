/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2023, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/Clock.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
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

		/** initial pose used to intialize the filter.
		 *  Empty=not set yet.
		 */
		std::optional<mrpt::poses::CPose3DPDFGaussian> initial_pose;

		mrpt::maps::CMultiMetricMap::Ptr metric_map;  //!< Empty=uninitialized

		/** Shows a custom MRPT GUI with the PF and map state
		 *  Can be changed at any moment.
		 */
		bool gui_enable = true;

		/** If false (default), will use 2D (SE(2)) particle filter. Otherwise,
		 * the 3D mode (SE(3)) is enabled.
		 * Can be changed while state = UNINITIALIZED.
		 */
		bool use_se3_pf = false;

		/** If gui_enable==true, makes the camera to follow the mean of the
		 * particles.
		 * Can be changed at any moment.
		 */
		bool gui_camera_follow_robot = true;

		/** For SE(2) mode: Uncertainty motion model for regular odometry-based
		 * motion. Can be changed at any moment.
		 */
		mrpt::obs::CActionRobotMovement2D::TMotionModelOptions motion_model_2d;

		/** For SE(2) mode: Uncertainty motion model to use when NO odometry has
		 * been received. Can be changed at any moment.
		 */
		mrpt::obs::CActionRobotMovement2D::TMotionModelOptions
			motion_model_no_odom_2d;

		/** For SE(3) mode: Uncertainty motion model for regular odometry-based
		 * motion. Can be changed at any moment.
		 */
		mrpt::obs::CActionRobotMovement3D::TMotionModelOptions motion_model_3d;

		/** For SE(3) mode: Uncertainty motion model to use when NO odometry has
		 * been received. Can be changed at any moment.
		 */
		mrpt::obs::CActionRobotMovement3D::TMotionModelOptions
			motion_model_no_odom_3d;

		/** All the PF parameters: algorithm, number of samples, dynamic
		 * samples, etc.
		 * Can be changed while state = UNINITIALIZED.
		 */
		mrpt::bayes::CParticleFilter::TParticleFilterOptions pf_options;

		/** Dynamic sampling-related parameters.
		 * Can be changed at any moment.
		 */
		mrpt::slam::TKLDParams kld_options;

		/** Number of particles upon initialization.
		 *  Can be changed while state = UNINITIALIZED.
		 */
		unsigned int initial_particle_count = 2000;

		/// This method loads all parameters from the YAML, except the
		/// metric_map (handled in parent class):
		void load_from(const mrpt::containers::yaml& params);
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
		/// Running as usual, the robot can move and particles will update.
		RUNNING,
		/// Special case: the robot is known to be stopped, do not update the
		/// particles.
		RUNNING_STILL
	};

	/** @name Main API
	 *  @{ */

	/** Load all params from a YAML source.
	 *  This method loads all required params and put the system from
	 * UNINITIALIZED into TO_BE_INITIALIZED.
	 */
	void init_from_yaml(const mrpt::containers::yaml& params);

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

	/** Defines the map to use from a pair of files: an MRPT metric map
	 * definition INI file, and a .simplemap file with sensor observations.
	 *  \return true on success, false on any error.
	 */
	bool set_map_from_simple_map(
		const std::string& map_config_ini_file,
		const std::string& simplemap_file);

	// TODO: Getters
	State getState() const { return state_.fsm_state; }

	/** Returns a *copy* (it is intentional) of the parameters at this moment */
	const Parameters getParams() { return params_; }

	/** @} */

   protected:
	Parameters params_;

	struct InternalState
	{
		InternalState() = default;

		State fsm_state = State::UNINITIALIZED;

		mrpt::maps::CMultiMetricMap::Ptr metric_map;  //!< Empty=uninitialized

		mrpt::bayes::CParticleFilter pf;  ///< interface for particle filters

		mrpt::bayes::CParticleFilter::TParticleFilterStats pf_stats;

		/// The filter:
		std::optional<mrpt::slam::CMonteCarloLocalization2D> pdf2d;
		std::optional<mrpt::slam::CMonteCarloLocalization3D> pdf3d;

		/// Timestamp of the last update (default=INVALID)
		mrpt::Clock::time_point time_last_update;

		mrpt::obs::CObservationOdometry::Ptr last_odom;

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

	/** To be called only when state=UNINITIALIZED.
	 * Checks if the minimum set of params are set, then move state to
	 *TO_BE_INITIALIZED
	 **/
	void onStateUninitialized();

	/** To be called only when state=TO_BE_INITIALIZED.
	 * Initializes the filter at pose initial_pose with initial_particle_count.
	 **/
	void onStateToBeInitialized();

	void onStateRunning();

	void init_gui();
	void update_gui(const mrpt::obs::CSensoryFrame& sf);
};
