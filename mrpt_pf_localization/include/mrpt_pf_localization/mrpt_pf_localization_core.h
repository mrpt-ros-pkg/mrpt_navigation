/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mp2p_icp/ICP.h>
#include <mp2p_icp/Parameters.h>
#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/Clock.h>
#include <mrpt/core/pimpl.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>	// TLikelihoodOptions
#include <mrpt/maps/CPointsMap.h>  // TLikelihoodOptions
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CObservationGPS.h>
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
	PFLocalizationCore();
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

		std::set<std::string> metric_map_use_only_these_layers;

		mrpt::maps::CMultiMetricMap::Ptr metric_map;  //!< Empty=uninitialized
		std::optional<mp2p_icp::metric_map_t::Georeferencing> georeferencing;

		// used internally for relocalization only:
		std::vector<std::string> metric_map_layer_names;

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
		mrpt::obs::CActionRobotMovement2D::TMotionModelOptions motion_model_no_odom_2d;

		/** For SE(3) mode: Uncertainty motion model for regular odometry-based
		 * motion. Can be changed at any moment.
		 */
		mrpt::obs::CActionRobotMovement3D::TMotionModelOptions motion_model_3d;

		/** For SE(3) mode: Uncertainty motion model to use when NO odometry has
		 * been received. Can be changed at any moment.
		 */
		mrpt::obs::CActionRobotMovement3D::TMotionModelOptions motion_model_no_odom_3d;

		/** All the PF parameters: algorithm, number of samples, dynamic
		 * samples, etc.
		 * Can be changed while state = UNINITIALIZED.
		 */
		mrpt::bayes::CParticleFilter::TParticleFilterOptions pf_options;

		/** Dynamic sampling-related parameters.
		 * Can be changed at any moment.
		 */
		mrpt::slam::TKLDParams kld_options;

		// likelihood option overrides:
		std::optional<mrpt::maps::CPointsMap::TLikelihoodOptions> override_likelihood_point_maps;
		std::optional<mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions>
			override_likelihood_gridmaps;

		/** Number of particles/m² to use upon initialization.
		 *  Can be changed while state = UNINITIALIZED.
		 */
		unsigned int initial_particles_per_m2 = 10;

		/** If true, the particles will be initialized according to the first
		 *  incomming GNSS observation, once the map has been also received.
		 *  \note This requires a georeferencied metric_map_t.
		 */
		bool initialize_from_gnss = false;

		/// If >0, new tentative particles will be generated from GNSS data,
		/// to help re-localizing if using georeferenced maps:
		uint32_t samples_drawn_from_gnss = 20;

		/// If samples_drawn_from_gnss is enabled, the number of standard
		/// deviations ("sigmas") to use as the area in which to draw random
		/// samples around the GNSS prediction:
		double gnss_samples_num_sigmas = 6.0;

		/// The number of standard deviations ("sigmas") to use as the area in
		/// which to draw random samples around the input initialization pose
		/// (when NOT using GNSS as input)
		double relocalize_num_sigmas = 3.0;

		unsigned int relocalization_initial_divisions_xy = 4;
		unsigned int relocalization_initial_divisions_phi = 4;

		unsigned int relocalization_min_sample_copies_per_candidate = 4;

		double relocalization_resolution_xy = 0.50;	 // [m]
		double relocalization_resolution_phi = 0.20;  // [rad]
		double relocalization_minimum_icp_quality = 0.50;
		double relocalization_icp_sigma = 5.0;	// [m]

		mp2p_icp::ICP::Ptr relocalization_icp;
		mp2p_icp::Parameters relocalization_icp_params;

		mp2p_icp_filters::FilterPipeline relocalization_obs_filter;
		mp2p_icp::ParameterSource paramSource;

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
	};

	/** @name Main API
	 *  @{ */

	/** Load all params from a YAML source.
	 *  This method loads all required params and put the system from
	 * UNINITIALIZED into TO_BE_INITIALIZED.
	 */
	void init_from_yaml(
		const mrpt::containers::yaml& pf_params,
		const mrpt::containers::yaml& relocalization_pipeline);

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
		const std::string& map_config_ini_file, const std::string& simplemap_file);

	/** Defines the map to use from a multimetric map, which may contain
	 * gridmaps, pointclouds, etc.
	 */
	void set_map_from_metric_map(
		const mrpt::maps::CMultiMetricMap::Ptr& metricMap,
		const std::optional<mp2p_icp::metric_map_t::Georeferencing>& georeferencing = std::nullopt,
		const std::vector<std::string>& layerNames = {});

	/** Defines the map to use from a metric_map_t map, with optional
	 * georeferencing.
	 */
	void set_map_from_metric_map(const mp2p_icp::metric_map_t& mm);

	void relocalize_here(const mrpt::poses::CPose3DPDFGaussian& pose);

	bool input_queue_has_odometry();
	std::optional<mrpt::Clock::time_point> input_queue_last_stamp();

	void set_fake_odometry_increment(const mrpt::poses::CPose3D& incrPose);

	// TODO: Getters
	State getState() const { return state_.fsm_state; }

	/** Returns a *copy* (it is intentional) of the parameters at this moment */
	const Parameters getParams() { return params_; }

	/** Returns the last filter estimate, or empty ptr if never run yet.
	 *  Multi thread safe.
	 */
	mrpt::poses::CPose3DPDFParticles::Ptr getLastPoseEstimation() const;

	/** @} */

   protected:
	Parameters params_;

	struct InternalState
	{
		InternalState();

		State fsm_state = State::UNINITIALIZED;

		mrpt::maps::CMultiMetricMap::Ptr metric_map;  //!< Empty=uninitialized
		std::optional<mp2p_icp::metric_map_t::Georeferencing> georeferencing;

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

		/** The last state of the filter, for sending as a copy to the user API
		 */
		mrpt::poses::CPose3DPDFParticles::Ptr lastResult;

		std::optional<mrpt::poses::CPose3D> nextFakeOdometryIncrPose;

		struct Relocalization;
		mrpt::pimpl<Relocalization> pendingRelocalization;
	};

	mrpt::obs::CObservationGPS::Ptr get_last_gnss_obs() const
	{
		auto lck = mrpt::lockHelper(pendingObsMtx_);
		return last_gnss_;
	}

   private:
	InternalState state_;
	std::mutex stateMtx_;

	std::mutex pendingObsMtx_;
	mrpt::obs::CObservationGPS::Ptr last_gnss_;	 // use mtx: pendingObsMtx_

	mrpt::system::CTimeLogger profiler_{true /*enabled*/, "mrpt_pf_localization" /*name*/};

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

	void internal_fill_state_lastResult();

	std::optional<mrpt::poses::CPose3DPDFGaussian> get_gnss_pose_prediction();
};
