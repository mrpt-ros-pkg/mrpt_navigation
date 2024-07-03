/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include <mp2p_icp/icp_pipeline_from_yaml.h>
#include <mp2p_icp_filters/Generator.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/opengl/CEllipsoid2D.h>
#include <mrpt/opengl/CEllipsoid3D.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/ros2bridge/map.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/hyperlink.h>
#include <mrpt/topography/conversions.h>  // geodeticToENU_WGS84
#include <mrpt/topography/data_types.h>	 // TGeodeticCoords
#include <mrpt_pf_localization/mrpt_pf_localization_core.h>

#ifdef HAVE_MOLA_RELOCALIZATION
#include <mola_relocalization/relocalization.h>
#endif

#include <Eigen/Dense>
#include <chrono>

using mrpt::maps::CSimplePointsMap;

template <typename T>
void getOptParam(
	const mrpt::containers::yaml& p, T& var, const std::string& name)
{
	var = p.getOrDefault(name, var);
}

PFLocalizationCore::Parameters::Parameters()
{
	auto cov = mrpt::math::CMatrixDouble66::Identity();
	for (int i = 3; i < 6; i++) cov(i, i) = mrpt::square(1 * M_PI);

	initial_pose =
		mrpt::poses::CPose3DPDFGaussian(mrpt::poses::CPose3D(0, 0, 0), cov);

	// Without odometry: use a large uncertainty for each time step:
	// 2D mode:
	motion_model_no_odom_2d.modelSelection =
		mrpt::obs::CActionRobotMovement2D::mmGaussian;
	motion_model_no_odom_2d.gaussianModel.minStdXY = 0.10;
	motion_model_no_odom_2d.gaussianModel.minStdPHI = 2.0;

	// 3D mode:
	motion_model_no_odom_3d.modelSelection =
		mrpt::obs::CActionRobotMovement3D::mmGaussian;
	// TODO: Params that make more sense?
}

#define MCP_LOAD_REQ_HERE(Yaml__, Var__, Trg__)                                \
	if (!Yaml__.has(#Var__))                                                   \
		throw std::invalid_argument(mrpt::format(                              \
			"Required parameter `%s` not an existing key in dictionary.",      \
			#Var__));                                                          \
	if constexpr (std::is_enum_v<decltype(Trg__)>)                             \
		Trg__ = mrpt::typemeta::TEnumType<std::remove_cv_t<decltype(Trg__)>>:: \
			name2value(Yaml__[#Var__].as<std::string>());                      \
	else                                                                       \
		Trg__ = Yaml__[#Var__].as<decltype(Trg__)>()

#define MCP_LOAD_OPT_HERE(Yaml__, Var__, Trg__)                                \
	if constexpr (std::is_enum_v<decltype(Trg__)>)                             \
	{                                                                          \
		if (!Yaml__.empty() && Yaml__.has(#Var__))                             \
			Trg__ =                                                            \
				mrpt::typemeta::TEnumType<std::remove_cv_t<decltype(Trg__)>>:: \
					name2value(Yaml__[#Var__].as<std::string>());              \
	}                                                                          \
	else if (!Yaml__.isNullNode() && !Yaml__.empty() && Yaml__.has(#Var__))    \
	Trg__ = Yaml__[#Var__].as<decltype(Trg__)>()

#define MCP_LOAD_OPT_DEG_HERE(Yaml__, Var__, Trg__)                    \
	if (!Yaml__.isNullNode() && !Yaml__.empty() && Yaml__.has(#Var__)) \
	Trg__ = mrpt::DEG2RAD(Yaml__[#Var__].as<decltype(Trg__)>())

namespace
{
void load_motion_model2d_from(
	const mrpt::containers::yaml& p,
	mrpt::obs::CActionRobotMovement2D::TMotionModelOptions& mmo)
{
	MCP_LOAD_REQ_HERE(p, modelSelection, mmo.modelSelection);

	ASSERT_(p.has("gaussianModel"));

	MCP_LOAD_OPT_HERE(p, a1, mmo.gaussianModel.a1);
	MCP_LOAD_OPT_HERE(p, a2, mmo.gaussianModel.a2);
	MCP_LOAD_OPT_DEG_HERE(p, a3, mmo.gaussianModel.a3);
	MCP_LOAD_OPT_HERE(p, a4, mmo.gaussianModel.a4);
	MCP_LOAD_OPT_HERE(p, minStdXY, mmo.gaussianModel.minStdXY);
	MCP_LOAD_OPT_DEG_HERE(p, minStdPHI, mmo.gaussianModel.minStdPHI);

	MCP_LOAD_OPT_HERE(p, alfa1_rot_rot, mmo.thrunModel.alfa1_rot_rot);
	MCP_LOAD_OPT_HERE(p, alfa2_rot_trans, mmo.thrunModel.alfa2_rot_trans);
	MCP_LOAD_OPT_HERE(p, alfa3_trans_trans, mmo.thrunModel.alfa3_trans_trans);
	MCP_LOAD_OPT_HERE(p, alfa4_trans_rot, mmo.thrunModel.alfa4_trans_rot);
	MCP_LOAD_OPT_HERE(p, additional_std_XY, mmo.thrunModel.additional_std_XY);
	MCP_LOAD_OPT_DEG_HERE(
		p, additional_std_phi, mmo.thrunModel.additional_std_phi);
}

}  // namespace

void PFLocalizationCore::Parameters::load_from(
	const mrpt::containers::yaml& params)
{
	MCP_LOAD_OPT(params, gui_enable);
	MCP_LOAD_REQ(params, use_se3_pf);
	MCP_LOAD_OPT(params, gui_camera_follow_robot);

	// motion_model_2d
	ASSERT_(params.has("motion_model_2d"));
	load_motion_model2d_from(params["motion_model_2d"], motion_model_2d);

	// motion_model_no_odom_2d
	ASSERT_(params.has("motion_model_no_odom_2d"));
	load_motion_model2d_from(
		params["motion_model_no_odom_2d"], motion_model_2d);

	// motion_model_3d
	MCP_LOAD_OPT(params, motion_model_3d.modelSelection);

	// motion_model_no_odom_3d
	MCP_LOAD_OPT(params, motion_model_no_odom_3d.modelSelection);

	// initial_pose: check minimum required fields, if given via YAML.
	if (params.has("initial_pose"))
	{
		using mrpt::square;

		ASSERT_(params["initial_pose"].isMap());
		ASSERT_(params["initial_pose"].has("mean"));
		ASSERT_(params["initial_pose"]["mean"]["x"].isScalar());
		ASSERT_(params["initial_pose"]["mean"]["y"].isScalar());
		ASSERT_(params["initial_pose"]["std_x"].isScalar());
		ASSERT_(params["initial_pose"]["std_y"].isScalar());

		const auto& ipP = params["initial_pose"];
		const auto& m = ipP["mean"];
		auto& ip = initial_pose;
		ip.emplace();
		ip->mean.x(m["x"].as<double>());
		ip->mean.y(m["y"].as<double>());
		if (m.has("z")) ip->mean.z(m["z"].as<double>());

		double yaw = 0, pitch = 0, roll = 0;
		if (m.has("yaw")) yaw = m["yaw"].as<double>();
		if (m.has("pitch")) pitch = m["pitch"].as<double>();
		if (m.has("roll")) roll = m["roll"].as<double>();

		ip->mean.setYawPitchRoll(yaw, pitch, roll);

		// Defaults:
		ip->cov.setDiagonal(0.0);

		// params:
		ip->cov(0, 0) = square(ipP["std_x"].as<double>());
		ip->cov(1, 1) = square(ipP["std_y"].as<double>());
		ip->cov(2, 2) = square(ipP.getOrDefault("std_z", 0.0));
		ip->cov(3, 3) = square(ipP.getOrDefault("std_yaw", 2.0 * M_PI));
		ip->cov(4, 4) = square(ipP.getOrDefault("std_pitch", 0.0));
		ip->cov(5, 5) = square(ipP.getOrDefault("std_roll", 0.0));
	}

	// pf_options:
	ASSERT_(params.has("pf_options"));
	auto& pfo = params["pf_options"];
	getOptParam(pfo, pf_options.BETA, "BETA");

	{
		// Define these temporary variables to exploit the automatic conversion
		// to/from enums in MCP_LOAD_OPT():
		mrpt::bayes::CParticleFilter::TParticleFilterAlgorithm PF_algorithm =
			pf_options.PF_algorithm;
		MCP_LOAD_OPT(pfo, PF_algorithm);
		pf_options.PF_algorithm = PF_algorithm;

		mrpt::bayes::CParticleFilter::TParticleResamplingAlgorithm
			resamplingMethod = pf_options.resamplingMethod;
		MCP_LOAD_OPT(pfo, resamplingMethod);
		pf_options.resamplingMethod = resamplingMethod;
	}

	getOptParam(pfo, pf_options.adaptiveSampleSize, "adaptiveSampleSize");
	getOptParam(
		pfo, pf_options.pfAuxFilterOptimal_MaximumSearchSamples,
		"pfAuxFilterOptimal_MaximumSearchSamples");
	getOptParam(pfo, pf_options.powFactor, "powFactor");
	getOptParam(
		pfo, pf_options.max_loglikelihood_dyn_range,
		"max_loglikelihood_dyn_range");
	getOptParam(
		pfo, pf_options.pfAuxFilterStandard_FirstStageWeightsMonteCarlo,
		"pfAuxFilterStandard_FirstStageWeightsMonteCarlo");
	getOptParam(
		pfo, pf_options.pfAuxFilterOptimal_MLE, "pfAuxFilterOptimal_MLE");

	// kld_options:
	ASSERT_(params.has("kld_options"));
	auto& kldo = params["kld_options"];
	getOptParam(kldo, kld_options.KLD_binSize_XY, "KLD_binSize_XY");
	getOptParam(kldo, kld_options.KLD_binSize_PHI, "KLD_binSize_PHI");
	getOptParam(kldo, kld_options.KLD_delta, "KLD_delta");
	getOptParam(kldo, kld_options.KLD_epsilon, "KLD_epsilon");
	getOptParam(kldo, kld_options.KLD_maxSampleSize, "KLD_maxSampleSize");
	getOptParam(kldo, kld_options.KLD_minSampleSize, "KLD_minSampleSize");
	getOptParam(kldo, kld_options.KLD_minSamplesPerBin, "KLD_minSamplesPerBin");

	// override_likelihood_point_maps
	if (params.has("override_likelihood_point_maps"))
	{
		auto& likOpts = override_likelihood_point_maps.emplace();

		mrpt::config::CConfigFileMemory cfg;
		std::stringstream ss;
		ss << params["override_likelihood_point_maps"];
		cfg.setContentFromYAML(ss.str());
		likOpts.loadFromConfigFile(cfg, "");
	}

	// override_likelihood_gridmaps
	if (params.has("override_likelihood_gridmaps"))
	{
		auto& likOpts = override_likelihood_gridmaps.emplace();

		mrpt::config::CConfigFileMemory cfg;
		std::stringstream ss;
		ss << params["override_likelihood_gridmaps"];
		cfg.setContentFromYAML(ss.str());
		likOpts.loadFromConfigFile(cfg, "");
	}

	//
	MCP_LOAD_OPT(params, initial_particles_per_m2);
	MCP_LOAD_OPT(params, initialize_from_gnss);
	MCP_LOAD_OPT(params, samples_drawn_from_gnss);
	MCP_LOAD_OPT(params, gnss_samples_num_sigmas);
	MCP_LOAD_OPT(params, relocalize_num_sigmas);

	// relocalization:
	MCP_LOAD_OPT(params, relocalization_minimum_icp_quality);
	MCP_LOAD_OPT(params, relocalization_icp_sigma);
	MCP_LOAD_OPT(params, relocalization_resolution_xy);
	MCP_LOAD_OPT(params, relocalization_min_sample_copies_per_candidate);
	MCP_LOAD_OPT_DEG(params, relocalization_resolution_phi);
	MCP_LOAD_OPT(params, relocalization_initial_divisions_xy);
	MCP_LOAD_OPT(params, relocalization_initial_divisions_phi);
}

struct PFLocalizationCore::InternalState::Relocalization
{
#ifdef HAVE_MOLA_RELOCALIZATION
	std::optional<mola::RelocalizationICP_SE2::Input> pending_se2;
#endif
};

PFLocalizationCore::InternalState::InternalState()
	: pendingRelocalization(
		  mrpt::make_impl<PFLocalizationCore::InternalState::Relocalization>())
{
}

PFLocalizationCore::PFLocalizationCore()
	: mrpt::system::COutputLogger("mrpt_pf_localization")
{
}

void PFLocalizationCore::on_observation(const mrpt::obs::CObservation::Ptr& obs)
{
	auto tle = mrpt::system::CTimeLoggerEntry(profiler_, "on_observation");

	auto lck = mrpt::lockHelper(pendingObsMtx_);
	state_.pendingObs.push_back(obs);

	if (auto gps = std::dynamic_pointer_cast<mrpt::obs::CObservationGPS>(obs);
		gps && gps->has_GGA_datum())
	{
		// for the PF, we only care about GPS observations with GGA positioning:
		// (Note: all NavSatFix msgs are mapped into MRPT GGA GPS messages)
		last_gnss_ = gps;
	}
}

bool PFLocalizationCore::input_queue_has_odometry()
{
	auto lck = mrpt::lockHelper(pendingObsMtx_);
	for (const auto& o : state_.pendingObs)
	{
		if (auto oOdo =
				std::dynamic_pointer_cast<mrpt::obs::CObservationOdometry>(o);
			oOdo)
			return true;
	}
	return false;
}

std::optional<mrpt::Clock::time_point>
	PFLocalizationCore::input_queue_last_stamp()
{
	auto lck = mrpt::lockHelper(pendingObsMtx_);

	std::optional<mrpt::Clock::time_point> lastStamp;
	for (const auto& o : state_.pendingObs)
	{
		ASSERT_(o);
		const auto t = o->timestamp;
		if (!lastStamp)
			lastStamp = t;
		else
			mrpt::keep_min(*lastStamp, t);
	}
	return lastStamp;
}

// The main API call: executes one PF step, taking into account all the
// parameters and observations gathered so far, updates the optional GUI, etc.
void PFLocalizationCore::step()
{
	auto lck = mrpt::lockHelper(stateMtx_);

	switch (state_.fsm_state)
	{
		case State::UNINITIALIZED:
			onStateUninitialized();
			break;
		case State::TO_BE_INITIALIZED:
			onStateToBeInitialized();
			break;
		case State::RUNNING:
			onStateRunning();
			break;

		default:
			THROW_EXCEPTION("Invalid internal FSM state (!?)");
	}
}

/** Reset the object to the initial state as if created from scratch */
void PFLocalizationCore::reset()
{
	auto lck = mrpt::lockHelper(stateMtx_);
	state_ = InternalState();
}

void PFLocalizationCore::onStateUninitialized()
{
	using namespace std::string_literals;

	auto lck = mrpt::lockHelper(pendingObsMtx_);  // to protect last_gnss_

	// Check if we have everything we need to get going:
	if (((!params_.initialize_from_gnss && params_.initial_pose.has_value()) ||
		 (params_.initialize_from_gnss && last_gnss_)) &&
		params_.metric_map)
	{
		// Move:
		state_.fsm_state = State::TO_BE_INITIALIZED;

		MRPT_LOG_INFO(
			"Required configuration has been set, changing to "
			"'State::TO_BE_INITIALIZED'");

		return;
	}

	// We don't have parameters / map yet. Do nothing:
	std::string excuses;
	if (!params_.metric_map) excuses += "No reference metric map. ";
	if (params_.initialize_from_gnss)
	{
		if (!last_gnss_) excuses += "No GNSS observation received yet. ";
	}
	else
	{
		if (!params_.initial_pose) excuses += "No initial pose. ";
	}

	MRPT_LOG_THROTTLE_WARN(
		3.0,
		"Doing nothing, state is UNINITIALIZED yet. "
		"Excuses: "s +
			excuses + "Refer to "s +
			mrpt::system::hyperlink(
				"package documentation.",
				"https://github.com/mrpt-ros-pkg/mrpt_navigation", true));
}

void PFLocalizationCore::onStateToBeInitialized()
{
	// Profiler report:
	profiler_.setLoggerName(profiler_.getName());

	auto tle =
		mrpt::system::CTimeLoggerEntry(profiler_, "onStateToBeInitialized");

	// Reset state:
	auto& _ = state_;
	_ = InternalState();

	// fsm:
	_.fsm_state = State::RUNNING;

	// the map to use:
	ASSERT_(params_.metric_map);
	_.metric_map = params_.metric_map;
	_.georeferencing = params_.georeferencing;

	// Create the 2D or 3D particle filter object:
	if (params_.use_se3_pf)
	{
		MRPT_LOG_INFO_STREAM(
			"[onStateToBeInitialized] Initializing in SE(3) mode");
		_.pdf3d.emplace();
	}
	else
	{
		MRPT_LOG_INFO_STREAM(
			"[onStateToBeInitialized] Initializing in SE(2) mode");
		_.pdf2d.emplace();
	}

	double gnss_std_factor = 1.0;

	// Get desired initial pose uncertainty, either from
	// direct pose, or from GNSS:
	const auto [pCov, pMean] =
		[&]() -> std::tuple<mrpt::math::CMatrixDouble66, mrpt::poses::CPose3D>
	{
		if (!params_.initialize_from_gnss ||
			// because may be here after a manual click-to-relocalize in RViz,
			// even if GNSS localization is enabled:
			(!get_last_gnss_obs() && params_.initial_pose.has_value())	//
		)
		{
			MRPT_LOG_INFO_STREAM(
				"[onStateToBeInitialized] Initial pose: "
				<< *params_.initial_pose);

			const auto [pCov, pMean] =
				params_.initial_pose->getCovarianceAndMean();
			return {pCov, pMean};
		}
		else
		{
			ASSERTMSG_(
				_.georeferencing,
				"The provided metric map needs to be georeferenced for "
				"'initialize_from_gnss' = true");

			const auto gnssMeasInMap = get_gnss_pose_prediction();
			ASSERT_(gnssMeasInMap);

			MRPT_LOG_INFO_STREAM(
				"Initializing from GNSS measurement with: " << *gnssMeasInMap);

			gnss_std_factor =
				params_.gnss_samples_num_sigmas / params_.relocalize_num_sigmas;

			return {gnssMeasInMap->cov, gnssMeasInMap->mean};
		}
	}();

	const double stdX = std::sqrt(pCov(0, 0));
	const double stdY = std::sqrt(pCov(1, 1));
	const double stdZ = std::sqrt(pCov(2, 2));
	const double stdYaw = std::sqrt(pCov(3, 3));
	const double stdPitch = std::sqrt(pCov(4, 4));
	const double stdRoll = std::sqrt(pCov(5, 5));

	const double nStds = params_.relocalize_num_sigmas * gnss_std_factor;

	const auto pMin = mrpt::math::TPose3D(
		pMean.x() - nStds * stdX, pMean.y() - nStds * stdY,
		pMean.z() - nStds * stdZ,  //
		std::max(-M_PI, pMean.yaw() - nStds * stdYaw),
		std::max(-M_PI, pMean.pitch() - nStds * stdPitch),
		std::max(-M_PI, pMean.roll() - nStds * stdRoll));

	const auto pMax = mrpt::math::TPose3D(
		pMean.x() + nStds * stdX, pMean.y() + nStds * stdY,
		pMean.z() + nStds * stdZ,  //
		std::min(M_PI, pMean.yaw() + nStds * stdYaw),
		std::min(M_PI, pMean.pitch() + nStds * stdPitch),
		std::min(M_PI, pMean.roll() + nStds * stdRoll));

	// two options here:
	// 1) pure particle filter
	// 2) use mola_relocalization to help focus on the interesting areas:
	bool use_mola_relocalization = false;

#if defined(HAVE_MOLA_RELOCALIZATION)
	// so far, only for SE(2) mode:
	if (_.pdf2d) use_mola_relocalization = true;
#endif

	if (!use_mola_relocalization)
	{  // 1) pure PF:
		bool initDone = false;

		const double area =
			std::max<double>(10.0, (pMax.x - pMin.x) * (pMax.y - pMin.y));
		const size_t initParticleCount =
			static_cast<size_t>(params_.initial_particles_per_m2 * area);

		if (auto gridMap =
				_.metric_map->mapByClass<mrpt::maps::COccupancyGridMap2D>();
			gridMap && _.pdf2d)
		{
			// initialize over free space only:
			try
			{
				const float gridFreenessThreshold = 0.7f;
				_.pdf2d->resetUniformFreeSpace(
					gridMap.get(), gridFreenessThreshold, initParticleCount,
					pMin.x, pMax.x, pMin.y, pMax.y, pMin.yaw, pMax.yaw);

				initDone = true;
			}
			catch (const std::exception& e)
			{
				MRPT_LOG_ERROR_STREAM(
					"Error trying to initialize over gridmap empty space, "
					"falling "
					"back to provided initialPose. Error: "
					<< e.what());
			}
		}

		if (!initDone)
		{
			if (_.pdf2d)
				_.pdf2d->resetUniform(
					pMin.x, pMax.x, pMin.y, pMax.y, pMin.yaw, pMax.yaw,
					initParticleCount);
			else
				_.pdf3d->resetUniform(pMin, pMax, initParticleCount);
		}
	}
	else
	{
#if defined(HAVE_MOLA_RELOCALIZATION)
		// 2) Use mola_relocalization
		auto& in = state_.pendingRelocalization->pending_se2.emplace();
		in.icp_minimum_quality = params_.relocalization_minimum_icp_quality;

		in.icp_parameters = params_.relocalization_icp_params;
		in.icp_pipeline = {params_.relocalization_icp};

		MRPT_LOG_INFO_STREAM(
			"Setting up relocalization with: pMin=" << pMin
													<< " pMax=" << pMax);
		const auto pDiag = pMax - pMin;

		in.initial_guess_lattice.corner_min = mrpt::math::TPose2D(pMin);
		in.initial_guess_lattice.corner_max = mrpt::math::TPose2D(pMax);
		in.initial_guess_lattice.resolution_xy = std::max<double>(
			0.5, pDiag.norm() / params_.relocalization_initial_divisions_xy);
		in.initial_guess_lattice.resolution_phi = std::max<double>(
			mrpt::DEG2RAD(5.0),
			(pMax.yaw - pMin.yaw) /
				params_.relocalization_initial_divisions_phi);

		in.output_lattice.resolution_xyz = params_.relocalization_resolution_xy;
		in.output_lattice.resolution_yaw =
			params_.relocalization_resolution_phi;

		// in.local_map: to be populated in running state

		// (shallow) copy metric maps into expected format:
		const auto& maps = _.metric_map->maps;
		ASSERT_(!maps.empty());
		ASSERT_(
			params_.metric_map_layer_names.empty() ||
			params_.metric_map_layer_names.size() == maps.size());
		for (size_t i = 0; i < maps.size(); i++)
		{
			const std::string layerName =
				params_.metric_map_layer_names.empty()
					? std::to_string(i)
					: params_.metric_map_layer_names.at(i);
			in.reference_map.layers[layerName] = maps.at(i);
		}

		// If the referenceMap is a "plain old" gridMap, create an auxiliary
		// point cloud for ICP to work fine:
		if (auto gridMap =
				_.metric_map->mapByClass<mrpt::maps::COccupancyGridMap2D>();
			gridMap)
		{
			auto gridPts = mrpt::maps::CSimplePointsMap::Create();
			gridMap->getAsPointCloud(*gridPts);
			in.reference_map.layers["localmap"] = gridPts;

			MRPT_LOG_INFO_STREAM(
				"Creating localmap layer with "
				<< gridPts->size() << " points from the occupied grid cells.");
		}

#else
		THROW_EXCEPTION("Should not reach here");
#endif
	}

	internal_fill_state_lastResult();
}

void PFLocalizationCore::onStateRunning()
{
	auto tle = mrpt::system::CTimeLoggerEntry(profiler_, "onStateRunning");

	// Collect observations since last execution and build "action" and
	// "observations" for the Bayes filter:
	mrpt::obs::CSensoryFrame sf;  // sorted, and thread-safe copy of all obs.
	mrpt::Clock::time_point sfLastTimeStamp;
	{
		// If we have multiple observations of the same sensor for this
		// single PF step, discard all but the latest one. Temporary storage
		// of observations:
		std::map<std::string, mrpt::obs::CObservation::Ptr> obsByLabel;
		std::map<std::string, std::string> obsClassByLabel;

		auto lck = mrpt::lockHelper(pendingObsMtx_);
		for (auto& o : state_.pendingObs)
		{
			if (!o) continue;  // who knows...users may be evil :-)
			sfLastTimeStamp = o->getTimeStamp();

			const std::string thisObsClass = o->GetRuntimeClass()->className;

			if (auto it = obsClassByLabel.find(o->sensorLabel);
				it == obsClassByLabel.end())
			{
				obsByLabel[o->sensorLabel] = o;
				obsClassByLabel[o->sensorLabel] = thisObsClass;
			}
			else
			{
				// Sanity check:
				if (it->second != thisObsClass)
				{
					THROW_EXCEPTION_FMT(
						"ERROR: Received two observations with "
						"sensorLabel='%s' and different classes: '%s' vs "
						"'%s'",
						o->sensorLabel.c_str(), it->second.c_str(),
						thisObsClass.c_str());
				}
				// All is correct. Update last obs of this type:
				obsByLabel[o->sensorLabel] = o;
			}
		}
		state_.pendingObs.clear();

		// Insert the last obs only for each type:
		for (const auto& kv : obsByLabel) sf.insert(kv.second);
	}

	// Do we have *any* usable observation?
	// Not any observation is usable with any map:
	bool canComputeLikelihood = false;
	for (const auto& m : state_.metric_map->maps)
	{
		if (m->canComputeObservationsLikelihood(sf))
		{
			canComputeLikelihood = true;
			break;
		}
	}

	if (!canComputeLikelihood)
	{
		MRPT_LOG_DEBUG(
			"No usable observation in the input queue. Skipping PF "
			"update.");

		internal_fill_state_lastResult();
		if (params_.gui_enable) update_gui(sf);
		return;
	}

	// --------------------------------------------

	// "Action"
	// ------------------------
	// Build it from odometry, if we have, or from "fake null odometry"
	// instead
	mrpt::obs::CObservationOdometry::Ptr odomObs;
	// get the LAST odometry reading, if we have many:
	for (size_t i = 0;
		 auto o = sf.getObservationByClass<mrpt::obs::CObservationOdometry>(i);
		 i++)
	{
		odomObs = o;
	}

	MRPT_LOG_DEBUG_STREAM(
		"onStateRunning: " << sf.size() << " observations=\n"
						   <<
		[&]()
		{
			std::stringstream ss;
			for (const auto& obs : sf)
				ss << " - " << obs->sensorLabel
				   << " class: " << obs->GetRuntimeClass()->className << "\n";
			return ss.str();
		}());

	const bool is_3D = state_.pdf3d.has_value();

	std::optional<mrpt::obs::CActionRobotMovement2D::Ptr> odomMove2D;
	std::optional<mrpt::obs::CActionRobotMovement3D::Ptr> odomMove3D;

	if (is_3D)
	{
		odomMove3D.emplace(mrpt::obs::CActionRobotMovement3D::Create());
		odomMove3D.value()->timestamp = sfLastTimeStamp;
	}
	else
	{
		odomMove2D.emplace(mrpt::obs::CActionRobotMovement2D::Create());
		odomMove2D.value()->timestamp = sfLastTimeStamp;
	}

	// Use real odometry increments, or fake odom instead:
	if (odomObs)
	{
		const mrpt::poses::CPose2D incOdoPose =
			state_.last_odom ? odomObs->odometry - state_.last_odom->odometry
							 : mrpt::poses::CPose2D::Identity();

		state_.last_odom = odomObs;

		if (odomMove2D.has_value())
		{
			odomMove2D.value()->computeFromOdometry(
				incOdoPose, params_.motion_model_2d);

			MRPT_LOG_DEBUG_STREAM(
				"onStateRunning: motion model= " <<
				[&]()
				{
					std::stringstream ss;
					odomMove2D.value()->getDescriptionAsText(ss);
					return ss.str();
				}());
		}
		else
		{
			// TODO: Use 3D odometry observations?
			// Does it make sense for some application?

			odomMove3D.value()->computeFromOdometry(
				mrpt::poses::CPose3D(incOdoPose), params_.motion_model_3d);

			MRPT_LOG_DEBUG_STREAM(
				"onStateRunning: motion model= " <<
				[&]()
				{
					std::stringstream ss;
					odomMove3D.value()->getDescriptionAsText(ss);
					return ss.str();
				}());
		}
	}
	else
	{
		// Use fake "null movement" motion model with the special
		// uncertainty:
		const mrpt::poses::CPose3D odoIncrPose =
			state_.nextFakeOdometryIncrPose.has_value()
				? *state_.nextFakeOdometryIncrPose
				: mrpt::poses::CPose3D::Identity();

		if (odomMove2D.has_value())
		{
			odomMove2D.value()->computeFromOdometry(
				mrpt::poses::CPose2D(odoIncrPose),
				params_.motion_model_no_odom_2d);
		}
		else
		{
			odomMove3D.value()->computeFromOdometry(
				odoIncrPose, params_.motion_model_no_odom_3d);
		}
		MRPT_LOG_DEBUG_STREAM(
			"onStateRunning: motion model=NONE, with fake odo incrPose="
			<< odoIncrPose.asString());
	}

	state_.nextFakeOdometryIncrPose.reset();  // In any case, forget this.

	mrpt::obs::CActionCollection actions;
	if (is_3D)
		actions.insertPtr(*odomMove3D);
	else
		actions.insertPtr(*odomMove2D);

		// Any pending relocalization step?
		// -----------------------------------------
#if defined(HAVE_MOLA_RELOCALIZATION)
	if (auto& in = state_.pendingRelocalization->pending_se2; in)
	{
		// populate the missing field to "in": "local_map"

		// Use default generator: takes observations and populate a "raw" layer
		auto gen = mp2p_icp_filters::Generator::Create();
		gen->initialize({});
		mp2p_icp_filters::GeneratorSet gens = {gen};

		in->local_map = mp2p_icp_filters::apply_generators(gens, sf);

		// Apply optional filtering
		mp2p_icp_filters::apply_filter_pipeline(
			params_.relocalization_obs_filter, in->local_map);

		if (auto rawPts = in->local_map.point_layer("raw");
			!rawPts || rawPts->empty())
		{
			MRPT_LOG_WARN_STREAM(
				"Relocalization skipped in this iteration, it seems no valid "
				"observations have reached yet (empty local observation map)");
			return;
		}

		MRPT_LOG_INFO_STREAM(
			"About to run Relocalization with local_map="
			<< in->local_map.contents_summary() << " from |SF|=" << sf.size()
			<< " reference_map=" << in->reference_map.contents_summary());

		const auto reloc = mola::RelocalizationICP_SE2::run(*in);

		std::vector<mrpt::math::TPose2D> candidates;
		reloc.found_poses.visitAllPoses(
			[&](const auto& p)
			{ candidates.push_back(mrpt::math::TPose2D(p)); });

#if 0
		for (const auto& p : candidates)
			MRPT_LOG_INFO_STREAM("Candidate: " << p);
#endif

		if (candidates.empty())
		{
			MRPT_LOG_WARN(
				"Could not find any good match between the input observation "
				"and the map (Is the correct map loaded?).");

			// Create one candidate at the center of the requested
			// initialization ROI:
			const auto& igl = in->initial_guess_lattice;
			candidates.emplace_back(
				0.5 * (igl.corner_max.x + igl.corner_min.x),
				0.5 * (igl.corner_max.y + igl.corner_min.y),
				0.5 * (igl.corner_max.phi + igl.corner_min.phi));
		}

		// Create a few particles around each best candidate:
		ASSERT_(state_.pdf2d);
		auto& parts = state_.pdf2d->m_particles;
		parts.clear();
		mrpt::random::CRandomGenerator rng;
		const double sigmaXY = params_.relocalization_resolution_xy * 0.33;
		const double sigmaPhi = params_.relocalization_resolution_phi * 0.33;

		const size_t numCopies = std::max<size_t>(
			params_.relocalization_min_sample_copies_per_candidate,
			mrpt::round(
				params_.initial_particles_per_m2 * mrpt::square(sigmaXY)));

		MRPT_LOG_INFO_STREAM(
			"RelocalizationICP_SE2 took "
			<< reloc.time_cost << " s and gave " << candidates.size()
			<< " candidates. Particles copies per candidate=" << numCopies);

		for (const auto& pose : candidates)
		{
			for (size_t i = 0; i < numCopies; i++)
			{
				auto p = pose;
				p.x += rng.drawGaussian1D(0, sigmaXY);
				p.y += rng.drawGaussian1D(0, sigmaXY);
				p.phi += rng.drawGaussian1D(0, sigmaPhi);
				p.normalizePhi();
				parts.emplace_back(p, 0.0 /*log weight*/);
			}
		}

		// mark the relocalization as done:
		state_.pendingRelocalization->pending_se2.reset();

	}  // end relocalization
#endif

	// Make sure params are up-to-date in the PF
	// (they may change on-the-fly by users):
	// -----------------------------------------
	state_.pf.m_options = params_.pf_options;
	mrpt::slam::TMonteCarloLocalizationParams pdfPredictionOptions;
	pdfPredictionOptions.KLD_params = params_.kld_options;

	if (state_.pdf2d)
	{
		state_.pdf2d->options = pdfPredictionOptions;
		state_.pdf2d->options.metricMap = params_.metric_map;
	}
	else
	{
		state_.pdf3d->options = pdfPredictionOptions;
		state_.pdf3d->options.metricMap = params_.metric_map;
	}

	// Draw additional helper samples from GNSS readings?
	// ----------------------------------------------------
	if (auto gnssPos = get_gnss_pose_prediction();
		gnssPos && params_.samples_drawn_from_gnss > 0)
	{
		mrpt::poses::CPoseRandomSampler sampler;
		sampler.setPosePDF(*gnssPos);

		for (size_t i = 0; i < params_.samples_drawn_from_gnss; i++)
		{
			mrpt::poses::CPose3D p;
			sampler.drawSample(p);

			if (state_.pdf2d)
			{
				auto& newPart = state_.pdf2d->m_particles.emplace_back();
				newPart.log_w = .0;
				newPart.d = mrpt::math::TPose2D(p.asTPose());
			}
			else
			{
				auto& newPart = state_.pdf3d->m_particles.emplace_back();
				newPart.log_w = .0;
				newPart.d = p.asTPose();
			}
		}
	}

	// Process PF
	// ------------------------
	mrpt::bayes::CParticleFilterCapable& pfc =
		state_.pdf2d
			? static_cast<mrpt::bayes::CParticleFilterCapable&>(*state_.pdf2d)
			: static_cast<mrpt::bayes::CParticleFilterCapable&>(*state_.pdf3d);

	state_.pf.executeOn(pfc, &actions, &sf, &state_.pf_stats);

	MRPT_LOG_DEBUG_STREAM(
		"onStateRunning: executed PF, ESS_beforeResample="
		<< state_.pf_stats.ESS_beforeResample);

	// Collect further output stats:
	// ------------------------------
	state_.time_last_update = sfLastTimeStamp;

	internal_fill_state_lastResult();

	// clear last GNSS so we do not use it more than once:
	last_gnss_.reset();

	// GUI:
	// -----------
	// Init optional debug GUI:
	if (params_.gui_enable) update_gui(sf);
}

bool PFLocalizationCore::set_map_from_simple_map(
	const std::string& map_config_ini_file, const std::string& simplemap_file)
{
	// No need to lock mutex, done in set_map_from_metric_map()

	ASSERT_FILE_EXISTS_(map_config_ini_file);
	ASSERT_FILE_EXISTS_(simplemap_file);

	mrpt::config::CConfigFile cfg(map_config_ini_file);

	auto newMap = mrpt::maps::CMultiMetricMap::Create();

	bool ok = mrpt::ros2bridge::MapHdl::loadMap(
		*newMap, cfg, simplemap_file, "metricMap");

	if (!ok)
	{
		MRPT_LOG_ERROR_STREAM(
			"Error loading metric map from: map_config_ini_file='"
			<< map_config_ini_file << "', simplemap_file='" << simplemap_file
			<< "'");
	}
	else
	{
		MRPT_LOG_DEBUG_STREAM(
			"Successful load of metric map from: map_config_ini_file='"
			<< map_config_ini_file << "', simplemap_file='" << simplemap_file
			<< "'");

		// Actually switch/set the map:
		set_map_from_metric_map(newMap);
	}

	return ok;
}

void PFLocalizationCore::set_map_from_metric_map(
	const mp2p_icp::metric_map_t& mm)
{
	// Convert it to CMultiMetricMap, and save the optional georeferrencing:
	auto mMap = mrpt::maps::CMultiMetricMap::Create();

	std::vector<std::string> layerNames;
	for (const auto& [layerName, layerMap] : mm.layers)
	{
		mMap->maps.push_back(layerMap);
		layerNames.push_back(layerName);
	}

	this->set_map_from_metric_map(mMap, mm.georeferencing, layerNames);
}

void PFLocalizationCore::set_map_from_metric_map(
	const mrpt::maps::CMultiMetricMap::Ptr& metricMap,
	const std::optional<mp2p_icp::metric_map_t::Georeferencing>& georeferencing,
	const std::vector<std::string>& layerNames)
{
	auto lck = mrpt::lockHelper(stateMtx_);

	for (const auto& m : metricMap->maps)
	{
		ASSERT_(m);

		if (auto pts = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(m);
			pts && params_.override_likelihood_point_maps)
		{
			pts->likelihoodOptions = *params_.override_likelihood_point_maps;
		}
		else if (auto occ2D =
					 std::dynamic_pointer_cast<mrpt::maps::COccupancyGridMap2D>(
						 m);
				 occ2D && params_.override_likelihood_gridmaps)
		{
			occ2D->likelihoodOptions = *params_.override_likelihood_gridmaps;
		}
	}

	params_.metric_map = metricMap;
	params_.georeferencing = georeferencing;
	params_.metric_map_layer_names = layerNames;

	// debug trace with full submap details: ----------------------------
	MRPT_LOG_DEBUG_STREAM(
		"set_map_from_metric_map: Map contents: " <<
		[&]()
		{
			std::stringstream ss;
			ss << metricMap->asString() << ". Maps:\n";
			for (const auto& m : metricMap->maps)
			{
				ASSERT_(m);
				ss << " - " << m->asString() << "\n";
				if (auto pts =
						std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(m);
					pts)
				{
					ss << "  CPointsMap::likelihoodOptions:\n";

					pts->likelihoodOptions.dumpToTextStream(ss);
					ss << "\n";
				}
				else if (auto occ2D = std::dynamic_pointer_cast<
							 mrpt::maps::COccupancyGridMap2D>(m);
						 occ2D)
				{
					ss << "  COccupancyGridMap2D::likelihoodOptions:\n";
					occ2D->likelihoodOptions.dumpToTextStream(ss);
					ss << "\n";
				}
				ss << "\n";
			}
			return ss.str();
		}());
	// end of debug trace ^^^^^^^^^^^^^^^^^
}

/* Load all params from a YAML source.
 *  This method loads all required params and put the system from
 * UNINITIALIZED into TO_BE_INITIALIZED.
 */
void PFLocalizationCore::init_from_yaml(
	const mrpt::containers::yaml& pf_params,
	const mrpt::containers::yaml& relocalization_pipeline)
{
	MRPT_LOG_INFO("Called init_from_yaml()");

	auto lck = mrpt::lockHelper(stateMtx_);

	// Load all required and optional params:
	params_.load_from(pf_params);

	if (pf_params.asMap().count("log_level_core"))
	{
		const auto coreLogLevel =
			mrpt::typemeta::str2enum<mrpt::system::VerbosityLevel>(
				pf_params["log_level_core"].as<std::string>());
		this->setMinLoggingLevel(coreLogLevel);
	}

#ifdef HAVE_MOLA_RELOCALIZATION
	if (!relocalization_pipeline.empty())
	{
		// ICP pipeline:
		const auto [icp, icpParams] =
			mp2p_icp::icp_pipeline_from_yaml(relocalization_pipeline);
		params_.relocalization_icp = icp;
		params_.relocalization_icp_params = icpParams;

		params_.paramSource.updateVariable(
			"ADAPTIVE_THRESHOLD_SIGMA", params_.relocalization_icp_sigma);
		params_.paramSource.updateVariable("ICP_ITERATION", 0);
		params_.relocalization_icp->attachToParameterSource(
			params_.paramSource);
		params_.paramSource.realize();

		// filter?
		if (relocalization_pipeline.has("relocalization_observation_pipeline"))
		{
			params_.relocalization_obs_filter =
				mp2p_icp_filters::filter_pipeline_from_yaml(
					relocalization_pipeline
						["relocalization_observation_pipeline"]);
		}
	}
	else
	{
		MRPT_LOG_WARN(
			"No relocalization_pipeline YAML configuration provided, so this "
			"feature will be disabled");
	}
#endif
}

void PFLocalizationCore::init_gui()
{
	MRPT_LOG_DEBUG("Initializing GUI");

	if (!params_.gui_enable) return;
	if (win3D_) return;	 // already done.

	win3D_ =
		mrpt::gui::CDisplayWindow3D::Create("mrpt_pf_localization", 1000, 600);
	win3D_->setCameraZoom(20);
	win3D_->setCameraAzimuthDeg(-45);

	if (state_.metric_map)
	{
		auto glMap = state_.metric_map->getVisualization();

		auto scene = win3D_->get3DSceneAndLock();
		scene->insert(glMap);
		scene->enableFollowCamera(params_.gui_camera_follow_robot);
		win3D_->unlockAccess3DScene();
	}
}

void PFLocalizationCore::update_gui(const mrpt::obs::CSensoryFrame& sf)
{
	using namespace mrpt::opengl;

	auto tle = mrpt::system::CTimeLoggerEntry(profiler_, "show3DDebug");

#if !MRPT_HAS_WXWIDGETS
	return;	 // we don't have built-in GUI!
#endif

	if (!params_.gui_enable) return;

	// Create 3D window if requested:
	if (!win3D_) init_gui();

	mrpt::system::TTimeStamp cur_obs_timestamp = INVALID_TIMESTAMP;
	if (!sf.empty()) cur_obs_timestamp = sf.getObservationByIndex(0)->timestamp;

	// Get current estimation as 3D pose PDF:
	mrpt::poses::CPose3DPDFGaussian estimatedPose;
	if (state_.pdf2d)
	{
		// SE(2) to SE(3):
		const auto [cov2D, meanPose2D] = state_.pdf2d->getCovarianceAndMean();
		estimatedPose.mean = mrpt::poses::CPose3D(meanPose2D);
		estimatedPose.cov.setZero();
		constexpr int remapIdx[3] = {0, 1, 3};	// x->x, y->y, yaw->phi
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				estimatedPose.cov(remapIdx[i], remapIdx[j]) = cov2D(i, j);
	}
	else
	{
		const auto [cov, meanPose] = state_.pdf3d->getCovarianceAndMean();
		estimatedPose.mean = meanPose;
		estimatedPose.cov = cov;
	}
	const auto& meanPose = estimatedPose.mean;

	mrpt::opengl::Scene::Ptr scene;
	{
		mrpt::gui::CDisplayWindow3DLocker winLock(*win3D_, scene);

		win3D_->setCameraPointingToPoint(
			estimatedPose.mean.x(), estimatedPose.mean.y(), 0);

		mrpt::opengl::TFontParams fp;
		fp.color = mrpt::img::TColorf(.8f, .8f, .8f);
		fp.vfont_name = "mono";
		fp.vfont_scale = 15;

		win3D_->addTextMessage(
			10, 10,
			mrpt::format(
				"timestamp: %s",
				cur_obs_timestamp != INVALID_TIMESTAMP
					? mrpt::system::dateTimeLocalToString(cur_obs_timestamp)
						  .c_str()
					: "(none)"),
			6001, fp);

		win3D_->addTextMessage(
			10, 33,
			mrpt::format(
				"Particle count= %7u",
				static_cast<unsigned int>(
					state_.pdf2d ? state_.pdf2d->size()
								 : state_.pdf3d->size())),
			6002, fp);

		win3D_->addTextMessage(
			10, 55,
			mrpt::format(
				"mean pose (x y z yaw_deg pitch_deg roll_deg)= %s",
				estimatedPose.getPoseMean().asString().c_str()),
			6003, fp);

		// The particles:
		{
			CRenderizable::Ptr parts = scene->getByName("particles");
			if (parts) scene->removeObject(parts);

			CSetOfObjects::Ptr p =
				state_.pdf2d
					? state_.pdf2d->getAs3DObject<CSetOfObjects::Ptr>()
					: state_.pdf3d->getAs3DObject<CSetOfObjects::Ptr>();
			p->setName("particles");
			scene->insert(p);
		}

		// The particles' covariance as an ellipsoid:
		if (state_.pdf2d)
		{
			CRenderizable::Ptr ellip = scene->getByName("parts_cov");
			if (!ellip)
			{
				auto o = CEllipsoid2D::Create();
				ellip = o;
				ellip->setName("parts_cov");
				ellip->setColor(1, 0, 0, 0.6);

				o->setLineWidth(2);
				o->setQuantiles(3);
				o->set2DsegmentsCount(60);
				scene->insert(ellip);
			}
			ellip->setLocation(meanPose.x(), meanPose.y(), 0.05);
			dynamic_cast<CEllipsoid2D*>(ellip.get())
				->setCovMatrix(estimatedPose.cov.blockCopy<2, 2>());
		}
		else
		{
			CRenderizable::Ptr ellip = scene->getByName("parts_cov");
			if (!ellip)
			{
				auto o = CEllipsoid3D::Create();
				ellip = o;
				ellip->setName("parts_cov");
				ellip->setColor(1, 0, 0, 0.6);

				o->setLineWidth(2);
				o->setQuantiles(3);
				o->set3DsegmentsCount(60);
				scene->insert(ellip);
			}
			ellip->setLocation(meanPose.translation());
			dynamic_cast<CEllipsoid3D*>(ellip.get())
				->setCovMatrix(estimatedPose.cov.blockCopy<3, 3>());
		}

		// The laser scan and other observations:
		{
			CRenderizable::Ptr scan_pts = scene->getByName("scan");
			if (!scan_pts)
			{
				auto o = CPointCloud::Create();
				scan_pts = o;
				scan_pts->setName("scan");
				scan_pts->setColor(1, 0, 0, 0.9);
				o->enableColorFromZ(false);
				o->setPointSize(4);
				scene->insert(scan_pts);
			}

			mrpt::maps::CSimplePointsMap map;
			static mrpt::maps::CSimplePointsMap last_map;

			map.clear();
			sf.insertObservationsInto(map);

			dynamic_cast<CPointCloud*>(scan_pts.get())
				->loadFromPointsMap(&last_map);
			dynamic_cast<CPointCloud*>(scan_pts.get())->setPose(meanPose);
			last_map = map;
		}

		// The camera:
		scene->enableFollowCamera(true);

		// Views:
		auto view1 = scene->getViewport("main");
		{
			CCamera& cam = view1->getCamera();
			cam.setAzimuthDegrees(-90);
			cam.setElevationDegrees(90);
			cam.setPointingAt(meanPose);
			cam.setZoomDistance(5);
			cam.setOrthogonal();
		}

		// Move camera:
		// win3D_->setCameraPointingToPoint( curRobotPose.x, curRobotPose.y,
		// curRobotPose.z );

	}  // end scene lock

	// Update:
	win3D_->forceRepaint();
}

void PFLocalizationCore::relocalize_here(
	const mrpt::poses::CPose3DPDFGaussian& pose)
{
	auto lck = mrpt::lockHelper(stateMtx_);

	params_.initial_pose.emplace(pose);
	// Only if we were already running, reset and restart with a
	// relocalization:
	if (state_.fsm_state == State::RUNNING)
	{
		state_.fsm_state = State::TO_BE_INITIALIZED;
	}
}

mrpt::poses::CPose3DPDFParticles::Ptr
	PFLocalizationCore::getLastPoseEstimation() const
{
	auto lck = mrpt::lockHelper(stateMtx_);
	return state_.lastResult;
}

void PFLocalizationCore::internal_fill_state_lastResult()
{
	if (state_.pdf2d)
	{
		// Convert SE(2) -> SE(3)
		state_.lastResult = mrpt::poses::CPose3DPDFParticles::Create();
		const size_t N = state_.pdf2d->size();
		state_.lastResult->resetDeterministic({}, N);
		for (size_t i = 0; i < N; i++)
		{
			auto& trg = state_.lastResult->m_particles.at(i);
			const auto& org = state_.pdf2d->m_particles.at(i);

			trg.log_w = org.log_w;
			trg.d = mrpt::math::TPose3D(org.d);
		}
	}
	else if (state_.pdf3d)
	{
		// Make a copy:
		state_.lastResult = mrpt::poses::CPose3DPDFParticles::Create();
		*state_.lastResult = *state_.pdf3d;
	}

	MRPT_LOG_DEBUG_STREAM(
		"internal_fill_state_lastResult: " << state_.lastResult->asString());
}

void PFLocalizationCore::set_fake_odometry_increment(
	const mrpt::poses::CPose3D& incrPose)
{
	state_.nextFakeOdometryIncrPose = incrPose;
}

std::optional<mrpt::poses::CPose3DPDFGaussian>
	PFLocalizationCore::get_gnss_pose_prediction()
{
	if (!state_.georeferencing) return {};

	auto gps = get_last_gnss_obs();

	if (!gps) return {};

	const auto gga = gps->getMsgByClassPtr<mrpt::obs::gnss::Message_NMEA_GGA>();
	if (!gga) return {};

	const auto coords = gga->getAsStruct<mrpt::topography::TGeodeticCoords>();

	/* Scheme of transformations:
	 *
	 * enu_origin (+) T_enu_to_map = map_xyz_origin
	 *
	 * {}^{map}P_{meas} = {}^{map}T_{enu} {}^{enu}P_{meas}
	 *
	 */
	const auto T_map2enu = -(state_.georeferencing->T_enu_to_map.mean);

	// current GNSS measurement (ENU frame)
	mrpt::math::TPoint3D P_enu_meas;
	mrpt::topography::geodeticToENU_WGS84(
		coords, P_enu_meas, state_.georeferencing->geo_coord);

	const mrpt::math::TPoint3D P_map_meas = T_map2enu.composePoint(P_enu_meas);

	mrpt::poses::CPose3DPDFGaussian gnssMeasInMap;

	gnssMeasInMap.mean = mrpt::poses::CPose3D::FromTranslation(P_map_meas);

	gnssMeasInMap.cov.fill(0);
	if (gps->covariance_enu.has_value())
	{
		const auto& gpsCov = *gps->covariance_enu;

		// XYZ: copy from GPS obs:
		gnssMeasInMap.cov.block(0, 0, 3, 3) = gpsCov.asEigen();
	}
	else
	{
		// Default uncertainty:
		MRPT_LOG_WARN(
			"Initializing from GNSS measurement without "
			"covariance_enu, thus using default uncertainty");

		// XYZ: default values:
		gnssMeasInMap.cov(0, 0) = gnssMeasInMap.cov(1, 1) =
			gnssMeasInMap.cov(2, 2) = mrpt::square(5.0);
	}
	// Yaw: large uncertainty:
	gnssMeasInMap.cov(3, 3) = mrpt::square(M_PI);

	return gnssMeasInMap;
}
