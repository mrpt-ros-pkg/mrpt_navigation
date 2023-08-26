/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2023, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include <mrpt/config/CConfigFile.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/opengl/CEllipsoid2D.h>
#include <mrpt/opengl/CEllipsoid3D.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/ros2bridge/map.h>
#include <mrpt/system/filesystem.h>
#include <mrpt_pf_localization/mrpt_pf_localization_core.h>

#include <chrono>
#include <thread>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::gui;
using namespace mrpt::bayes;
using namespace mrpt::poses;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::img;
using namespace mrpt::config;
using namespace std;

using mrpt::maps::CLandmarksMap;
using mrpt::maps::COccupancyGridMap2D;
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

void PFLocalizationCore::Parameters::load_from(
	const mrpt::containers::yaml& params)
{
	MCP_LOAD_OPT(params, gui_enable);
	MCP_LOAD_REQ(params, use_se3_pf);
	MCP_LOAD_OPT(params, gui_camera_follow_robot);

	// motion_model_2d
	MRPT_TODO("Load motion model params!");

	// motion_model_no_odom_2d

	// motion_model_3d

	// motion_model_no_odom_3d

	// initial_pose: check minimum required fields, if given via YAML.
	if (params.has("initial_pose"))
	{
		using mrpt::square;

		ASSERT_(params["initial_pose"].isMap());
		ASSERT_(params["initial_pose"]["mean"].isMap());
		ASSERT_(params["initial_pose"]["mean"]["x"].isScalar());
		ASSERT_(params["initial_pose"]["mean"]["y"].isScalar());
		ASSERT_(params["initial_pose"]["std_x"].isScalar());
		ASSERT_(params["initial_pose"]["std_y"].isScalar());

		auto ipP = params["initial_pose"];
		auto m = ipP["mean"];
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

	//
	MCP_LOAD_OPT(params, initial_particle_count);

	// The map is not loaded here, but from independent methods in the parent
	// class.
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
	// Check if we have everything we need to get going:
	if (params_.initial_pose.has_value() && params_.metric_map)
	{
		// Move:
		state_.fsm_state = State::TO_BE_INITIALIZED;

		MRPT_LOG_INFO(
			"Required configuration has been set, changing to "
			"'State::TO_BE_INITIALIZED'");
		return;
	}

	// We don't have parameters / map yet. Do nothing:
	MRPT_LOG_THROTTLE_WARN(
		3.0,
		"Doing nothing, since state is UNINITIALIZED yet. "
		"Probably parameters or the map has not been loaded yet. Refer "
		"to package documentation.");
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

	// Get desired initial pose uncertainty:
	MRPT_LOG_INFO_STREAM(
		"[onStateToBeInitialized] Initial pose: " << *params_.initial_pose);

	const auto [pCov, pMean] = params_.initial_pose->getCovarianceAndMean();

	const double stdX = std::sqrt(pCov(0, 0));
	const double stdY = std::sqrt(pCov(1, 1));
	const double stdZ = std::sqrt(pCov(2, 2));
	const double stdYaw = std::sqrt(pCov(3, 3));
	const double stdPitch = std::sqrt(pCov(4, 4));
	const double stdRoll = std::sqrt(pCov(5, 5));

	const double nStds = 2.0;  // number of sigmas ("quantiles")

	const auto pMin = mrpt::math::TPose3D(
		pMean.x() - nStds * stdX, pMean.y() - nStds * stdY,
		pMean.z() - nStds * stdZ, pMean.yaw() - nStds * stdYaw,
		pMean.pitch() - nStds * stdPitch, pMean.roll() - nStds * stdRoll);

	const auto pMax = mrpt::math::TPose3D(
		pMean.x() + nStds * stdX, pMean.y() + nStds * stdY,
		pMean.z() + nStds * stdZ, pMean.yaw() + nStds * stdYaw,
		pMean.pitch() + nStds * stdPitch, pMean.roll() + nStds * stdRoll);

	if (auto gridMap = _.metric_map->mapByClass<COccupancyGridMap2D>();
		gridMap && _.pdf2d)
	{
		// initialize over free space only:
		const float gridFreenessThreshold = 0.7f;
		_.pdf2d->resetUniformFreeSpace(
			gridMap.get(), gridFreenessThreshold,
			params_.initial_particle_count, pMin.x, pMax.x, pMin.y, pMax.y,
			pMin.yaw, pMax.yaw);
	}
	else
	{
		if (_.pdf2d)
			_.pdf2d->resetUniform(
				pMin.x, pMax.x, pMin.y, pMax.y, pMin.yaw, pMax.yaw,
				params_.initial_particle_count);
		else
			_.pdf3d->resetUniform(pMin, pMax, params_.initial_particle_count);
	}
}

void PFLocalizationCore::onStateRunning()
{
	auto tle = mrpt::system::CTimeLoggerEntry(profiler_, "onStateRunning");

	// Collect observations since last execution and build "action" and
	// "observations" for the Bayes filter:
	mrpt::obs::CSensoryFrame sf;  // sorted, and thread-safe copy of all obs.
	mrpt::Clock::time_point sfLastTimeStamp;
	{
		// If we have multiple observations of the same sensor for this single
		// PF step, discard all but the latest one.
		// Temporary storage of observations:
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
						"sensorLabel='%s' and different classes: '%s' vs '%s'",
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

	// Do we have *any* observation?
	if (sf.empty())
	{
		MRPT_LOG_THROTTLE_WARN(
			2.0, "No observation in the input queue at all...");

		// Default timestamp to "now":
		sfLastTimeStamp = mrpt::Clock::now();
	}

	// --------------------------------------------

	// "Action"
	// ------------------------
	// Build it from odometry, if we have, or from "fake null odometry" instead
	mrpt::obs::CObservationOdometry::Ptr odomObs;
	// get the LAST odometry reading, if we have many:
	for (size_t i = 0;
		 auto o = sf.getObservationByClass<mrpt::obs::CObservationOdometry>(i);
		 i++)
	{
		odomObs = o;
	}

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
		}
		else
		{
			// TODO: Use 3D odometry observations?
			// Does it make sense for some application?

			odomMove3D.value()->computeFromOdometry(
				mrpt::poses::CPose3D(incOdoPose), params_.motion_model_3d);
		}
	}
	else
	{
		// Use fake "null movement" motion model with the special uncertainty:
		if (odomMove2D.has_value())
		{
			odomMove2D.value()->computeFromOdometry(
				mrpt::poses::CPose2D::Identity(),
				params_.motion_model_no_odom_2d);
		}
		else
		{
			odomMove3D.value()->computeFromOdometry(
				mrpt::poses::CPose3D::Identity(),
				params_.motion_model_no_odom_3d);
		}
	}

	mrpt::obs::CActionCollection actions;
	if (is_3D)
		actions.insertPtr(*odomMove3D);
	else
		actions.insertPtr(*odomMove2D);

	// Make sure params are up-to-date in the PF
	// (they may change on-the-fly by users):
	// -----------------------------------------
	state_.pf.m_options = params_.pf_options;
	TMonteCarloLocalizationParams pdfPredictionOptions;
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

	// Process PF
	// ------------------------
	mrpt::bayes::CParticleFilterCapable& pfc =
		state_.pdf2d
			? static_cast<mrpt::bayes::CParticleFilterCapable&>(*state_.pdf2d)
			: static_cast<mrpt::bayes::CParticleFilterCapable&>(*state_.pdf3d);

	state_.pf.executeOn(pfc, &actions, &sf, &state_.pf_stats);

	// Collect further output stats:
	// ------------------------------
	state_.time_last_update = sfLastTimeStamp;

	// GUI:
	// -----------
	// Init optional debug GUI:
	if (params_.gui_enable) update_gui(sf);
}

bool PFLocalizationCore::set_map_from_simple_map(
	const std::string& map_config_ini_file, const std::string& simplemap_file)
{
	auto lck = mrpt::lockHelper(stateMtx_);

	ASSERT_FILE_EXISTS_(map_config_ini_file);
	ASSERT_FILE_EXISTS_(simplemap_file);

	mrpt::config::CConfigFile cfg(map_config_ini_file);

	params_.metric_map = mrpt::maps::CMultiMetricMap::Create();

	bool ok = mrpt::ros2bridge::MapHdl::loadMap(
		*params_.metric_map, cfg, simplemap_file, "metricMap");

	return ok;
}

/* Load all params from a YAML source.
 *  This method loads all required params and put the system from
 * UNINITIALIZED into TO_BE_INITIALIZED.
 */
void PFLocalizationCore::init_from_yaml(const mrpt::containers::yaml& params)
{
	MRPT_LOG_INFO("Called init_from_yaml()");

	auto lck = mrpt::lockHelper(stateMtx_);

	// Load all required and optional params:
	params_.load_from(params);

	// Now, parse the map, if given in the YAML in any of the supported
	// formats:

	// if (0)
	// set_map_from_simple_map();

	// set_map_from_metric_map_file();

	// set_map_from_ros_yaml();
}

void PFLocalizationCore::init_gui()
{
	MRPT_LOG_INFO("init3DDebug");

	if (!params_.gui_enable) return;
	if (win3D_) return;	 // already done.

	win3D_ = CDisplayWindow3D::Create("mrpt_pf_localization", 1000, 600);
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

void PFLocalizationCore::update_gui(const CSensoryFrame& sf)
{
	auto tle = mrpt::system::CTimeLoggerEntry(profiler_, "show3DDebug");

#if !MRPT_HAS_WXWIDGETS
	return;	 // we don't have built-in GUI!
#endif

	if (!params_.gui_enable) return;

	// Create 3D window if requested:
	if (!win3D_) init_gui();

	TTimeStamp cur_obs_timestamp = INVALID_TIMESTAMP;
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
		fp.color = TColorf(.8f, .8f, .8f);
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

			CSimplePointsMap map;
			static CSimplePointsMap last_map;

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
	state_.fsm_state = State::TO_BE_INITIALIZED;
}
