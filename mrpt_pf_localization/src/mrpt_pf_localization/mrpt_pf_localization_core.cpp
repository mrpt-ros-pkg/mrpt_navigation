/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2023, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include <mrpt/core/lock_helper.h>
#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
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

PFLocalizationCore::Parameters::Parameters()
{
	auto cov = mrpt::math::CMatrixDouble66::Identity();
	for (int i = 3; i < 6; i++) cov(i, i) = mrpt::square(1 * M_PI);

	initial_pose =
		mrpt::poses::CPose3DPDFGaussian(mrpt::poses::CPose3D(0, 0, 0), cov);

	// Without odometry: use a large uncertainty for each time step:
	motion_model_default_options.modelSelection =
		mrpt::obs::CActionRobotMovement2D::mmGaussian;
	motion_model_default_options.gaussianModel.minStdXY = 0.10;
	motion_model_default_options.gaussianModel.minStdPHI = 2.0;
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
			// We don't have parameters / map yet. Do nothing.
			MRPT_LOG_THROTTLE_WARN(
				3.0,
				"[step] Doing nothing, since state is UNINITIALIZED yet. "
				"Probably parameters or the map has not been loaded yet. Refer "
				"to package documentation.");
			break;
		case State::TO_BE_INITIALIZED:
			onStateToBeInitialized();
			break;
		case State::RUNNING_STILL:
			break;
		case State::RUNNING_MOVING:
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
	_.fsm_state = State::RUNNING_MOVING;

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
		"[onStateToBeInitialized] Initial pose: " << params_.initial_pose);

	const auto [pCov, pMean] = params_.initial_pose.getCovarianceAndMean();

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

void PFLocalizationCore::onStateRunningMoving()
{
	auto tle =
		mrpt::system::CTimeLoggerEntry(profiler_, "onStateRunningMoving");

	// Collect observations since last execution and build "action" and
	// "observations" for the Bayes filter:
	mrpt::obs::CSensoryFrame sf;  // sorted, and thread-safe copy of all obs.
	mrpt::Clock::time_point sfLastTimeStamp;
	{
		auto lck = mrpt::lockHelper(pendingObsMtx_);
		for (auto& o : state_.pendingObs)
		{
			if (!o) continue;  // who knows...users may be evil :-)
			sf.insert(o);
			sfLastTimeStamp = o->getTimeStamp();
		}
		state_.pendingObs.clear();
	}

	// Do we have *any* observation?
	if (sf.empty())
	{
		MRPT_LOG_THROTTLE_WARN(
			2.0, "No observation in the input queue at all...");
	}

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

	mrpt::obs::CActionCollection actions;
	mrpt::obs::CActionRobotMovement2D odom_move;
	odom_move.timestamp = sfLastTimeStamp;
	if (_odometry)
	{
		if (odom_last_observation_.empty())
		{
			odom_last_observation_ = _odometry->odometry;
		}
		mrpt::poses::CPose2D incOdoPose =
			_odometry->odometry - odom_last_observation_;
		odom_last_observation_ = _odometry->odometry;
		odom_move.computeFromOdometry(incOdoPose, motion_model_options_);
		action->insert(odom_move);
		updateFilter(action, _sf);
	}
	else
	{
		if (use_motion_model_default_options_)
		{
			MRPT_LOG_INFO_STREAM(
				"No odometry at update " << update_counter_
										 << " -> using dummy");
			odom_move.computeFromOdometry(
				mrpt::poses::CPose2D(0, 0, 0), motion_model_default_options_);
			action->insert(odom_move);
			updateFilter(action, _sf);
		}
		else
		{
			MRPT_LOG_INFO_STREAM(
				"No odometry at update " << update_counter_
										 << " -> skipping observation");
		}
	}

	// Process PF
	// ------------------------
	state_.pf.executeOn(
		state_.pdf2d
			? static_cast<mrpt::bayes::CParticleFilterCapable&>(*state_.pdf2d)
			: static_cast<mrpt::bayes::CParticleFilterCapable&>(*state_.pdf3d),
		&actions, &sf, &state_.pf_stats);

	// Collect further output stats:
	// ------------------------------
	state_.time_last_update = sfLastTimeStamp;

	// GUI:
	// -----------
	// Init optional debug GUI:
	if (params_.gui_enable) update_gui(sf);
}

void PFLocalizationCore::onStateRunningStill()
{
	auto tle = mrpt::system::CTimeLoggerEntry(profiler_, "onStateRunningStill");
}

void PFLocalizationCore::init()
{
	MRPT_LOG_INFO_STREAM("ini_file ready " << param_->ini_file);
	ASSERT_FILE_EXISTS_(param_->ini_file);
	MRPT_LOG_INFO_STREAM("ASSERT_FILE_EXISTS_ " << param_->ini_file);
	CConfigFile ini_file;
	ini_file.setFileName(param_->ini_file);
	MRPT_LOG_INFO_STREAM("CConfigFile: " << param_->ini_file);

	// Number of initial particles (if size>1, run the experiments N times)
	std::vector<int> particles_count;

	// Load configuration:
	// -----------------------------------------
	string iniSectionName("LocalizationExperiment");
	update_counter_ = 0;

	// Mandatory entries:
	ini_file.read_vector(
		iniSectionName, "particles_count", std::vector<int>(1, 0),
		particles_count,
		/*Fail if not found*/ true);

	if (param_->map_file.empty())
	{
		param_->map_file = ini_file.read_string(iniSectionName, "map_file", "");
	}

	// Non-mandatory entries:
	SCENE3D_FREQ_ = ini_file.read_int(iniSectionName, "3DSceneFrequency", 10);
	SCENE3D_FOLLOW_ =
		ini_file.read_bool(iniSectionName, "3DSceneFollowRobot", true);

	SHOW_PROGRESS_3D_REAL_TIME_ =
		ini_file.read_bool(iniSectionName, "SHOW_PROGRESS_3D_REAL_TIME", false);

#if !MRPT_HAS_WXWIDGETS
	SHOW_PROGRESS_3D_REAL_TIME_ = false;
#endif

	// Default odometry uncertainty parameters in "odom_params_default_"
	// depending on how fast the robot moves, etc...
	//  Only used for observations-only rawlogs:
	motion_model_default_options_.modelSelection =
		CActionRobotMovement2D::mmGaussian;

	motion_model_default_options_.gaussianModel.minStdXY =
		ini_file.read_double("DummyOdometryParams", "minStdXY", 0.04);
	motion_model_default_options_.gaussianModel.minStdPHI = DEG2RAD(
		ini_file.read_double("DefaultOdometryParams", "minStdPHI", 2.0));

	// Read initial particles distribution; fail if any parameter is not found
	init_PDF_mode =
		ini_file.read_bool(iniSectionName, "init_PDF_mode", false, true);
	init_PDF_min_x =
		ini_file.read_float(iniSectionName, "init_PDF_min_x", 0, true);
	init_PDF_max_x =
		ini_file.read_float(iniSectionName, "init_PDF_max_x", 0, true);
	init_PDF_min_y =
		ini_file.read_float(iniSectionName, "init_PDF_min_y", 0, true);
	init_PDF_max_y =
		ini_file.read_float(iniSectionName, "init_PDF_max_y", 0, true);
	float min_phi = DEG2RAD(
		ini_file.read_float(iniSectionName, "init_PDF_min_phi_deg", -180));
	float max_phi = DEG2RAD(
		ini_file.read_float(iniSectionName, "init_PDF_max_phi_deg", 180));
	mrpt::poses::CPose2D p;
	mrpt::math::CMatrixDouble33 cov;
	cov(0, 0) = fabs(init_PDF_max_x - init_PDF_min_x);
	cov(1, 1) = fabs(init_PDF_max_y - init_PDF_min_y);
	cov(2, 2) =
		min_phi < max_phi ? max_phi - min_phi : (max_phi + 2 * M_PI) - min_phi;
	p.x() = init_PDF_min_x + cov(0, 0) / 2.0;
	p.y() = init_PDF_min_y + cov(1, 1) / 2.0;
	p.phi() = min_phi + cov(2, 2) / 2.0;
	MRPT_LOG_DEBUG_FMT(
		"----------- phi: %4.3f: %4.3f <-> %4.3f, %4.3f\n", p.phi(), min_phi,
		max_phi, cov(2, 2));
	initial_pose_ = mrpt::poses::CPosePDFGaussian(p, cov);
	state_ = INIT;

	configureFilter(ini_file);
	// Metric map options:

	ASSERT_(metric_map_);

	if (!mrpt::ros2bridge::MapHdl::loadMap(
			*metric_map_, ini_file, param_->map_file, "metricMap",
			param_->debug))
	{
		waitForMap();
	}

	initial_particle_count_ = *particles_count.begin();
}

void PFLocalizationCore::configureFilter(const CConfigFile& _configFile)
{
	// PF-algorithm Options:
	// ---------------------------
	CParticleFilter::TParticleFilterOptions pfOptions;
	pfOptions.loadFromConfigFile(_configFile, "PF_options");
	pfOptions.dumpToConsole();

	// PDF Options:
	// ------------------
	TMonteCarloLocalizationParams pdfPredictionOptions;
	pdfPredictionOptions.KLD_params.loadFromConfigFile(
		_configFile, "KLD_options");

	pdf_.clear();

	// PDF Options:
	pdf_.options = pdfPredictionOptions;

	pdf_.options.metricMap = metric_map_;

	// Create the PF object:
	pf_.m_options = pfOptions;
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
