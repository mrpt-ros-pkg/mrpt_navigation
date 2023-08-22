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
	mrpt::math::CMatrixDouble33 cov;
	cov(0, 0) = 1, cov(1, 1) = 1, cov(2, 2) = mrpt::square(1 * M_PI);
	initial_pose =
		mrpt::poses::CPosePDFGaussian(mrpt::poses::CPose2D(0, 0, 0), cov);

	// Without odometry: use a large uncertainty for each time step:
	motion_model_default_options.modelSelection =
		mrpt::obs::CActionRobotMovement2D::mmGaussian;
	motion_model_default_options.gaussianModel.minStdXY = 0.10;
	motion_model_default_options.gaussianModel.minStdPHI = 2.0;
}

void PFLocalizationCore::on_observation(const mrpt::obs::CObservation::Ptr& obs)
{
	auto lck = mrpt::lockHelper(state_.pendingObsMtx_);
	state_.pendingObs.push_back(obs);
}

// The main API call: executes one PF step, taking into account all the
// parameters and observations gathered so far, updates the optional GUI, etc.
void PFLocalizationCore::step()
{
	auto lck = mrpt::lockHelper(stateMtx_);
	//
}

/** Reset the object to the initial state as if created from scratch */
void PFLocalizationCore::reset()
{
	auto lck = mrpt::lockHelper(stateMtx_);
	//
}

void PFLocalizationCore::initializeFilter()
{
	const auto [cov, mean_point] = initial_pose_.getCovarianceAndMean();

	MRPT_LOG_INFO_FMT(
		"InitializeFilter: %4.3fm, %4.3fm, %4.3frad ", mean_point.x(),
		mean_point.y(), mean_point.phi());

	float min_x = mean_point.x() - cov(0, 0);
	float max_x = mean_point.x() + cov(0, 0);
	float min_y = mean_point.y() - cov(1, 1);
	float max_y = mean_point.y() + cov(1, 1);
	float min_phi = mean_point.phi() - cov(2, 2);
	float max_phi = mean_point.phi() + cov(2, 2);

	if (metric_map_->countMapsByClass<COccupancyGridMap2D>() && !init_PDF_mode)
	{
		pdf_.resetUniformFreeSpace(
			metric_map_->mapByClass<COccupancyGridMap2D>().get(), 0.7f,
			initial_particle_count_, min_x, max_x, min_y, max_y, min_phi,
			max_phi);
	}
	else if (metric_map_->countMapsByClass<CLandmarksMap>() || init_PDF_mode)
	{
		pdf_.resetUniform(
			min_x, max_x, min_y, max_y, min_phi, max_phi,
			initial_particle_count_);
	}
	state_ = RUN;
}

void PFLocalizationCore::updateFilter(
	mrpt::obs::CActionCollection::Ptr _action,
	mrpt::obs::CSensoryFrame::Ptr _sf)
{
	if (state_ == INIT) initializeFilter();
	tictac_.Tic();
	pf_.executeOn(pdf_, _action.get(), _sf.get(), &pf_stats_);
	time_last_update_ = _sf->getObservationByIndex(0)->timestamp;
	update_counter_++;
}

void PFLocalizationCore::observation(
	mrpt::obs::CSensoryFrame::Ptr _sf,
	mrpt::obs::CObservationOdometry::Ptr _odometry)
{
	auto action = mrpt::obs::CActionCollection::Create();
	mrpt::obs::CActionRobotMovement2D odom_move;
	odom_move.timestamp = _sf->getObservationByIndex(0)->timestamp;
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

	if (param_->gui_mrpt) init3DDebug();
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

void PFLocalizationCore::init3DDebug()
{
	MRPT_LOG_INFO("init3DDebug");

	if (!params_.gui_enable) return;
	if (win3D_) return;	 // already done.

	win3D_ = CDisplayWindow3D::Create(
		"pf-localization - The MRPT project", 1000, 600);
	win3D_->setCameraZoom(20);
	win3D_->setCameraAzimuthDeg(-45);

	auto glMap = metric_map_->getVisualization();

	COpenGLScene::Ptr ptr_scene = win3D_->get3DSceneAndLock();
	ptr_scene->insert(glMap);
	ptr_scene->enableFollowCamera(params_.gui_camera_follow_robot);
	win3D_->unlockAccess3DScene();
}

void PFLocalizationCore::show3DDebug(CSensoryFrame::Ptr _observations)
{
	// Create 3D window if requested:
	if (!SHOW_PROGRESS_3D_REAL_TIME_) return;

	TTimeStamp cur_obs_timestamp = INVALID_TIMESTAMP;
	if (_observations->size() > 0)
		cur_obs_timestamp = _observations->getObservationByIndex(0)->timestamp;

	const auto [cov, meanPose] = pdf_.getCovarianceAndMean();

	COpenGLScene::Ptr ptr_scene = win3D_->get3DSceneAndLock();

	win3D_->setCameraPointingToPoint(meanPose.x(), meanPose.y(), 0);

	mrpt::opengl::TFontParams fp;
	fp.color = TColorf(.8f, .8f, .8f);
	fp.vfont_name = "mono";
	fp.vfont_scale = 15;

	win3D_->addTextMessage(
		10, 10,
		mrpt::format(
			"timestamp: %s",
			cur_obs_timestamp != INVALID_TIMESTAMP
				? mrpt::system::dateTimeLocalToString(cur_obs_timestamp).c_str()
				: "(none)"),
		6001, fp);

	win3D_->addTextMessage(
		10, 33,
		mrpt::format("#particles= %7u", static_cast<unsigned int>(pdf_.size())),
		6002, fp);

	win3D_->addTextMessage(
		10, 55,
		mrpt::format(
			"mean pose (x y phi_deg)= %s", meanPose.asString().c_str()),
		6003, fp);

	// The particles:
	{
		CRenderizable::Ptr parts = ptr_scene->getByName("particles");
		if (parts) ptr_scene->removeObject(parts);

		CSetOfObjects::Ptr p = pdf_.getAs3DObject<CSetOfObjects::Ptr>();
		p->setName("particles");
		ptr_scene->insert(p);
	}

	// The particles' cov:
	{
		CRenderizable::Ptr ellip = ptr_scene->getByName("parts_cov");
		if (!ellip)
		{
			auto o = CEllipsoid2D::Create();
			ellip = o;
			ellip->setName("parts_cov");
			ellip->setColor(1, 0, 0, 0.6);

			o->setLineWidth(2);
			o->setQuantiles(3);
			o->set2DsegmentsCount(60);
			ptr_scene->insert(ellip);
		}
		ellip->setLocation(meanPose.x(), meanPose.y(), 0.05);
		dynamic_cast<CEllipsoid2D*>(ellip.get())
			->setCovMatrix(cov.blockCopy<2, 2>());
	}

	// The laser scan:
	{
		CRenderizable::Ptr scan_pts = ptr_scene->getByName("scan");
		if (!scan_pts)
		{
			auto o = CPointCloud::Create();
			scan_pts = o;
			scan_pts->setName("scan");
			scan_pts->setColor(1, 0, 0, 0.9);
			o->enableColorFromZ(false);
			o->setPointSize(4);
			ptr_scene->insert(scan_pts);
		}

		CSimplePointsMap map;
		static CSimplePointsMap last_map;

		CPose3D robot_pose_3D(meanPose);

		map.clear();
		_observations->insertObservationsInto(map);

		dynamic_cast<CPointCloud*>(scan_pts.get())
			->loadFromPointsMap(&last_map);
		dynamic_cast<CPointCloud*>(scan_pts.get())->setPose(robot_pose_3D);
		last_map = map;
	}

	// The camera:
	ptr_scene->enableFollowCamera(true);

	// Views:
	COpenGLViewport::Ptr view1 = ptr_scene->getViewport("main");
	{
		CCamera& cam = view1->getCamera();
		cam.setAzimuthDegrees(-90);
		cam.setElevationDegrees(90);
		cam.setPointingAt(meanPose);
		cam.setZoomDistance(5);
		cam.setOrthogonal();
	}

	win3D_->unlockAccess3DScene();

	// Move camera:
	// win3D_->setCameraPointingToPoint( curRobotPose.x, curRobotPose.y,
	// curRobotPose.z );

	// Update:
	win3D_->forceRepaint();
}
