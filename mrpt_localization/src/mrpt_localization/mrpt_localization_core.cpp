/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2023, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt_localization/mrpt_localization_core.h>
#include <ros/console.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace std;

using mrpt::maps::CLandmarksMap;
using mrpt::maps::COccupancyGridMap2D;

PFLocalizationCore::~PFLocalizationCore() {}
PFLocalizationCore::PFLocalizationCore() : state_(NA) {}
void PFLocalizationCore::init()
{
	mrpt::math::CMatrixDouble33 cov;
	cov(0, 0) = 1, cov(1, 1) = 1, cov(2, 2) = 2 * M_PI;
	initial_pose_ =
		mrpt::poses::CPosePDFGaussian(mrpt::poses::CPose2D(0, 0, 0), cov);
	initial_particle_count_ = 1000;

	motion_model_default_options_.modelSelection =
		CActionRobotMovement2D::mmGaussian;
	motion_model_default_options_.gaussianModel.minStdXY = 0.10;
	motion_model_default_options_.gaussianModel.minStdPHI = 2.0;
}

void PFLocalizationCore::initializeFilter()
{
	const auto [cov, mean_point] = initial_pose_.getCovarianceAndMean();

	ROS_INFO(
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
	CActionCollection::Ptr _action, CSensoryFrame::Ptr _sf)
{
	if (state_ == INIT) initializeFilter();
	tictac_.Tic();
	pf_.executeOn(pdf_, _action.get(), _sf.get(), &pf_stats_);
	time_last_update_ = _sf->getObservationByIndex(0)->timestamp;
	update_counter_++;
}

void PFLocalizationCore::observation(
	CSensoryFrame::Ptr _sf, CObservationOdometry::Ptr _odometry)
{
	auto action = CActionCollection::Create();
	CActionRobotMovement2D odom_move;
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
			ROS_INFO_STREAM(
				"No odometry at update " << update_counter_
										 << " -> using dummy");
			odom_move.computeFromOdometry(
				mrpt::poses::CPose2D(0, 0, 0), motion_model_default_options_);
			action->insert(odom_move);
			updateFilter(action, _sf);
		}
		else
		{
			ROS_INFO_STREAM(
				"No odometry at update " << update_counter_
										 << " -> skipping observation");
		}
	}
}
