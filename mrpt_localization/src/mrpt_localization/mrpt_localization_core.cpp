/***********************************************************************************
 * Revised BSD License *
 * Copyright (c) 2014, Markus Bader <markus.bader@tuwien.ac.at> *
 * All rights reserved. *
 *                                                                                 *
 * Redistribution and use in source and binary forms, with or without *
 * modification, are permitted provided that the following conditions are met: *
 *     * Redistributions of source code must retain the above copyright *
 *       notice, this list of conditions and the following disclaimer. *
 *     * Redistributions in binary form must reproduce the above copyright *
 *       notice, this list of conditions and the following disclaimer in the *
 *       documentation and/or other materials provided with the distribution. *
 *     * Neither the name of the Vienna University of Technology nor the *
 *       names of its contributors may be used to endorse or promote products *
 *       derived from this software without specific prior written permission. *
 *                                                                                 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND *
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 **
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE *
 * DISCLAIMED. IN NO EVENT SHALL Markus Bader BE LIABLE FOR ANY *
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES *
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 **
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND *
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 **
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **                       *
 ***********************************************************************************/

#include <mrpt_localization/mrpt_localization_core.h>
#include <mrpt/version.h>

#include <mrpt/maps/COccupancyGridMap2D.h>
using mrpt::maps::COccupancyGridMap2D;

#include <mrpt/maps/CLandmarksMap.h>
using mrpt::maps::CLandmarksMap;

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace std;

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
#if MRPT_VERSION >= 0x199
	const auto [cov, mean_point] = initial_pose_.getCovarianceAndMean();
#else
	mrpt::math::CMatrixDouble33 cov;
	mrpt::poses::CPose2D mean_point;
	initial_pose_.getCovarianceAndMean(cov, mean_point);
#endif

	log_info(
		"InitializeFilter: %4.3fm, %4.3fm, %4.3frad ", mean_point.x(),
		mean_point.y(), mean_point.phi());
	float min_x = mean_point.x() - cov(0, 0);
	float max_x = mean_point.x() + cov(0, 0);
	float min_y = mean_point.y() - cov(1, 1);
	float max_y = mean_point.y() + cov(1, 1);
	float min_phi = mean_point.phi() - cov(2, 2);
	float max_phi = mean_point.phi() + cov(2, 2);

#if MRPT_VERSION >= 0x199
	if (metric_map_.countMapsByClass<COccupancyGridMap2D>() && !init_PDF_mode)
	{
		pdf_.resetUniformFreeSpace(
			metric_map_.mapByClass<COccupancyGridMap2D>().get(), 0.7f,
			initial_particle_count_, min_x, max_x, min_y, max_y, min_phi,
			max_phi);
	}
	else if (metric_map_.countMapsByClass<CLandmarksMap>() || init_PDF_mode)
#else
	if (metric_map_.m_gridMaps.size() && !init_PDF_mode)
	{
		pdf_.resetUniformFreeSpace(
			metric_map_.m_gridMaps[0].get(), 0.7f, initial_particle_count_,
			min_x, max_x, min_y, max_y, min_phi, max_phi);
	}
	else if (metric_map_.m_landmarksMap || init_PDF_mode)
#endif
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
			log_info(
				"No odometry at update %4i -> using dummy", update_counter_);
			odom_move.computeFromOdometry(
				mrpt::poses::CPose2D(0, 0, 0), motion_model_default_options_);
			action->insert(odom_move);
			updateFilter(action, _sf);
		}
		else
		{
			log_info(
				"No odometry at update %4i -> skipping observation",
				update_counter_);
		}
	}
}
