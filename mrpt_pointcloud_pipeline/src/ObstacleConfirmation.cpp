/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2026, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include <mrpt_pointcloud_pipeline/ObstacleConfirmation.h>

#include <algorithm>

namespace mrpt_pointcloud_pipeline
{
namespace
{
bool disjoint(
	const ObstacleConfirmation::ObservationIds& a, const ObstacleConfirmation::ObservationIds& b)
{
	return std::none_of(a.begin(), a.end(), [&b](const auto& id) { return b.count(id) != 0; });
}
}  // namespace

void ObstacleConfirmation::update(
	const mrpt::maps::CPointsMap& candidates, const mrpt::poses::CPose3D& robotPose, double t,
	const ObservationIds& ids, mrpt::maps::CPointsMap& confirmed)
{
	// Forget old updates (and keep the order if the clock goes back):
	while (!history_.empty() && (t - history_.front().t > params.max_age || history_.front().t > t))
	{
		history_.pop_front();
	}

	// The latest past update with no observation in common:
	const PastUpdate* ref = nullptr;
	for (auto it = history_.rbegin(); it != history_.rend(); ++it)
	{
		if (disjoint(it->ids, ids))
		{
			ref = &*it;
			break;
		}
	}

	PastUpdate cur;
	cur.t = t;
	cur.ids = ids;

	const auto& xs = candidates.getPointsBufferRef_x();
	const auto& ys = candidates.getPointsBufferRef_y();
	const auto& zs = candidates.getPointsBufferRef_z();
	const float r2 = params.radius * params.radius;
	for (size_t i = 0; i < xs.size(); i++)
	{
		double gx = 0;
		double gy = 0;
		double gz = 0;
		robotPose.composePoint(xs[i], ys[i], zs[i], gx, gy, gz);
		const auto p = mrpt::math::TPoint3Df(
			static_cast<float>(gx), static_cast<float>(gy), static_cast<float>(gz));
		cur.points.insertPoint(p);

		if (!ref || ref->points.empty())
		{
			continue;
		}
		float distSqr = 0;
		ref->points.kdTreeClosestPoint3D(p.x, p.y, p.z, distSqr);
		if (distSqr <= r2)
		{
			confirmed.insertPoint(xs[i], ys[i], zs[i]);
		}
	}

	history_.push_back(std::move(cur));
}

}  // namespace mrpt_pointcloud_pipeline
