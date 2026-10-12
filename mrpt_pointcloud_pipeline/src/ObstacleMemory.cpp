/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2026, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include <mrpt_pointcloud_pipeline/ObstacleMemory.h>

#include <cmath>

namespace mrpt_pointcloud_pipeline
{
bool ObstacleMemory::in_region(float x, float y, float margin) const
{
	return x >= params.region_min_x - margin && x <= params.region_max_x + margin &&
		   y >= params.region_min_y - margin && y <= params.region_max_y + margin;
}

uint64_t ObstacleMemory::voxel_key(float x, float y, float z) const
{
	// 21 bits per axis: +-100 km at 0.1 m resolution.
	constexpr int64_t kOffset = 1 << 20;
	constexpr uint64_t kMask = (1ULL << 21) - 1;
	const auto idx = [&](float v)
	{ return static_cast<uint64_t>(std::floor(v / params.voxel_size) + kOffset) & kMask; };
	return idx(x) | (idx(y) << 21) | (idx(z) << 42);
}

void ObstacleMemory::update(
	const mrpt::maps::CPointsMap& live, const mrpt::poses::CPose3D& robotPose, double t,
	mrpt::maps::CPointsMap& out)
{
	const auto& xs = live.getPointsBufferRef_x();
	const auto& ys = live.getPointsBufferRef_y();
	const auto& zs = live.getPointsBufferRef_z();

	for (size_t i = 0; i < xs.size(); i++)
	{
		if (!in_region(xs[i], ys[i], params.store_margin))
		{
			continue;
		}
		double gx = 0;
		double gy = 0;
		double gz = 0;
		robotPose.composePoint(xs[i], ys[i], zs[i], gx, gy, gz);

		auto& c = voxels_[voxel_key(gx, gy, gz)];
		c.x = static_cast<float>(gx);
		c.y = static_cast<float>(gy);
		c.z = static_cast<float>(gz);
		c.last_seen = t;
	}

	for (auto it = voxels_.begin(); it != voxels_.end();)
	{
		const Cell& c = it->second;
		double lx = 0;
		double ly = 0;
		double lz = 0;
		robotPose.inverseComposePoint(c.x, c.y, c.z, lx, ly, lz);

		const auto fx = static_cast<float>(lx);
		const auto fy = static_cast<float>(ly);
		const bool inside = in_region(fx, fy, 0);
		const double age = t - c.last_seen;

		const bool forget =
			age > params.max_age ||
			(!inside && (age > params.unseen_timeout || !in_region(fx, fy, params.store_margin)));
		if (forget)
		{
			it = voxels_.erase(it);
			continue;
		}
		// Points seen in this update are already in the live data:
		if (inside && age > 0)
		{
			out.insertPoint(fx, fy, static_cast<float>(lz));
		}
		++it;
	}
}

}  // namespace mrpt_pointcloud_pipeline
