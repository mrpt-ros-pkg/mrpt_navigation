/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2026, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#pragma once

// Helpers of the goal manager to check planned paths against obstacles.

#include <mrpt/core/round.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/wrap2pi.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace mrpt_tps_astar_planner::path_validation
{
// Cumulative arc-length of a path:
inline std::vector<double> arc_lengths(const std::vector<mrpt::math::TPose2D>& poses)
{
	std::vector<double> s(poses.size(), 0.0);
	for (size_t i = 1; i < poses.size(); i++)
	{
		s[i] = s[i - 1] + (poses[i].translation() - poses[i - 1].translation()).norm();
	}
	return s;
}

// Index of the path pose nearest to `p`, searching around `hint`:
inline size_t nearest_pose(
	const std::vector<mrpt::math::TPose2D>& poses, const mrpt::math::TPoint2D& p, size_t hint)
{
	constexpr size_t kBack = 10;
	constexpr size_t kFwd = 80;
	size_t best = hint;
	double bestD2 = std::numeric_limits<double>::infinity();
	const size_t i0 = hint > kBack ? hint - kBack : 0;
	const size_t i1 = std::min(poses.size(), hint + kFwd);
	for (size_t i = i0; i < i1; i++)
	{
		const double d2 = mrpt::square(poses[i].x - p.x) + mrpt::square(poses[i].y - p.y);
		if (d2 < bestD2)
		{
			bestD2 = d2;
			best = i;
		}
	}
	return best;
}

// Signed distance from a point (robot frame) to the robot footprint border
// (negative: inside).
inline double distance_to_footprint(
	const mrpt::math::TPolygon2D& poly, const mrpt::math::TPoint2D& q)
{
	double minD2 = std::numeric_limits<double>::infinity();
	for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
	{
		const auto& a = poly[j];
		const auto& b = poly[i];
		const double ex = b.x - a.x;
		const double ey = b.y - a.y;
		const double l2 = ex * ex + ey * ey;
		const double t =
			l2 > 0 ? std::clamp(((q.x - a.x) * ex + (q.y - a.y) * ey) / l2, 0.0, 1.0) : 0.0;
		minD2 = std::min(
			minD2, mrpt::square(q.x - (a.x + t * ex)) + mrpt::square(q.y - (a.y + t * ey)));
	}
	return poly.contains(q) ? -std::sqrt(minD2) : std::sqrt(minD2);
}

// Obstacle points in a coarse 2D hash grid, for fast footprint checks.
class ObstacleIndex
{
   public:
	explicit ObstacleIndex(double cellSize) : cell_(cellSize) {}

	void add(const mrpt::maps::CPointsMap& pts)
	{
		const auto& xs = pts.getPointsBufferRef_x();
		const auto& ys = pts.getPointsBufferRef_y();
		for (size_t i = 0; i < xs.size(); i++)
		{
			cells_[key(idx(xs[i]), idx(ys[i]))].emplace_back(xs[i], ys[i]);
		}
	}

	// An obstacle closer than `margin` to the footprint border at `pose`
	// (negative: at least that far inside), if any.
	std::optional<mrpt::math::TPoint2D> collides(
		const mrpt::math::TPose2D& pose, const mrpt::math::TPolygon2D& footprint,
		double footprintRadius, double margin) const
	{
		const double r = footprintRadius + std::max(margin, 0.0);
		const int64_t ix0 = idx(pose.x - r);
		const int64_t ix1 = idx(pose.x + r);
		const int64_t iy0 = idx(pose.y - r);
		const int64_t iy1 = idx(pose.y + r);
		const double c = std::cos(pose.phi);
		const double s = std::sin(pose.phi);
		for (int64_t ix = ix0; ix <= ix1; ix++)
		{
			for (int64_t iy = iy0; iy <= iy1; iy++)
			{
				const auto it = cells_.find(key(ix, iy));
				if (it == cells_.end())
				{
					continue;
				}
				for (const auto& p : it->second)
				{
					const double dx = p.x - pose.x;
					const double dy = p.y - pose.y;
					if (dx * dx + dy * dy > r * r)
					{
						continue;
					}
					// In the robot frame:
					const mrpt::math::TPoint2D q(c * dx + s * dy, -s * dx + c * dy);
					if (distance_to_footprint(footprint, q) <= margin)
					{
						return p;
					}
				}
			}
		}
		return std::nullopt;
	}

   private:
	double cell_;
	std::unordered_map<int64_t, std::vector<mrpt::math::TPoint2D>> cells_;

	int64_t idx(double v) const { return static_cast<int64_t>(std::floor(v / cell_)); }
	static int64_t key(int64_t ix, int64_t iy)
	{
		// (shifting as unsigned: left-shifting negative values is undefined)
		return static_cast<int64_t>(
			(static_cast<uint64_t>(ix) << 32) ^ (static_cast<uint64_t>(iy) & 0xffffffffULL));
	}
};

// The first pose index from `from` whose footprint collides, and the obstacle
// point, checking also between samples (every `step` meters).
inline std::optional<std::pair<size_t, mrpt::math::TPoint2D>> first_colliding_pose(
	const std::vector<mrpt::math::TPose2D>& poses, size_t from, const ObstacleIndex& obs,
	const mrpt::math::TPolygon2D& footprint, double footprintRadius, double margin)
{
	constexpr double step = 0.05;  // [m]
	for (size_t i = from; i < poses.size(); i++)
	{
		if (i == from)
		{
			if (const auto p = obs.collides(poses[i], footprint, footprintRadius, margin); p)
			{
				return std::make_pair(i, *p);
			}
			continue;
		}
		const auto& a = poses[i - 1];
		const auto& b = poses[i];
		const double len = (b.translation() - a.translation()).norm();
		const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
		const double dphi = mrpt::math::wrapToPi(b.phi - a.phi);
		for (int k = 1; k <= n; k++)
		{
			const double t = static_cast<double>(k) / n;
			const mrpt::math::TPose2D q(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.phi + t * dphi);
			if (const auto p = obs.collides(q, footprint, footprintRadius, margin); p)
			{
				return std::make_pair(i, *p);
			}
		}
	}
	return std::nullopt;
}

}  // namespace mrpt_tps_astar_planner::path_validation
