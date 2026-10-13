/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2026, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include <mrpt/maps/CSimplePointsMap.h>

#include <mrpt_tps_astar_planner/obstacle_regions.hpp>
#include <utility>

namespace mrpt_tps_astar_planner
{
std::vector<mrpt::math::TPolygon2D> to_polygons(
	const std::vector<geometry_msgs::msg::Polygon>& polygons)
{
	std::vector<mrpt::math::TPolygon2D> out;
	for (const auto& p : polygons)
	{
		if (p.points.size() < 3)
		{
			continue;
		}
		auto& poly = out.emplace_back();
		for (const auto& v : p.points)
		{
			poly.emplace_back(v.x, v.y);
		}
	}
	return out;
}

mrpt::maps::CPointsMap::Ptr without_points_in(
	const mrpt::maps::CPointsMap::Ptr& pts, const std::vector<mrpt::math::TPolygon2D>& regions)
{
	if (!pts || regions.empty())
	{
		return pts;
	}

	// Bounding boxes, for a quick rejection test:
	std::vector<std::pair<mrpt::math::TPoint2D, mrpt::math::TPoint2D>> bboxes(regions.size());
	for (size_t i = 0; i < regions.size(); i++)
	{
		regions[i].getBoundingBox(bboxes[i].first, bboxes[i].second);
	}
	const auto inside_any = [&](float x, float y)
	{
		for (size_t i = 0; i < regions.size(); i++)
		{
			const auto& [bbMin, bbMax] = bboxes[i];
			if (x < bbMin.x || x > bbMax.x || y < bbMin.y || y > bbMax.y)
			{
				continue;
			}
			if (regions[i].contains({x, y}))
			{
				return true;
			}
		}
		return false;
	};

	const auto& xs = pts->getPointsBufferRef_x();
	const auto& ys = pts->getPointsBufferRef_y();
	const auto& zs = pts->getPointsBufferRef_z();

	auto out = mrpt::maps::CSimplePointsMap::Create();
	out->reserve(xs.size());
	for (size_t i = 0; i < xs.size(); i++)
	{
		if (!inside_any(xs[i], ys[i]))
		{
			out->insertPointFast(xs[i], ys[i], zs[i]);
		}
	}
	if (out->size() == xs.size())
	{
		return pts;
	}
	out->mark_as_modified();
	return out;
}

}  // namespace mrpt_tps_astar_planner
