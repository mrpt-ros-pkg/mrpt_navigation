/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2026, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <mrpt_tps_astar_planner/obstacle_regions.hpp>

using mrpt_tps_astar_planner::to_polygons;
using mrpt_tps_astar_planner::without_points_in;

namespace
{
geometry_msgs::msg::Polygon square(float x0, float y0, float size)
{
	geometry_msgs::msg::Polygon p;
	for (const auto& [dx, dy] : {std::pair{0.f, 0.f}, {size, 0.f}, {size, size}, {0.f, size}})
	{
		geometry_msgs::msg::Point32 v;
		v.x = x0 + dx;
		v.y = y0 + dy;
		p.points.push_back(v);
	}
	return p;
}

mrpt::maps::CPointsMap::Ptr points_along_x()
{
	auto pts = mrpt::maps::CSimplePointsMap::Create();
	for (int i = 0; i < 10; i++)
	{
		pts->insertPoint(static_cast<float>(i) + 0.5f, 0.5f, 1.0f);
	}
	return pts;
}
}  // namespace

TEST(ObstacleRegions, RemovesPointsInsideAnyRegion)
{
	const auto pts = points_along_x();
	const auto regions = to_polygons({square(2, 0, 1), square(6, 0, 2)});
	ASSERT_EQ(regions.size(), 2U);

	const auto kept = without_points_in(pts, regions);
	ASSERT_EQ(kept->size(), 7U);
	for (size_t i = 0; i < kept->size(); i++)
	{
		float x = 0;
		float y = 0;
		float z = 0;
		kept->getPoint(i, x, y, z);
		EXPECT_FALSE(x == 2.5f || x == 6.5f || x == 7.5f) << "x=" << x;
		EXPECT_FLOAT_EQ(z, 1.0f);
	}
	// The input is left untouched:
	EXPECT_EQ(pts->size(), 10U);
}

TEST(ObstacleRegions, SameMapIfNothingRemoved)
{
	const auto pts = points_along_x();
	EXPECT_EQ(without_points_in(pts, {}), pts);
	EXPECT_EQ(without_points_in(pts, to_polygons({square(-5, -5, 1)})), pts);
}

TEST(ObstacleRegions, DegeneratePolygonsAreSkipped)
{
	geometry_msgs::msg::Polygon segment;
	segment.points.resize(2);
	EXPECT_TRUE(to_polygons({segment}).empty());
}
