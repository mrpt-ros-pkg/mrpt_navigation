/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2026, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <mrpt_tps_astar_planner/path_validation.hpp>

using namespace mrpt_tps_astar_planner::path_validation;
using mrpt::math::TPoint2D;
using mrpt::math::TPose2D;

namespace
{
// 1.0 x 0.5 m footprint centered at the origin
mrpt::math::TPolygon2D box()
{
	mrpt::math::TPolygon2D p;
	p.emplace_back(-0.5, -0.25);
	p.emplace_back(0.5, -0.25);
	p.emplace_back(0.5, 0.25);
	p.emplace_back(-0.5, 0.25);
	return p;
}

std::vector<TPose2D> straight(double x0, double x1, double step)
{
	std::vector<TPose2D> v;
	for (double x = x0; x <= x1 + 1e-9; x += step)
	{
		v.emplace_back(x, 0.0, 0.0);
	}
	return v;
}

ObstacleIndex indexOf(const std::vector<TPoint2D>& pts)
{
	mrpt::maps::CSimplePointsMap m;
	for (const auto& p : pts)
	{
		m.insertPoint(static_cast<float>(p.x), static_cast<float>(p.y), 0.f);
	}
	ObstacleIndex idx(0.5);
	idx.add(m);
	return idx;
}
}  // namespace

TEST(PathValidation, DistanceToFootprint)
{
	const auto fp = box();
	EXPECT_NEAR(distance_to_footprint(fp, {0.0, 0.0}), -0.25, 1e-9);  // inside
	EXPECT_NEAR(distance_to_footprint(fp, {0.5, 0.0}), 0.0, 1e-9);	// on the border
	EXPECT_NEAR(distance_to_footprint(fp, {1.5, 0.0}), 1.0, 1e-9);	// outside
	EXPECT_NEAR(distance_to_footprint(fp, {1.5, 1.25}), std::hypot(1.0, 1.0), 1e-9);  // corner
}

TEST(PathValidation, ArcLengthsAreCumulative)
{
	const auto s = arc_lengths({{0, 0, 0}, {3, 4, 0}, {3, 4, 1}, {3, 6, 0}});
	ASSERT_EQ(s.size(), 4U);
	EXPECT_DOUBLE_EQ(s[0], 0.0);
	EXPECT_DOUBLE_EQ(s[1], 5.0);
	EXPECT_DOUBLE_EQ(s[2], 5.0);
	EXPECT_DOUBLE_EQ(s[3], 7.0);
	EXPECT_TRUE(arc_lengths({}).empty());
}

TEST(PathValidation, NearestPoseSearchesAroundHint)
{
	const auto path = straight(0, 20, 0.1);	 // 201 poses
	EXPECT_EQ(nearest_pose(path, {5.04, 0.3}, 50), 50U);
	EXPECT_EQ(nearest_pose(path, {5.06, 0.3}, 50), 51U);
	// Behind the hint (a few poses) and ahead (many):
	EXPECT_EQ(nearest_pose(path, {4.6, 0}, 50), 46U);
	EXPECT_EQ(nearest_pose(path, {10.0, 0}, 50), 100U);
	// Out of the search window: stays within it
	EXPECT_LT(nearest_pose(path, {19.0, 0}, 10), 100U);
}

TEST(PathValidation, ObstacleIndexMargins)
{
	const auto fp = box();
	const auto idx = indexOf({{0.6, 0.0}});	 // 0.1 m ahead of the front border
	const TPose2D pose(0, 0, 0);
	EXPECT_TRUE(idx.collides(pose, fp, 0.6, 0.2).has_value());
	EXPECT_FALSE(idx.collides(pose, fp, 0.6, 0.05).has_value());
	// A negative margin tolerates obstacles slightly inside the footprint:
	const auto inside = indexOf({{0.45, 0.0}});	 // 5 cm inside
	EXPECT_TRUE(inside.collides(pose, fp, 0.6, 0.0).has_value());
	EXPECT_FALSE(inside.collides(pose, fp, 0.6, -0.1).has_value());
	// Rotated robot:
	EXPECT_FALSE(idx.collides(TPose2D(0, 0, M_PI / 2), fp, 0.6, 0.0).has_value());
}

TEST(PathValidation, NegativeCoordinatesDoNotAliasCells)
{
	// Obstacles in far cells with negative indices must not be found from
	// other cells (hash keys of cells with negative indices).
	const auto fp = box();
	const auto idx = indexOf({{-100.0, -50.0}, {-100.0, 50.0}, {100.0, -50.0}});
	EXPECT_FALSE(idx.collides(TPose2D(0, 0, 0), fp, 0.6, 0.5).has_value());
	EXPECT_TRUE(idx.collides(TPose2D(-100.0, -50.0, 0), fp, 0.6, 0.0).has_value());
	EXPECT_TRUE(idx.collides(TPose2D(100.0, -50.0, 0), fp, 0.6, 0.0).has_value());
}

TEST(PathValidation, FirstCollidingPoseChecksBetweenSamples)
{
	const auto fp = box();
	// Obstacle at x=2.5 on the path; poses every 2 m would skip over it
	const auto idx = indexOf({{2.5, 0.0}});
	const auto path = straight(0, 6, 2.0);
	const auto hit = first_colliding_pose(path, 0, idx, fp, 0.6, 0.0);
	ASSERT_TRUE(hit.has_value());
	EXPECT_EQ(hit->first, 1U);	// the segment ending at x=2
	EXPECT_NEAR(hit->second.x, 2.5, 1e-6);

	// Starting beyond it: clear
	EXPECT_FALSE(first_colliding_pose(path, 2, idx, fp, 0.6, 0.0).has_value());
	// Obstacle off the path:
	EXPECT_FALSE(first_colliding_pose(path, 0, indexOf({{2.5, 2.0}}), fp, 0.6, 0.0).has_value());
}
