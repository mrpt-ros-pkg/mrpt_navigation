/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2026, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt_pointcloud_pipeline/ObstacleConfirmation.h>

#include <array>
#include <initializer_list>

using mrpt::maps::CSimplePointsMap;
using mrpt::poses::CPose3D;
using mrpt_pointcloud_pipeline::ObstacleConfirmation;
using Ids = ObstacleConfirmation::ObservationIds;

namespace
{
CSimplePointsMap points(std::initializer_list<std::array<float, 2>> xy)
{
	CSimplePointsMap m;
	for (const auto& p : xy)
	{
		m.insertPoint(p[0], p[1], 0.3f);
	}
	return m;
}

// One sensor at 10 Hz: each update uses only its latest scan.
Ids scan(int i) { return {{"lidar", 0.1 * i}}; }
}  // namespace

TEST(ObstacleConfirmation, OneOffArtifactIsNeverReported)
{
	ObstacleConfirmation c;
	for (int i = 0; i < 10; i++)
	{
		const auto cand = i == 5 ? points({{2.0f, 0.0f}}) : points({});
		CSimplePointsMap out;
		c.update(cand, CPose3D(), 0.1 * i, scan(i), out);
		EXPECT_EQ(out.size(), 0u) << "update " << i;
	}
}

TEST(ObstacleConfirmation, StaticObstacleConfirmedWhileMoving)
{
	ObstacleConfirmation c;
	// The robot drives toward an obstacle at world x=5 at 1 m/s:
	for (int i = 0; i < 10; i++)
	{
		const double robotX = 0.1 * i;
		const auto cand = points({{static_cast<float>(5.0 - robotX), 0.0f}});
		CSimplePointsMap out;
		c.update(cand, CPose3D(robotX, 0, 0, 0, 0, 0), 0.1 * i, scan(i), out);
		// Reported from the second scan on, at its current (robot frame) place:
		ASSERT_EQ(out.size(), i == 0 ? 0u : 1u) << "update " << i;
		if (i > 0)
		{
			EXPECT_NEAR(out.getPointsBufferRef_x()[0], 5.0 - robotX, 1e-4);
		}
	}
}

TEST(ObstacleConfirmation, SharedObservationDoesNotConfirmItself)
{
	ObstacleConfirmation c;
	// Two sensors; updates alternate the newest scan of one of them, and
	// an artifact of scan "B@0.05" is in the two updates that use it:
	const Ids u0 = {{"A", 0.00}, {"B", -0.05}};
	const Ids u1 = {{"A", 0.00}, {"B", 0.05}};
	const Ids u2 = {{"A", 0.10}, {"B", 0.05}};
	const Ids u3 = {{"A", 0.10}, {"B", 0.15}};
	const auto empty = points({});
	const auto artifact = points({{1.5f, 0.5f}});
	CSimplePointsMap out;
	c.update(empty, CPose3D(), 0.00, u0, out);
	c.update(artifact, CPose3D(), 0.05, u1, out);
	c.update(artifact, CPose3D(), 0.10, u2, out);
	EXPECT_EQ(out.size(), 0u);
	// A real obstacle, seen in disjoint observations, is confirmed:
	c.update(points({{3.0f, 0.0f}}), CPose3D(), 0.15, u3, out);
	EXPECT_EQ(out.size(), 0u);
	const Ids u4 = {{"A", 0.20}, {"B", 0.15}};
	const Ids u5 = {{"A", 0.20}, {"B", 0.25}};
	c.update(points({{3.0f, 0.0f}}), CPose3D(), 0.20, u4, out);
	EXPECT_EQ(out.size(), 0u);	// u3 is not disjoint, u2 had no point there
	c.update(points({{3.0f, 0.0f}}), CPose3D(), 0.25, u5, out);
	EXPECT_EQ(out.size(), 1u);	// confirmed by u3
}

TEST(ObstacleConfirmation, WalkingPersonStaysConfirmedAndVanishesAtOnce)
{
	ObstacleConfirmation c;
	c.params.radius = 0.2f;
	// A person crossing at 1.5 m/s in front of a still robot:
	for (int i = 0; i < 8; i++)
	{
		const auto y = static_cast<float>(-1.0 + 0.15 * i);
		CSimplePointsMap out;
		c.update(points({{2.0f, y}}), CPose3D(), 0.1 * i, scan(i), out);
		EXPECT_EQ(out.size(), i == 0 ? 0u : 1u) << "update " << i;
	}
	// Once gone, it is no longer reported:
	CSimplePointsMap out;
	c.update(points({}), CPose3D(), 0.8, scan(8), out);
	EXPECT_EQ(out.size(), 0u);
}

TEST(ObstacleConfirmation, OldUpdatesAreForgotten)
{
	ObstacleConfirmation c;
	c.params.max_age = 0.5;
	CSimplePointsMap out;
	c.update(points({{2.0f, 0.0f}}), CPose3D(), 0.0, scan(0), out);
	c.update(points({{2.0f, 0.0f}}), CPose3D(), 1.0, scan(10), out);
	EXPECT_EQ(out.size(), 0u);
	EXPECT_EQ(c.size(), 1u);
}
