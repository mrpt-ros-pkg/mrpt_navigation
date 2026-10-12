/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2026, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt_pointcloud_pipeline/ObstacleMemory.h>

#include <cmath>

using mrpt::maps::CSimplePointsMap;
using mrpt::poses::CPose3D;
using mrpt_pointcloud_pipeline::ObstacleMemory;

namespace
{
// Blind zone right in front of the robot, from x=1.4 to x=2.0:
ObstacleMemory make_memory()
{
	ObstacleMemory m;
	m.params.region_min_x = 1.4f;
	m.params.region_max_x = 2.0f;
	m.params.region_min_y = -0.8f;
	m.params.region_max_y = 0.8f;
	m.params.max_age = 5.0;
	return m;
}

// A sensor that only sees an obstacle at world x=5 from farther than this:
constexpr double kObstacleX = 5.0;
constexpr double kMinVisibleRange = 2.0;

CSimplePointsMap sense(double robotX)
{
	CSimplePointsMap pts;
	const double range = kObstacleX - robotX;
	if (range >= kMinVisibleRange)
	{
		pts.insertPoint(static_cast<float>(range), 0.0f, 0.5f);
	}
	return pts;
}
}  // namespace

TEST(ObstacleMemory, RemembersObstacleInBlindZone)
{
	auto m = make_memory();
	double t = 0;
	// Approach the obstacle until it's 1.5 m ahead (inside the blind zone):
	for (double x = 0; x <= 3.5 + 1e-6; x += 0.1, t += 0.1)
	{
		const CPose3D pose(x, 0, 0, 0, 0, 0);
		const auto live = sense(x);
		CSimplePointsMap out;
		m.update(live, pose, t, out);

		const double range = kObstacleX - x;
		if (range < kMinVisibleRange && range >= 1.4)
		{
			// Not seen, but remembered at the right place:
			ASSERT_EQ(out.size(), 1U) << "x=" << x;
			float px = 0;
			float py = 0;
			float pz = 0;
			out.getPoint(0, px, py, pz);
			EXPECT_NEAR(px, range, 0.11) << "x=" << x;
			EXPECT_NEAR(py, 0.0, 1e-3);
		}
		else
		{
			// Seen live (or not yet relevant): nothing to add.
			EXPECT_EQ(out.size(), 0U) << "x=" << x;
		}
	}
}

TEST(ObstacleMemory, StaysWhileStoppedThenExpires)
{
	auto m = make_memory();
	double t = 0;
	double x = 0;
	for (; x <= 3.5 + 1e-6; x += 0.1, t += 0.1)
	{
		CSimplePointsMap out;
		m.update(sense(x), CPose3D(x, 0, 0, 0, 0, 0), t, out);
	}
	// Stopped in front of it: remembered up to max_age.
	const double tLastSeen = 3.0;  // x=3.0 was the last time it was visible
	for (; t < tLastSeen + m.params.max_age - 0.05; t += 0.1)
	{
		CSimplePointsMap out;
		m.update(sense(3.5), CPose3D(3.5, 0, 0, 0, 0, 0), t, out);
		ASSERT_EQ(out.size(), 1U) << "t=" << t;
	}
	t = tLastSeen + m.params.max_age + 0.1;
	CSimplePointsMap out;
	m.update(sense(3.5), CPose3D(3.5, 0, 0, 0, 0, 0), t, out);
	EXPECT_EQ(out.size(), 0U);
	EXPECT_EQ(m.size(), 0U);
}

TEST(ObstacleMemory, ForgetsWhatIsVisiblyGone)
{
	auto m = make_memory();
	// An obstacle seen 2.5 m ahead (outside the blind zone, within margin):
	CSimplePointsMap seen;
	seen.insertPoint(2.5f, 0.0f, 0.5f);
	CSimplePointsMap out;
	m.update(seen, CPose3D(), 0.0, out);
	EXPECT_EQ(m.size(), 1U);

	// It goes away while still in view: dropped after unseen_timeout.
	const CSimplePointsMap empty;
	for (double t = 0.1; t <= m.params.unseen_timeout + 0.15; t += 0.1)
	{
		out.clear();
		m.update(empty, CPose3D(), t, out);
		EXPECT_EQ(out.size(), 0U);
	}
	EXPECT_EQ(m.size(), 0U);
}

TEST(ObstacleMemory, IgnoresFarPointsAndFollowsRotation)
{
	auto m = make_memory();
	CSimplePointsMap seen;
	seen.insertPoint(10.0f, 0.0f, 0.5f);  // far: not stored
	seen.insertPoint(2.2f, 0.0f, 0.5f);	 // near the zone: stored
	CSimplePointsMap out;
	m.update(seen, CPose3D(), 0.0, out);
	EXPECT_EQ(m.size(), 1U);

	// Turn in place 90 deg left: the point is now on the right, out of the
	// zone, and it's no longer seen: forgotten.
	const CSimplePointsMap empty;
	out.clear();
	m.update(empty, CPose3D(0, 0, 0, M_PI / 2, 0, 0), 0.1, out);
	EXPECT_EQ(out.size(), 0U);

	// Back to the original heading, moved forward so it is in the zone:
	// it was forgotten while visible and unseen, so it's not reported.
	out.clear();
	m.update(empty, CPose3D(0.6, 0, 0, 0, 0, 0), 2.0, out);
	EXPECT_EQ(out.size(), 0U);
}
