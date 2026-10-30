/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2026, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/poses/CPose3D.h>

#include <deque>
#include <set>
#include <string>
#include <utility>

namespace mrpt_pointcloud_pipeline
{
/** Confirms obstacle points with independent sensor data.
 *
 *  A candidate point is reported only if the candidates of the latest past
 *  update built from a disjoint set of observations (none in common) had a
 *  point within `radius`. Points are compared in the reference frame (e.g.
 *  odom), so the robot motion is compensated. Thus one-off artifacts of a
 *  single scan are never reported, real obstacles are reported at most one
 *  sensor period later, and they disappear as soon as they are no longer
 *  observed (there is no memory).
 *
 *  Observations shared by consecutive updates (e.g. the latest scan of a
 *  second sensor) can't confirm their own artifacts, hence the disjoint sets.
 */
class ObstacleConfirmation
{
   public:
	struct Parameters
	{
		/// Max distance to a past candidate to confirm a point [m]. It must
		/// cover how far obstacles move in one sensor period.
		float radius = 0.20f;
		/// Past updates older than this are not used [s]
		double max_age = 0.5;
	};

	Parameters params;

	/// Observations an update was built from: (source, timestamp) pairs.
	using ObservationIds = std::set<std::pair<std::string, double>>;

	/** Takes the candidate points (robot frame) built at time `t` [s] from the
	 *  observations `ids`, with the robot at `robotPose` in the reference frame,
	 *  and appends to `confirmed` (robot frame) those confirmed by past updates.
	 */
	void update(
		const mrpt::maps::CPointsMap& candidates, const mrpt::poses::CPose3D& robotPose, double t,
		const ObservationIds& ids, mrpt::maps::CPointsMap& confirmed);

	void clear() { history_.clear(); }

	/// Number of stored past updates
	[[nodiscard]] std::size_t size() const { return history_.size(); }

   private:
	struct PastUpdate
	{
		double t = 0;
		ObservationIds ids;
		/// Candidates, in the reference frame
		mrpt::maps::CSimplePointsMap points;
	};
	std::deque<PastUpdate> history_;
};

}  // namespace mrpt_pointcloud_pipeline
