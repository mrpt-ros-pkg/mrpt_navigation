/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2026, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/maps/CPointsMap.h>
#include <mrpt/poses/CPose3D.h>

#include <cstddef>
#include <cstdint>
#include <unordered_map>

namespace mrpt_pointcloud_pipeline
{
/** Short-term memory of obstacle points inside a region around the robot
 *  that its sensors can't see (e.g. right in front of a high-mounted lidar).
 *
 *  Live points inside or near the region are stored (voxelized) in the
 *  reference frame (e.g. odom). Each update returns the stored points that
 *  are now inside the region and were not seen in that update, so obstacles
 *  that move into the blind zone are still reported. Stored points outside
 *  the region are forgotten as soon as they are no longer observed (the
 *  sensor sees there), and all of them after `max_age`.
 */
class ObstacleMemory
{
   public:
	struct Parameters
	{
		/// Region (robot frame, XY) where obstacles are remembered [m]
		float region_min_x = 0;
		float region_min_y = 0;
		float region_max_x = 0;
		float region_max_y = 0;

		/// Live points this close to the region are stored too, so they
		/// are known if they enter it later [m]
		float store_margin = 1.0f;

		/// Storage resolution [m]
		float voxel_size = 0.10f;

		/// Remembered points are forgotten after this long without being
		/// observed [s]
		double max_age = 10.0;

		/// Points outside the region are forgotten after this long without
		/// being observed [s]
		double unseen_timeout = 1.0;
	};

	Parameters params;

	/** Stores the live points (robot frame) observed at time `t` [s], with
	 *  the robot at `robotPose` in the reference frame, and appends to `out`
	 *  (robot frame) the remembered points now inside the region that were
	 *  not part of `live`.
	 */
	void update(
		const mrpt::maps::CPointsMap& live, const mrpt::poses::CPose3D& robotPose, double t,
		mrpt::maps::CPointsMap& out);

	void clear() { voxels_.clear(); }

	/// Number of stored voxels
	[[nodiscard]] std::size_t size() const { return voxels_.size(); }

   private:
	struct Cell
	{
		float x = 0;
		float y = 0;
		float z = 0;
		double last_seen = 0;
	};

	std::unordered_map<uint64_t, Cell> voxels_;

	[[nodiscard]] bool in_region(float x, float y, float margin) const;
	[[nodiscard]] uint64_t voxel_key(float x, float y, float z) const;
};

}  // namespace mrpt_pointcloud_pipeline
