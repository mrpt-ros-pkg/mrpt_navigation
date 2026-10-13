/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2026, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/maps/CPointsMap.h>
#include <mrpt/math/TPolygon2D.h>

#include <geometry_msgs/msg/polygon.hpp>
#include <vector>

namespace mrpt_tps_astar_planner
{
/// XY polygons from ROS messages (z is ignored)
std::vector<mrpt::math::TPolygon2D> to_polygons(
	const std::vector<geometry_msgs::msg::Polygon>& polygons);

/** Returns `pts` without the points whose (x,y) lies inside any of `regions`,
 *  or `pts` itself if none does. */
mrpt::maps::CPointsMap::Ptr without_points_in(
	const mrpt::maps::CPointsMap::Ptr& pts, const std::vector<mrpt::math::TPolygon2D>& regions);

}  // namespace mrpt_tps_astar_planner
