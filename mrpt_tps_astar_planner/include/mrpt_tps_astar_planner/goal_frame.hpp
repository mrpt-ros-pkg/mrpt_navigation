/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2026, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/TPose2D.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <optional>
#include <string>
#include <tf2/buffer_core.hpp>

namespace mrpt_tps_astar_planner
{
/** Returns the 2D pose of a navigation goal in the planner map frame
 *  `frame_id_map`.
 *
 *  A goal with an empty frame_id is taken as already given in the map frame.
 *  Otherwise it is transformed with `tf`, at the goal stamp if available, or
 *  with the latest transform (e.g. a goal stamped slightly after the latest
 *  transform, or with a zero stamp).
 *
 *  Returns std::nullopt, and the reason in `error` if not null, if the goal
 *  frame cannot be transformed into the map frame.
 */
std::optional<mrpt::math::TPose2D> goal_in_map_frame(
	const geometry_msgs::msg::PoseStamped& goal, const std::string& frame_id_map,
	const tf2::BufferCore& tf, std::string* error = nullptr);

}  // namespace mrpt_tps_astar_planner
