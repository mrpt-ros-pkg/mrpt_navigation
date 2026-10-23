/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2026, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPose2D.h>

#include <mrpt_nav_interfaces/msg/navigation_status.hpp>
#include <optional>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

namespace mrpt_tps_astar_planner
{
/// Helpers to build the navigation status and its visualization (no ROS
/// communication, so they can be tested alone).
namespace nav_status
{
using NavigationStatus = mrpt_nav_interfaces::msg::NavigationStatus;

/// "INITIALIZING", "FOLLOWING", ... ("UNKNOWN" for invalid values)
const char* state_name(uint8_t state);

/// SUCCEEDED, FAILED or CANCELED
bool is_terminal(uint8_t state);

/// A goal is being handled: PLANNING to STALLED
bool is_active_goal(uint8_t state);

/// A status in `state` with no goal and unknown (NaN) measurements
NavigationStatus make_status(uint8_t state);

/// Sets the goal fields (id, source and pose in `frame`) and clears its measurements
void set_goal(
	NavigationStatus& s, uint32_t id, bool fromAction, const mrpt::math::TPose2D& pose,
	const std::string& frame);

/// Extra details shown in the 3D views, as set by the goal manager
struct HudExtras
{
	/// Obstacle points remembered for having blocked a path of the goal
	std::vector<mrpt::math::TPoint2D> blockingPoints;
	/// Where the latest path got blocked
	std::optional<mrpt::math::TPose2D> blockedPose;
	/// End of a partial path, where the robot waits for a longer one
	std::optional<mrpt::math::TPose2D> partialEnd;
};

/// One or two lines of text summarizing the status
std::string hud_text(const NavigationStatus& s);

/**
 * Markers to display the status in 3D views (namespaces `status_text`,
 * `goal`, `blocking` and `partial_end`). Markers of things not to show now
 * are DELETE ones, so every call describes the whole HUD. Markers expire
 * after `lifetime` seconds, so they vanish if no longer refreshed.
 *
 * \param footprint Robot footprint in the robot frame (empty: not shown at the goal)
 * \param frameRobot Frame of the status text, placed above it
 * \param frameMap Frame of the goal and blocking markers
 */
visualization_msgs::msg::MarkerArray make_markers(
	const NavigationStatus& s, const HudExtras& extras,
	const std::vector<mrpt::math::TPoint2D>& footprint, const std::string& frameRobot,
	const std::string& frameMap, double lifetime = 3.0);

}  // namespace nav_status
}  // namespace mrpt_tps_astar_planner
