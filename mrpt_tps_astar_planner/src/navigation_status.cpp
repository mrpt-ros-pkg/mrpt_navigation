/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2026, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include <cmath>
#include <cstdio>
#include <limits>
#include <mrpt_tps_astar_planner/navigation_status.hpp>
#include <rclcpp/duration.hpp>

namespace mrpt_tps_astar_planner::nav_status
{
namespace
{
using visualization_msgs::msg::Marker;

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

struct Color
{
	float r;
	float g;
	float b;
};

constexpr Color kGreen{0.2f, 0.8f, 0.2f};
constexpr Color kYellow{1.0f, 0.8f, 0.0f};
constexpr Color kRed{0.9f, 0.1f, 0.1f};
constexpr Color kGray{0.6f, 0.6f, 0.6f};
constexpr Color kBlue{0.2f, 0.4f, 1.0f};

Color color_for_level(const NavigationStatus& s)
{
	if (s.state == NavigationStatus::STATE_READY)
	{
		return kGray;
	}
	switch (s.level)
	{
		case NavigationStatus::LEVEL_ERROR:
			return kRed;
		case NavigationStatus::LEVEL_WARN:
			return kYellow;
		default:
			return kGreen;
	}
}

Color color_for_goal(uint8_t state)
{
	switch (state)
	{
		case NavigationStatus::STATE_SUCCEEDED:
			return kGreen;
		case NavigationStatus::STATE_FAILED:
			return kRed;
		case NavigationStatus::STATE_CANCELED:
			return kGray;
		default:
			return kBlue;
	}
}

Marker base_marker(
	const std::string& ns, int id, const std::string& frame, const rclcpp::Duration& lifetime)
{
	Marker m;
	m.header.frame_id = frame;
	// (stamp 0: the latest transform, so it does not lag or fail to extrapolate)
	m.ns = ns;
	m.id = id;
	m.lifetime = lifetime;
	m.pose.orientation.w = 1.0;
	return m;
}

Marker delete_marker(const std::string& ns, int id, const std::string& frame)
{
	Marker m = base_marker(ns, id, frame, rclcpp::Duration(0, 0));
	m.action = Marker::DELETE;
	return m;
}

void set_color(Marker& m, const Color& c, float alpha = 1.0f)
{
	m.color.r = c.r;
	m.color.g = c.g;
	m.color.b = c.b;
	m.color.a = alpha;
}

geometry_msgs::msg::Point make_point(double x, double y, double z)
{
	geometry_msgs::msg::Point p;
	p.x = x;
	p.y = y;
	p.z = z;
	return p;
}

// Yaw of a planar orientation (rotation around Z only)
double planar_yaw(const geometry_msgs::msg::Quaternion& q) { return 2.0 * std::atan2(q.z, q.w); }

bool has_goal(const NavigationStatus& s)
{
	return s.goal_id != 0 && (is_active_goal(s.state) || is_terminal(s.state) ||
							  s.state == NavigationStatus::STATE_INITIALIZING);
}

std::string format_distance(float meters)
{
	char buf[32];
	std::snprintf(buf, sizeof(buf), "%.1f m", static_cast<double>(meters));
	return buf;
}
}  // namespace

const char* state_name(uint8_t state)
{
	switch (state)
	{
		case NavigationStatus::STATE_INITIALIZING:
			return "INITIALIZING";
		case NavigationStatus::STATE_INIT_FAILED:
			return "INIT_FAILED";
		case NavigationStatus::STATE_READY:
			return "READY";
		case NavigationStatus::STATE_PLANNING:
			return "PLANNING";
		case NavigationStatus::STATE_FOLLOWING_PARTIAL:
			return "FOLLOWING_PARTIAL";
		case NavigationStatus::STATE_FOLLOWING:
			return "FOLLOWING";
		case NavigationStatus::STATE_REPLANNING:
			return "REPLANNING";
		case NavigationStatus::STATE_BLOCKED:
			return "BLOCKED";
		case NavigationStatus::STATE_STALLED:
			return "STALLED";
		case NavigationStatus::STATE_SUCCEEDED:
			return "SUCCEEDED";
		case NavigationStatus::STATE_FAILED:
			return "FAILED";
		case NavigationStatus::STATE_CANCELED:
			return "CANCELED";
		default:
			return "UNKNOWN";
	}
}

bool is_terminal(uint8_t state)
{
	return state == NavigationStatus::STATE_SUCCEEDED || state == NavigationStatus::STATE_FAILED ||
		   state == NavigationStatus::STATE_CANCELED;
}

bool is_active_goal(uint8_t state)
{
	return state >= NavigationStatus::STATE_PLANNING && state <= NavigationStatus::STATE_STALLED;
}

NavigationStatus make_status(uint8_t state)
{
	NavigationStatus s;
	s.state = state;
	s.state_name = state_name(state);
	s.level = state == NavigationStatus::STATE_INIT_FAILED ? NavigationStatus::LEVEL_ERROR
														   : NavigationStatus::LEVEL_OK;
	s.distance_to_goal = kNaN;
	s.remaining_path_length = kNaN;
	s.eta = kNaN;
	return s;
}

void set_goal(
	NavigationStatus& s, uint32_t id, bool fromAction, const mrpt::math::TPose2D& pose,
	const std::string& frame)
{
	s.goal_id = id;
	s.goal_from_action = fromAction;
	s.goal.header.frame_id = frame;
	s.goal.pose.position.x = pose.x;
	s.goal.pose.position.y = pose.y;
	s.goal.pose.position.z = 0;
	s.goal.pose.orientation.x = 0;
	s.goal.pose.orientation.y = 0;
	s.goal.pose.orientation.z = std::sin(0.5 * pose.phi);
	s.goal.pose.orientation.w = std::cos(0.5 * pose.phi);
	s.elapsed = 0;
	s.distance_to_goal = kNaN;
	s.remaining_path_length = kNaN;
	s.path_is_partial = false;
	s.eta = kNaN;
	s.time_since_progress = 0;
	s.replans = 0;
	s.follower_status.clear();
}

std::string hud_text(const NavigationStatus& s)
{
	std::string line1;
	std::string line2 = s.message;
	switch (s.state)
	{
		case NavigationStatus::STATE_INITIALIZING:
		{
			line1 = "Planner initializing";
			if (!s.init_detail.empty())
			{
				char buf[16];
				std::snprintf(buf, sizeof(buf), " (%.0f%%)", 100.0 * s.init_progress);
				line1 += ": " + s.init_detail + buf;
			}
			else
			{
				line1 += "...";
			}
			if (s.goal_id != 0)
			{
				line2 = "Goal #" + std::to_string(s.goal_id) + " queued";
			}
			else
			{
				line2.clear();
			}
			break;
		}
		case NavigationStatus::STATE_INIT_FAILED:
			line1 = "Planner initialization FAILED";
			break;
		case NavigationStatus::STATE_READY:
			line1 = "Ready";
			line2.clear();
			break;
		default:
		{
			line1 = "#" + std::to_string(s.goal_id) + " " + s.state_name;
			if (is_active_goal(s.state) && std::isfinite(s.remaining_path_length))
			{
				line1 += "  " + format_distance(s.remaining_path_length);
				if (std::isfinite(s.eta))
				{
					char buf[24];
					std::snprintf(buf, sizeof(buf), "  ETA %.0f s", static_cast<double>(s.eta));
					line1 += buf;
				}
			}
			break;
		}
	}
	return line2.empty() ? line1 : line1 + "\n" + line2;
}

visualization_msgs::msg::MarkerArray make_markers(
	const NavigationStatus& s, const HudExtras& extras,
	const std::vector<mrpt::math::TPoint2D>& footprint, const std::string& frameRobot,
	const std::string& frameMap, double lifetime)
{
	visualization_msgs::msg::MarkerArray arr;
	const rclcpp::Duration life = rclcpp::Duration::from_seconds(lifetime);

	// Status text above the robot:
	{
		Marker m = base_marker("status_text", 0, frameRobot, life);
		m.type = Marker::TEXT_VIEW_FACING;
		m.action = Marker::ADD;
		m.pose.position.z = 1.5;
		m.scale.z = 0.3;
		set_color(m, color_for_level(s));
		m.text = hud_text(s);
		arr.markers.push_back(m);
	}

	// Goal pose and footprint (the goal of a finished navigation is shown
	// until the status goes back to READY):
	const bool showGoal = has_goal(s);
	if (showGoal)
	{
		const Color c = color_for_goal(s.state);
		Marker arrow = base_marker("goal", 0, frameMap, life);
		arrow.type = Marker::ARROW;
		arrow.action = Marker::ADD;
		arrow.pose = s.goal.pose;
		arrow.scale.x = 0.8;
		arrow.scale.y = 0.12;
		arrow.scale.z = 0.12;
		set_color(arrow, c);
		arr.markers.push_back(arrow);

		if (!footprint.empty())
		{
			Marker fp = base_marker("goal", 1, frameMap, life);
			fp.type = Marker::LINE_STRIP;
			fp.action = Marker::ADD;
			fp.scale.x = 0.05;
			set_color(fp, c);
			const double yaw = planar_yaw(s.goal.pose.orientation);
			const double cy = std::cos(yaw);
			const double sy = std::sin(yaw);
			for (size_t i = 0; i <= footprint.size(); i++)
			{
				const auto& p = footprint[i % footprint.size()];
				fp.points.push_back(make_point(
					s.goal.pose.position.x + cy * p.x - sy * p.y,
					s.goal.pose.position.y + sy * p.x + cy * p.y, 0.05));
			}
			arr.markers.push_back(fp);
		}
	}
	else
	{
		arr.markers.push_back(delete_marker("goal", 0, frameMap));
		arr.markers.push_back(delete_marker("goal", 1, frameMap));
	}

	// What blocks the way (only while handling a goal):
	const bool active = is_active_goal(s.state);
	if (active && !extras.blockingPoints.empty())
	{
		Marker m = base_marker("blocking", 0, frameMap, life);
		m.type = Marker::SPHERE_LIST;
		m.action = Marker::ADD;
		m.scale.x = m.scale.y = m.scale.z = 0.08;
		set_color(m, kRed);
		for (const auto& p : extras.blockingPoints)
		{
			m.points.push_back(make_point(p.x, p.y, 0.1));
		}
		arr.markers.push_back(m);
	}
	else
	{
		arr.markers.push_back(delete_marker("blocking", 0, frameMap));
	}
	if (active && extras.blockedPose)
	{
		Marker m = base_marker("blocking", 1, frameMap, life);
		m.type = Marker::SPHERE;
		m.action = Marker::ADD;
		m.pose.position = make_point(extras.blockedPose->x, extras.blockedPose->y, 0.2);
		m.scale.x = m.scale.y = m.scale.z = 0.3;
		set_color(m, kRed, 0.8f);
		arr.markers.push_back(m);
	}
	else
	{
		arr.markers.push_back(delete_marker("blocking", 1, frameMap));
	}

	// Where a partial path ends:
	if (active && extras.partialEnd)
	{
		Marker m = base_marker("partial_end", 0, frameMap, life);
		m.type = Marker::CYLINDER;
		m.action = Marker::ADD;
		m.pose.position = make_point(extras.partialEnd->x, extras.partialEnd->y, 0.15);
		m.scale.x = m.scale.y = 0.3;
		m.scale.z = 0.3;
		set_color(m, kYellow, 0.8f);
		arr.markers.push_back(m);
	}
	else
	{
		arr.markers.push_back(delete_marker("partial_end", 0, frameMap));
	}
	return arr;
}

}  // namespace mrpt_tps_astar_planner::nav_status
