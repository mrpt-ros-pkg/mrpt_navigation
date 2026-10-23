/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2026, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>

#include <cmath>
#include <mrpt_tps_astar_planner/navigation_status.hpp>
#include <set>

using mrpt_tps_astar_planner::nav_status::HudExtras;
using mrpt_tps_astar_planner::nav_status::NavigationStatus;
namespace ns = mrpt_tps_astar_planner::nav_status;
using visualization_msgs::msg::Marker;

namespace
{
const std::vector<mrpt::math::TPoint2D> kFootprint = {
	{-0.2, -0.2}, {0.3, -0.2}, {0.3, 0.2}, {-0.2, 0.2}};

NavigationStatus goal_status(uint8_t state)
{
	auto s = ns::make_status(state);
	ns::set_goal(s, 3, false, mrpt::math::TPose2D(1.0, 2.0, 0.5), "map");
	return s;
}

// Markers of a namespace and action:
std::vector<Marker> find(
	const visualization_msgs::msg::MarkerArray& arr, const std::string& name, uint8_t action)
{
	std::vector<Marker> r;
	for (const auto& m : arr.markers)
	{
		if (m.ns == name && m.action == action)
		{
			r.push_back(m);
		}
	}
	return r;
}
}  // namespace

TEST(NavigationStatus, StateNamesAreUniqueAndKnown)
{
	std::set<std::string> names;
	for (uint8_t st = NavigationStatus::STATE_INITIALIZING; st <= NavigationStatus::STATE_CANCELED;
		 st++)
	{
		const std::string n = ns::state_name(st);
		EXPECT_NE(n, "UNKNOWN") << "state " << int(st);
		names.insert(n);
	}
	EXPECT_EQ(names.size(), 12u);
	EXPECT_EQ(std::string(ns::state_name(NavigationStatus::STATE_FOLLOWING)), "FOLLOWING");
	EXPECT_EQ(std::string(ns::state_name(200)), "UNKNOWN");
}

TEST(NavigationStatus, StateClasses)
{
	EXPECT_TRUE(ns::is_terminal(NavigationStatus::STATE_SUCCEEDED));
	EXPECT_TRUE(ns::is_terminal(NavigationStatus::STATE_FAILED));
	EXPECT_TRUE(ns::is_terminal(NavigationStatus::STATE_CANCELED));
	EXPECT_FALSE(ns::is_terminal(NavigationStatus::STATE_FOLLOWING));
	EXPECT_TRUE(ns::is_active_goal(NavigationStatus::STATE_PLANNING));
	EXPECT_TRUE(ns::is_active_goal(NavigationStatus::STATE_STALLED));
	EXPECT_FALSE(ns::is_active_goal(NavigationStatus::STATE_READY));
	EXPECT_FALSE(ns::is_active_goal(NavigationStatus::STATE_SUCCEEDED));
}

TEST(NavigationStatus, MakeStatusHasUnknownMeasurements)
{
	const auto s = ns::make_status(NavigationStatus::STATE_READY);
	EXPECT_EQ(s.state_name, "READY");
	EXPECT_EQ(s.level, NavigationStatus::LEVEL_OK);
	EXPECT_EQ(s.goal_id, 0u);
	EXPECT_TRUE(std::isnan(s.distance_to_goal));
	EXPECT_TRUE(std::isnan(s.remaining_path_length));
	EXPECT_TRUE(std::isnan(s.eta));
	EXPECT_EQ(
		ns::make_status(NavigationStatus::STATE_INIT_FAILED).level, NavigationStatus::LEVEL_ERROR);
}

TEST(NavigationStatus, SetGoalEncodesYaw)
{
	const auto s = goal_status(NavigationStatus::STATE_PLANNING);
	EXPECT_EQ(s.goal_id, 3u);
	EXPECT_EQ(s.goal.header.frame_id, "map");
	EXPECT_DOUBLE_EQ(s.goal.pose.position.x, 1.0);
	EXPECT_DOUBLE_EQ(s.goal.pose.position.y, 2.0);
	EXPECT_NEAR(2.0 * std::atan2(s.goal.pose.orientation.z, s.goal.pose.orientation.w), 0.5, 1e-12);
}

TEST(NavigationStatus, HudText)
{
	auto s = ns::make_status(NavigationStatus::STATE_INITIALIZING);
	EXPECT_EQ(ns::hud_text(s), "Planner initializing...");
	s.init_detail = "PTG 2/4: DiffDrive_C (building grid)";
	s.init_progress = 0.5f;
	EXPECT_EQ(ns::hud_text(s), "Planner initializing: PTG 2/4: DiffDrive_C (building grid) (50%)");
	s.goal_id = 7;
	EXPECT_NE(ns::hud_text(s).find("Goal #7 queued"), std::string::npos);

	EXPECT_EQ(ns::hud_text(ns::make_status(NavigationStatus::STATE_READY)), "Ready");

	auto f = goal_status(NavigationStatus::STATE_FOLLOWING);
	f.state_name = ns::state_name(f.state);
	f.remaining_path_length = 12.44f;
	f.eta = 25.2f;
	EXPECT_EQ(ns::hud_text(f), "#3 FOLLOWING  12.4 m  ETA 25 s");
	f.eta = std::numeric_limits<float>::quiet_NaN();
	EXPECT_EQ(ns::hud_text(f), "#3 FOLLOWING  12.4 m");

	auto st = goal_status(NavigationStatus::STATE_STALLED);
	st.state_name = ns::state_name(st.state);
	st.message = "No progress for 32 s (gives up at 60 s)";
	EXPECT_EQ(ns::hud_text(st), "#3 STALLED\nNo progress for 32 s (gives up at 60 s)");

	auto done = goal_status(NavigationStatus::STATE_SUCCEEDED);
	done.state_name = ns::state_name(done.state);
	done.remaining_path_length = 5.0f;	// (not shown once finished)
	EXPECT_EQ(ns::hud_text(done), "#3 SUCCEEDED");
}

TEST(NavigationStatus, MarkersWithoutGoal)
{
	const auto arr = ns::make_markers(
		ns::make_status(NavigationStatus::STATE_READY), {}, kFootprint, "base_link", "map");

	const auto text = find(arr, "status_text", Marker::ADD);
	ASSERT_EQ(text.size(), 1u);
	EXPECT_EQ(text[0].type, Marker::TEXT_VIEW_FACING);
	EXPECT_EQ(text[0].header.frame_id, "base_link");
	EXPECT_EQ(text[0].header.stamp.sec, 0);
	EXPECT_EQ(text[0].color.a, 1.0f);
	EXPECT_GT(text[0].scale.z, 0.0);
	EXPECT_EQ(text[0].lifetime.sec, 3);

	EXPECT_TRUE(find(arr, "goal", Marker::ADD).empty());
	EXPECT_FALSE(find(arr, "goal", Marker::DELETE).empty());
	EXPECT_TRUE(find(arr, "blocking", Marker::ADD).empty());
	EXPECT_TRUE(find(arr, "partial_end", Marker::ADD).empty());
}

TEST(NavigationStatus, GoalMarkersAndColors)
{
	const auto color_of = [](uint8_t state)
	{
		const auto arr = ns::make_markers(goal_status(state), {}, kFootprint, "base_link", "map");
		const auto goal = find(arr, "goal", Marker::ADD);
		EXPECT_EQ(goal.size(), 2u);	 // arrow and footprint
		return goal.at(0).color;
	};
	const auto active = color_of(NavigationStatus::STATE_FOLLOWING);
	EXPECT_GT(active.b, active.r);
	const auto ok = color_of(NavigationStatus::STATE_SUCCEEDED);
	EXPECT_GT(ok.g, ok.r);
	const auto fail = color_of(NavigationStatus::STATE_FAILED);
	EXPECT_GT(fail.r, fail.g);
	const auto canceled = color_of(NavigationStatus::STATE_CANCELED);
	EXPECT_FLOAT_EQ(canceled.r, canceled.g);

	// The footprint is a closed loop around the goal pose:
	const auto arr = ns::make_markers(
		goal_status(NavigationStatus::STATE_PLANNING), {}, kFootprint, "base_link", "map");
	const auto goal = find(arr, "goal", Marker::ADD);
	const auto fp = goal.at(1);
	ASSERT_EQ(fp.points.size(), kFootprint.size() + 1);
	EXPECT_DOUBLE_EQ(fp.points.front().x, fp.points.back().x);
	EXPECT_DOUBLE_EQ(fp.points.front().y, fp.points.back().y);
	const double x0 = kFootprint[0].x;
	const double y0 = kFootprint[0].y;
	EXPECT_NEAR(fp.points[0].x, 1.0 + std::cos(0.5) * x0 - std::sin(0.5) * y0, 1e-9);
	EXPECT_NEAR(fp.points[0].y, 2.0 + std::sin(0.5) * x0 + std::cos(0.5) * y0, 1e-9);
}

TEST(NavigationStatus, QueuedGoalIsShownWhileInitializing)
{
	auto s = goal_status(NavigationStatus::STATE_INITIALIZING);
	const auto arr = ns::make_markers(s, {}, {}, "base_link", "map");
	EXPECT_EQ(find(arr, "goal", Marker::ADD).size(), 1u);  // (no footprint known yet)
}

TEST(NavigationStatus, BlockingAndPartialEndOnlyWhileNavigating)
{
	HudExtras extras;
	extras.blockingPoints = {{1.0, 1.0}, {1.1, 1.0}};
	extras.blockedPose = mrpt::math::TPose2D(2.0, 2.0, 0.0);
	extras.partialEnd = mrpt::math::TPose2D(3.0, 3.0, 0.0);

	const auto during = ns::make_markers(
		goal_status(NavigationStatus::STATE_BLOCKED), extras, kFootprint, "base_link", "map");
	const auto points = find(during, "blocking", Marker::ADD);
	ASSERT_EQ(points.size(), 2u);
	EXPECT_EQ(points[0].type, Marker::SPHERE_LIST);
	EXPECT_EQ(points[0].points.size(), 2u);
	EXPECT_EQ(find(during, "partial_end", Marker::ADD).size(), 1u);

	const auto after = ns::make_markers(
		goal_status(NavigationStatus::STATE_SUCCEEDED), extras, kFootprint, "base_link", "map");
	EXPECT_TRUE(find(after, "blocking", Marker::ADD).empty());
	EXPECT_TRUE(find(after, "partial_end", Marker::ADD).empty());
	EXPECT_FALSE(find(after, "blocking", Marker::DELETE).empty());
}

TEST(NavigationStatus, TextColorFollowsLevel)
{
	auto ok = goal_status(NavigationStatus::STATE_FOLLOWING);
	auto warn = goal_status(NavigationStatus::STATE_STALLED);
	warn.level = NavigationStatus::LEVEL_WARN;
	auto err = goal_status(NavigationStatus::STATE_FAILED);
	err.level = NavigationStatus::LEVEL_ERROR;
	const auto c = [](const NavigationStatus& s)
	{
		return find(ns::make_markers(s, {}, {}, "base_link", "map"), "status_text", Marker::ADD)[0]
			.color;
	};
	EXPECT_GT(c(ok).g, c(ok).r);
	EXPECT_GT(c(warn).r, 0.9f);
	EXPECT_GT(c(warn).g, 0.5f);
	EXPECT_GT(c(err).r, c(err).g);
	EXPECT_FLOAT_EQ(c(err).a, 1.0f);
}
