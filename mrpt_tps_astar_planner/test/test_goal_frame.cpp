/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2026, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/core/bits_math.h>

#include <mrpt_tps_astar_planner/goal_frame.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

using mrpt::DEG2RAD;
using mrpt_tps_astar_planner::goal_in_map_frame;

namespace
{
geometry_msgs::msg::Quaternion yaw_quat(double yaw)
{
	tf2::Quaternion q;
	q.setRPY(0, 0, yaw);
	geometry_msgs::msg::Quaternion m;
	m.x = q.x();
	m.y = q.y();
	m.z = q.z();
	m.w = q.w();
	return m;
}

geometry_msgs::msg::TransformStamped transform(
	const std::string& parent, const std::string& child, double x, double y, double yaw,
	int32_t stamp_sec = 0)
{
	geometry_msgs::msg::TransformStamped t;
	t.header.frame_id = parent;
	t.header.stamp.sec = stamp_sec;
	t.child_frame_id = child;
	t.transform.translation.x = x;
	t.transform.translation.y = y;
	t.transform.rotation = yaw_quat(yaw);
	return t;
}

geometry_msgs::msg::PoseStamped goal(
	const std::string& frame, double x, double y, double yaw, int32_t stamp_sec = 0)
{
	geometry_msgs::msg::PoseStamped g;
	g.header.frame_id = frame;
	g.header.stamp.sec = stamp_sec;
	g.pose.position.x = x;
	g.pose.position.y = y;
	g.pose.orientation = yaw_quat(yaw);
	return g;
}

constexpr double TOL = 1e-6;

void expect_pose(const std::optional<mrpt::math::TPose2D>& p, double x, double y, double yaw)
{
	ASSERT_TRUE(p.has_value());
	EXPECT_NEAR(p->x, x, TOL);
	EXPECT_NEAR(p->y, y, TOL);
	EXPECT_NEAR(mrpt::math::wrapToPi(p->phi - yaw), 0.0, TOL);
}

}  // namespace

// Legacy behavior: goals without a frame are taken as map coordinates.
TEST(GoalFrame, EmptyFrameIsMap)
{
	tf2::BufferCore tf;
	expect_pose(goal_in_map_frame(goal("", 1.0, 2.0, 0.5), "map", tf), 1.0, 2.0, 0.5);
}

TEST(GoalFrame, MapFrameNeedsNoTf)
{
	tf2::BufferCore tf;	 // empty: no transform needed
	expect_pose(goal_in_map_frame(goal("map", -3.0, 4.0, -1.0), "map", tf), -3.0, 4.0, -1.0);
}

TEST(GoalFrame, RobotFrameChain)
{
	tf2::BufferCore tf;
	tf.setTransform(transform("map", "odom", 10.0, 0.0, DEG2RAD(90.0)), "test", true);
	tf.setTransform(transform("odom", "base_link", 1.0, 0.0, 0.0), "test", true);

	// base_link is at map (10, 1) heading +y. A goal 2 m ahead of the robot,
	// heading to its left:
	const auto p = goal_in_map_frame(goal("base_link", 2.0, 0.0, DEG2RAD(90.0)), "map", tf);
	expect_pose(p, 10.0, 3.0, DEG2RAD(180.0));
}

TEST(GoalFrame, UsesTransformAtGoalStamp)
{
	tf2::BufferCore tf;
	tf.setTransform(transform("map", "base_link", 0.0, 0.0, 0.0, 10), "test", false);
	tf.setTransform(transform("map", "base_link", 4.0, 0.0, 0.0, 20), "test", false);

	// At t=10 the robot was at the origin; at t=15, halfway (interpolated):
	expect_pose(goal_in_map_frame(goal("base_link", 1.0, 0.0, 0.0, 10), "map", tf), 1.0, 0.0, 0.0);
	expect_pose(goal_in_map_frame(goal("base_link", 1.0, 0.0, 0.0, 15), "map", tf), 3.0, 0.0, 0.0);
}

TEST(GoalFrame, FallsBackToLatestTransform)
{
	tf2::BufferCore tf;
	tf.setTransform(transform("map", "base_link", 0.0, 0.0, 0.0, 10), "test", false);
	tf.setTransform(transform("map", "base_link", 4.0, 0.0, 0.0, 20), "test", false);

	// Goal newer than the latest transform, or with a zero stamp: latest one.
	expect_pose(goal_in_map_frame(goal("base_link", 1.0, 0.0, 0.0, 25), "map", tf), 5.0, 0.0, 0.0);
	expect_pose(goal_in_map_frame(goal("base_link", 1.0, 0.0, 0.0, 0), "map", tf), 5.0, 0.0, 0.0);
}

TEST(GoalFrame, UnknownFrameFails)
{
	tf2::BufferCore tf;
	tf.setTransform(transform("map", "odom", 0.0, 0.0, 0.0), "test", true);

	std::string error;
	const auto p = goal_in_map_frame(goal("camera", 1.0, 0.0, 0.0), "map", tf, &error);
	EXPECT_FALSE(p.has_value());
	EXPECT_FALSE(error.empty());
}

TEST(GoalFrame, NonDefaultMapFrame)
{
	tf2::BufferCore tf;
	tf.setTransform(transform("world", "base_link", 0.0, 5.0, 0.0), "test", true);

	expect_pose(goal_in_map_frame(goal("base_link", 1.0, 0.0, 0.0), "world", tf), 1.0, 5.0, 0.0);
	// "map" is not special: here it is just another frame, unknown to TF.
	EXPECT_FALSE(goal_in_map_frame(goal("map", 1.0, 0.0, 0.0), "world", tf).has_value());
}
