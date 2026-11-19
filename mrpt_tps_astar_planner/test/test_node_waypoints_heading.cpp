/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2026, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

// Integration test: the last waypoint of a NavigateWaypoints sequence with
// ignore_heading is planned as a position, so the path may reach it with any
// heading, instead of a heading guessed when the goal was received.

#include <gtest/gtest.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/math/wrap2pi.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <chrono>
#include <cmath>
#include <mrpt_nav_interfaces/action/navigate_waypoints.hpp>
#include <mrpt_nav_interfaces/msg/reference_path.hpp>
#include <mrpt_tps_astar_planner/mrpt_tps_astar_planner_node.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>

using namespace std::chrono_literals;
using mrpt::DEG2RAD;
using NavigateWaypoints = mrpt_nav_interfaces::action::NavigateWaypoints;

namespace
{
// Robot pose in the map frame (static TF map -> base_link):
constexpr double ROBOT_X = 5.0;
constexpr double ROBOT_Y = 2.0;
constexpr double ROBOT_YAW_DEG = 90.0;

double quat_yaw(const geometry_msgs::msg::Quaternion& m)
{
	return std::atan2(2.0 * (m.w * m.z + m.x * m.y), 1.0 - 2.0 * (m.y * m.y + m.z * m.z));
}

class PlannerWaypointsHeadingTest : public ::testing::Test
{
   protected:
	static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
	static void TearDownTestSuite() { rclcpp::shutdown(); }

	void SetUp() override
	{
#if !HAVE_MPP_CONTINUOUS_PLANNING
		GTEST_SKIP() << "navigate_waypoints needs an mrpt_path_planning with resumable A*";
#endif
		const std::string cfg = MRPT_TPS_ASTAR_PLANNER_CONFIGS_DIR;
		rclcpp::NodeOptions opts;
		opts.parameter_overrides({
			{"ptg_ini", cfg + "/ini/ptgs_jackal.ini"},
			{"planner_parameters", cfg + "/params/planner-params.yaml"},
			{"global_costmap_parameters", cfg + "/params/costmap-obstacles.yaml"},
			{"prefer_waypoints_parameters", cfg + "/params/costmap-prefer-waypoints.yaml"},
			{"frame_id_map", std::string("map")},
			{"frame_id_robot", std::string("base_link")},
			{"topic_obstacles_sub", std::string("")},
			{"topic_obstacles_gridmap_sub", std::string("")},
			{"topic_wp_seq_pub", std::string("/test_wps")},
			{"continuous_planning", true},
			{"max_computation_time", 10.0},
		});
		planner_ = std::make_shared<mrpt_tps_astar_planner::TPS_Astar_Planner_Node>(opts);
		client_node_ = std::make_shared<rclcpp::Node>("test_waypoints_heading_client");

		tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(client_node_);
		geometry_msgs::msg::TransformStamped t;
		t.header.stamp = client_node_->now();
		t.header.frame_id = "map";
		t.child_frame_id = "base_link";
		t.transform.translation.x = ROBOT_X;
		t.transform.translation.y = ROBOT_Y;
		t.transform.rotation.z = std::sin(DEG2RAD(ROBOT_YAW_DEG) / 2);
		t.transform.rotation.w = std::cos(DEG2RAD(ROBOT_YAW_DEG) / 2);
		tf_broadcaster_->sendTransform(t);

		sub_ = client_node_->create_subscription<mrpt_nav_interfaces::msg::ReferencePath>(
			"/test_wps_reference", rclcpp::QoS(10).reliable(),
			[this](const mrpt_nav_interfaces::msg::ReferencePath& msg)
			{
				std::lock_guard<std::mutex> lck(mtx_);
				if (!msg.path.poses.empty())
				{
					last_ = msg;
				}
			});

		executor_.add_node(planner_);
		executor_.add_node(client_node_);
		spin_thread_ = std::thread([this]() { executor_.spin(); });

		client_ = rclcpp_action::create_client<NavigateWaypoints>(
			client_node_,
			std::string(planner_->get_fully_qualified_name()) + "/navigate_waypoints");
		ASSERT_TRUE(client_->wait_for_action_server(120s));
	}

	void TearDown() override
	{
		executor_.cancel();
		spin_thread_.join();
	}

	static mrpt_msgs::msg::Waypoint waypoint(double x, double y)
	{
		mrpt_msgs::msg::Waypoint wp;
		wp.target.position.x = x;
		wp.target.position.y = y;
		wp.target.orientation.w = 1.0;
		wp.ignore_heading = true;
		wp.allow_skip = true;
		return wp;
	}

	// Sends the sequence (retrying while the PTG tables are still being built)
	// and returns the first path to the last waypoint.
	std::optional<mrpt_nav_interfaces::msg::ReferencePath> path_to(
		const std::vector<mrpt_msgs::msg::Waypoint>& wps, double gx, double gy)
	{
		const auto deadline = std::chrono::steady_clock::now() + 180s;
		while (std::chrono::steady_clock::now() < deadline)
		{
			NavigateWaypoints::Goal goal;
			goal.waypoints.header.frame_id = "map";
			goal.waypoints.waypoints = wps;
			auto fut = client_->async_send_goal(goal);
			if (fut.wait_for(10s) != std::future_status::ready || !fut.get())
			{
				std::this_thread::sleep_for(1s);
				continue;
			}
			const auto until = std::chrono::steady_clock::now() + 20s;
			while (std::chrono::steady_clock::now() < until)
			{
				{
					std::lock_guard<std::mutex> lck(mtx_);
					if (last_)
					{
						const auto& p = last_->path.poses.back().pose.position;
						if (std::hypot(p.x - gx, p.y - gy) < 1e-3)
						{
							return last_;
						}
					}
				}
				std::this_thread::sleep_for(100ms);
			}
		}
		return std::nullopt;
	}

	std::shared_ptr<mrpt_tps_astar_planner::TPS_Astar_Planner_Node> planner_;
	rclcpp::Node::SharedPtr client_node_;
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
	rclcpp::executors::MultiThreadedExecutor executor_;
	std::thread spin_thread_;
	rclcpp_action::Client<NavigateWaypoints>::SharedPtr client_;
	rclcpp::Subscription<mrpt_nav_interfaces::msg::ReferencePath>::SharedPtr sub_;
	std::mutex mtx_;
	std::optional<mrpt_nav_interfaces::msg::ReferencePath> last_;
};

}  // namespace

TEST_F(PlannerWaypointsHeadingTest, LastWaypointHasFreeHeading)
{
	// The first waypoint is at the robot (reached at once); the last one, 1.5 m
	// behind it. A heading from the first to the last waypoint (facing away from
	// the robot) would need a turn in place; with a free heading, the shortest
	// way is backing up, keeping the robot heading.
	const auto rp = path_to(
		{waypoint(ROBOT_X, ROBOT_Y), waypoint(ROBOT_X, ROBOT_Y - 1.5)}, ROBOT_X, ROBOT_Y - 1.5);
	ASSERT_TRUE(rp.has_value());
	const double endYaw = quat_yaw(rp->path.poses.back().pose.orientation);
	EXPECT_LT(std::abs(mrpt::math::wrapToPi(endYaw - DEG2RAD(ROBOT_YAW_DEG))), DEG2RAD(30.0))
		<< "final heading: " << mrpt::RAD2DEG(endYaw) << " deg";
}
