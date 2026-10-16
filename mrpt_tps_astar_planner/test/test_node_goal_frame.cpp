/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2026, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

// Integration test: the planner node expresses MakePlanTo targets given in
// other frames (e.g. the robot frame) in its map frame, keeps the legacy
// behavior for targets without a frame, and rejects unknown frames.

#include <gtest/gtest.h>
#include <mrpt/core/bits_math.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <chrono>
#include <mrpt_nav_interfaces/srv/make_plan_to.hpp>
#include <mrpt_tps_astar_planner/mrpt_tps_astar_planner_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <thread>

using namespace std::chrono_literals;
using mrpt::DEG2RAD;
using MakePlanTo = mrpt_nav_interfaces::srv::MakePlanTo;

namespace
{
// Robot pose in the map frame (static TF map -> base_link):
constexpr double ROBOT_X = 5.0;
constexpr double ROBOT_Y = 2.0;
constexpr double ROBOT_YAW_DEG = 90.0;

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

double quat_yaw(const geometry_msgs::msg::Quaternion& m)
{
	double roll = 0;
	double pitch = 0;
	double yaw = 0;
	tf2::Matrix3x3(tf2::Quaternion(m.x, m.y, m.z, m.w)).getRPY(roll, pitch, yaw);
	return yaw;
}

class PlannerGoalFrameTest : public ::testing::Test
{
   protected:
	static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
	static void TearDownTestSuite() { rclcpp::shutdown(); }

	void SetUp() override
	{
		const std::string cfg = MRPT_TPS_ASTAR_PLANNER_CONFIGS_DIR;
		rclcpp::NodeOptions opts;
		opts.parameter_overrides({
			{"ptg_ini", cfg + "/ini/ptgs_jackal.ini"},
			{"planner_parameters", cfg + "/params/planner-params.yaml"},
			{"global_costmap_parameters", cfg + "/params/costmap-obstacles.yaml"},
			{"prefer_waypoints_parameters", cfg + "/params/costmap-prefer-waypoints.yaml"},
			{"frame_id_map", std::string("map")},
			{"frame_id_robot", std::string("base_link")},
			{"max_computation_time", 10.0},
		});
		planner_ = std::make_shared<mrpt_tps_astar_planner::TPS_Astar_Planner_Node>(opts);
		client_node_ = std::make_shared<rclcpp::Node>("test_goal_frame_client");

		tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(client_node_);
		geometry_msgs::msg::TransformStamped t;
		t.header.stamp = client_node_->now();
		t.header.frame_id = "map";
		t.child_frame_id = "base_link";
		t.transform.translation.x = ROBOT_X;
		t.transform.translation.y = ROBOT_Y;
		t.transform.rotation = yaw_quat(DEG2RAD(ROBOT_YAW_DEG));
		tf_broadcaster_->sendTransform(t);

		executor_.add_node(planner_);
		executor_.add_node(client_node_);
		spin_thread_ = std::thread([this]() { executor_.spin(); });

		client_ = client_node_->create_client<MakePlanTo>(
			std::string(planner_->get_fully_qualified_name()) + "/make_plan_to");
		// The service is only offered once the PTG lookup tables are built
		ASSERT_TRUE(client_->wait_for_service(120s));
	}

	void TearDown() override
	{
		executor_.cancel();
		spin_thread_.join();
	}

	MakePlanTo::Response::SharedPtr call(const geometry_msgs::msg::PoseStamped& target)
	{
		auto req = std::make_shared<MakePlanTo::Request>();
		req->target = target;
		auto fut = client_->async_send_request(req);
		if (fut.wait_for(60s) != std::future_status::ready)
		{
			return nullptr;
		}
		return fut.get();
	}

	// The PTG lookup tables are built in the background: retry until a plan to
	// a reachable goal (in the map frame) succeeds.
	void wait_for_planner_ready()
	{
		const auto deadline = std::chrono::steady_clock::now() + 120s;
		while (std::chrono::steady_clock::now() < deadline)
		{
			const auto r = call(target("map", ROBOT_X, ROBOT_Y + 1.0, ROBOT_YAW_DEG));
			if (r && r->valid_path_found)
			{
				return;
			}
			std::this_thread::sleep_for(500ms);
		}
		FAIL() << "Planner not ready in time";
	}

	static geometry_msgs::msg::PoseStamped target(
		const std::string& frame, double x, double y, double yaw_deg)
	{
		geometry_msgs::msg::PoseStamped g;
		g.header.frame_id = frame;	// zero stamp: latest transform
		g.pose.position.x = x;
		g.pose.position.y = y;
		g.pose.orientation = yaw_quat(DEG2RAD(yaw_deg));
		return g;
	}

	static void expect_final_waypoint(
		const MakePlanTo::Response::SharedPtr& r, double x, double y, double yaw_deg)
	{
		ASSERT_TRUE(r);
		ASSERT_TRUE(r->valid_path_found);
		ASSERT_FALSE(r->waypoints.waypoints.empty());
		const auto& last = r->waypoints.waypoints.back().target;
		EXPECT_NEAR(last.position.x, x, 1e-3);
		EXPECT_NEAR(last.position.y, y, 1e-3);
		EXPECT_NEAR(mrpt::math::wrapToPi(quat_yaw(last.orientation) - DEG2RAD(yaw_deg)), 0.0, 1e-3);
	}

	std::shared_ptr<mrpt_tps_astar_planner::TPS_Astar_Planner_Node> planner_;
	rclcpp::Node::SharedPtr client_node_;
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
	rclcpp::executors::MultiThreadedExecutor executor_;
	std::thread spin_thread_;
	rclcpp::Client<MakePlanTo>::SharedPtr client_;
};

}  // namespace

TEST_F(PlannerGoalFrameTest, TargetFrames)
{
	wait_for_planner_ready();

	// In the map frame, and without frame (legacy: taken as map):
	expect_final_waypoint(call(target("map", 6.0, 4.0, 0.0)), 6.0, 4.0, 0.0);
	expect_final_waypoint(call(target("", 6.0, 4.0, 0.0)), 6.0, 4.0, 0.0);

	// 2 m ahead of the robot (heading +y in the map), same heading:
	expect_final_waypoint(
		call(target("base_link", 2.0, 0.0, 0.0)), ROBOT_X, ROBOT_Y + 2.0, ROBOT_YAW_DEG);

	// Unknown frame: no plan, instead of a plan to a misread pose.
	const auto r = call(target("no_such_frame", 2.0, 0.0, 0.0));
	ASSERT_TRUE(r);
	EXPECT_FALSE(r->valid_path_found);
}
