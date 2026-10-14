/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2026, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/ros2bridge/point_cloud2.h>
#include <mrpt_pointcloud_pipeline/mrpt_pointcloud_pipeline_node.h>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <cmath>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace
{
const std::string kCloudTopic = "/test_pose_at_stamp/cloud";
const std::string kOutputTopic = "/test_pose_at_stamp/local_map";

geometry_msgs::msg::TransformStamped odom_to_robot(
	const rclcpp::Time& t, double x, double y, double yaw)
{
	geometry_msgs::msg::TransformStamped tf;
	tf.header.stamp = t;
	tf.header.frame_id = "odom";
	tf.child_frame_id = "base_link";
	tf.transform.translation.x = x;
	tf.transform.translation.y = y;
	tf.transform.rotation.z = std::sin(0.5 * yaw);
	tf.transform.rotation.w = std::cos(0.5 * yaw);
	return tf;
}

void spin_for(rclcpp::Executor& ex, std::chrono::milliseconds d)
{
	const auto end = std::chrono::steady_clock::now() + d;
	while (std::chrono::steady_clock::now() < end)
	{
		ex.spin_some(std::chrono::milliseconds(10));
	}
}
}  // namespace

// Clouds sensed while the robot turns must each be placed with the pose at
// their timestamp, not with the latest one.
TEST(PointCloudPipeline, UsesRobotPoseAtSensorStamp)
{
	rclcpp::NodeOptions opts;
	opts.parameter_overrides({
		{"pipeline_yaml_file", std::string(TEST_PIPELINE_YAML)},
		{"source_topics_2d_scans", std::string("")},
		{"source_topics_pointclouds", kCloudTopic},
		{"filter_output_layer_name", std::string("output")},
		{"topic_local_map_pointcloud", kOutputTopic},
		{"time_window", 5.0},
		{"publish_period", 0.05},
	});
	auto pipeline = std::make_shared<LocalObstaclesNode>(opts);
	auto helper = std::make_shared<rclcpp::Node>("test_pose_at_stamp_helper");

	std::optional<mrpt::maps::CSimplePointsMap> output;
	auto sub = helper->create_subscription<sensor_msgs::msg::PointCloud2>(
		kOutputTopic, 10,
		[&output](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
		{
			mrpt::maps::CSimplePointsMap m;
			// Wait for the map with both observations:
			if (mrpt::ros2bridge::fromROS(*msg, m) && m.size() == 2)
			{
				output = m;
			}
		});
	auto pub = helper->create_publisher<sensor_msgs::msg::PointCloud2>(kCloudTopic, 10);
	tf2_ros::TransformBroadcaster tf_pub(*helper);

	rclcpp::executors::SingleThreadedExecutor ex;
	ex.add_node(pipeline);
	ex.add_node(helper);

	// The robot turns 90 deg in place between t0 and t1:
	const rclcpp::Time t1 = helper->now();
	const rclcpp::Time t0 = t1 - rclcpp::Duration::from_seconds(0.5);
	tf_pub.sendTransform({odom_to_robot(t0, 0, 0, 0), odom_to_robot(t1, 0, 0, M_PI / 2)});
	spin_for(ex, std::chrono::milliseconds(300));

	const auto publish_point = [&](const rclcpp::Time& t, float x, float y)
	{
		mrpt::maps::CSimplePointsMap pts;
		pts.insertPoint(x, y, 0.5f);
		std_msgs::msg::Header h;
		h.stamp = t;
		h.frame_id = "base_link";
		sensor_msgs::msg::PointCloud2 msg;
		ASSERT_TRUE(mrpt::ros2bridge::toROS(pts, h, msg));
		pub->publish(msg);
		spin_for(ex, std::chrono::milliseconds(100));
	};
	// An obstacle 3 m ahead sensed at t0, and another one 2 m ahead at t1:
	publish_point(t0, 3.0f, 0.0f);
	publish_point(t1, 2.0f, 0.0f);

	const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
	while (!output && std::chrono::steady_clock::now() < end)
	{
		ex.spin_some(std::chrono::milliseconds(10));
	}
	ASSERT_TRUE(output.has_value());

	// Relative to the pose at t1 (facing +Y), the first one is on the right:
	bool foundFirst = false;
	bool foundSecond = false;
	for (size_t i = 0; i < output->size(); i++)
	{
		float x = 0;
		float y = 0;
		float z = 0;
		output->getPoint(i, x, y, z);
		foundFirst = foundFirst || (std::abs(x) < 0.05 && std::abs(y + 3.0) < 0.05);
		foundSecond = foundSecond || (std::abs(x - 2.0) < 0.05 && std::abs(y) < 0.05);
	}
	EXPECT_TRUE(foundFirst);
	EXPECT_TRUE(foundSecond);
}

// The odometry for the time of a cloud may arrive after the cloud itself: the
// cloud must wait for it instead of being placed with an older pose.
TEST(PointCloudPipeline, WaitsForLateRobotPose)
{
	rclcpp::NodeOptions opts;
	opts.parameter_overrides({
		{"pipeline_yaml_file", std::string(TEST_PIPELINE_YAML)},
		{"source_topics_2d_scans", std::string("")},
		{"source_topics_pointclouds", kCloudTopic},
		{"filter_output_layer_name", std::string("output")},
		{"topic_local_map_pointcloud", kOutputTopic},
		{"time_window", 5.0},
		{"publish_period", 0.05},
		{"one_observation_per_topic", true},
	});
	auto pipeline = std::make_shared<LocalObstaclesNode>(opts);
	auto helper = std::make_shared<rclcpp::Node>("test_late_pose_helper");

	std::optional<mrpt::maps::CSimplePointsMap> output;
	auto sub = helper->create_subscription<sensor_msgs::msg::PointCloud2>(
		kOutputTopic, 10,
		[&output](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
		{
			mrpt::maps::CSimplePointsMap m;
			if (mrpt::ros2bridge::fromROS(*msg, m) && m.size() > 0)
			{
				output = m;
			}
		});
	auto pub = helper->create_publisher<sensor_msgs::msg::PointCloud2>(kCloudTopic, 10);
	tf2_ros::TransformBroadcaster tf_pub(*helper);

	rclcpp::executors::SingleThreadedExecutor ex;
	ex.add_node(pipeline);
	ex.add_node(helper);

	// Only the pose at t0 is known when the cloud sensed at t1 arrives:
	const rclcpp::Time t1 = helper->now();
	const rclcpp::Time t0 = t1 - rclcpp::Duration::from_seconds(0.5);
	tf_pub.sendTransform(odom_to_robot(t0, 0, 0, 0));
	spin_for(ex, std::chrono::milliseconds(300));

	mrpt::maps::CSimplePointsMap pts;
	pts.insertPoint(3.0f, 0.0f, 0.5f);
	std_msgs::msg::Header h;
	h.stamp = t1;
	h.frame_id = "base_link";
	sensor_msgs::msg::PointCloud2 msg;
	ASSERT_TRUE(mrpt::ros2bridge::toROS(pts, h, msg));
	pub->publish(msg);
	spin_for(ex, std::chrono::milliseconds(100));

	// The robot turned 90 deg in place by t1:
	tf_pub.sendTransform(odom_to_robot(t1, 0, 0, M_PI / 2));

	// Check the local maps built once that pose is known:
	spin_for(ex, std::chrono::milliseconds(100));
	output.reset();
	spin_for(ex, std::chrono::milliseconds(500));
	ASSERT_TRUE(output.has_value());

	// The only observation is the latest one: still 3 m ahead.
	ASSERT_EQ(output->size(), 1u);
	float x = 0;
	float y = 0;
	float z = 0;
	output->getPoint(0, x, y, z);
	EXPECT_NEAR(x, 3.0, 0.05);
	EXPECT_NEAR(y, 0.0, 0.05);
}

// An observation whose robot pose never arrives is dropped, not misplaced.
TEST(PointCloudPipeline, DropsObservationWithoutRobotPose)
{
	rclcpp::NodeOptions opts;
	opts.parameter_overrides({
		{"pipeline_yaml_file", std::string(TEST_PIPELINE_YAML)},
		{"source_topics_2d_scans", std::string("")},
		{"source_topics_pointclouds", kCloudTopic},
		{"filter_output_layer_name", std::string("output")},
		{"topic_local_map_pointcloud", kOutputTopic},
		{"time_window", 5.0},
		{"publish_period", 0.05},
		{"tf_max_wait", 0.3},
	});
	auto pipeline = std::make_shared<LocalObstaclesNode>(opts);
	auto helper = std::make_shared<rclcpp::Node>("test_drop_helper");

	bool gotPoints = false;
	auto sub = helper->create_subscription<sensor_msgs::msg::PointCloud2>(
		kOutputTopic, 10,
		[&gotPoints](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
		{
			mrpt::maps::CSimplePointsMap m;
			if (mrpt::ros2bridge::fromROS(*msg, m) && m.size() > 0)
			{
				gotPoints = true;
			}
		});
	auto pub = helper->create_publisher<sensor_msgs::msg::PointCloud2>(kCloudTopic, 10);
	tf2_ros::TransformBroadcaster tf_pub(*helper);

	rclcpp::executors::SingleThreadedExecutor ex;
	ex.add_node(pipeline);
	ex.add_node(helper);

	// The odometry stops at t0, and a cloud sensed later arrives:
	const rclcpp::Time t1 = helper->now();
	const rclcpp::Time t0 = t1 - rclcpp::Duration::from_seconds(1.0);
	tf_pub.sendTransform(odom_to_robot(t0, 0, 0, 0));
	spin_for(ex, std::chrono::milliseconds(300));

	mrpt::maps::CSimplePointsMap pts;
	pts.insertPoint(3.0f, 0.0f, 0.5f);
	std_msgs::msg::Header h;
	h.stamp = t1;
	h.frame_id = "base_link";
	sensor_msgs::msg::PointCloud2 msg;
	ASSERT_TRUE(mrpt::ros2bridge::toROS(pts, h, msg));
	pub->publish(msg);

	spin_for(ex, std::chrono::milliseconds(1000));
	EXPECT_FALSE(gotPoints);
}

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	rclcpp::init(argc, argv);
	const int ret = RUN_ALL_TESTS();
	rclcpp::shutdown();
	return ret;
}
