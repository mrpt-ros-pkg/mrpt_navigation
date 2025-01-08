/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include "rclcpp/rclcpp.hpp"
#include "mrpt_pointcloud_pipeline_component.cpp"

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

	auto node = std::make_shared<LocalObstaclesNode>();

	rclcpp::spin(node->get_node_base_interface());

	rclcpp::shutdown();

	return 0;
}