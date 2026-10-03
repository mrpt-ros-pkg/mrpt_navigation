/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include <mrpt_tps_astar_planner/mrpt_tps_astar_planner_node.hpp>

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<mrpt_tps_astar_planner::TPS_Astar_Planner_Node>();
	// Multi-threaded so planning service calls can run concurrently:
	rclcpp::executors::MultiThreadedExecutor exec;
	exec.add_node(node);
	exec.spin();
	rclcpp::shutdown();
	return 0;
}
