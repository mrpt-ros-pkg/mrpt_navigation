/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2026, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#if __has_include(<mpp/algos/TrajectoryFollower.h>)

#include <mrpt_trajectory_follower/mrpt_trajectory_follower_node.hpp>

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<mrpt_trajectory_follower::TrajectoryFollowerNode>());
	rclcpp::shutdown();
	return 0;
}

#else  // mpp::TrajectoryFollower not available in the linked mrpt_path_planning

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	RCLCPP_FATAL(
		rclcpp::get_logger("mrpt_trajectory_follower"),
		"This node requires a newer mrpt_path_planning providing "
		"mpp::TrajectoryFollower (mpp/algos/TrajectoryFollower.h). "
		"Update mrpt_path_planning and rebuild.");
	rclcpp::shutdown();
	return 1;
}

#endif	// __has_include(<mpp/algos/TrajectoryFollower.h>)
