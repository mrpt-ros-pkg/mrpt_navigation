/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include "rclcpp/rclcpp.hpp"
#include "mrpt_pf_localization_component.cpp"

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<PFLocalizationNode>();
	rclcpp::spin(std::dynamic_pointer_cast<rclcpp::Node>(node));
	rclcpp::shutdown();
	return 0;
}