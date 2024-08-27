cmake_minimum_required(VERSION 3.5)
project(mrpt_tps_astar_planner)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs)
find_package(sensor_msgs REQUIRED)
find_package(mrpt_msgs REQUIRED)
find_package(mrpt_nav_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


## System dependencies are found with CMake's conventions
find_package(mrpt-ros2bridge REQUIRED)
find_package(mrpt-nav REQUIRED)
find_package(mrpt-kinematics REQUIRED)
find_package(mrpt-math REQUIRED)
find_package(mrpt-system REQUIRED)
find_package(mrpt-containers REQUIRED)
find_package(mrpt-maps REQUIRED)
find_package(mrpt-gui REQUIRED)
find_package(mrpt-opengl REQUIRED)

find_package(mrpt_path_planning REQUIRED)
if (TARGET mrpt_path_planning AND NOT TARGET mrpt_path_planning::mrpt_path_planning)
  add_library(mrpt_path_planning::mrpt_path_planning ALIAS mrpt_path_planning)
endif()

message(STATUS "MRPT_VERSION: ${MRPT_VERSION}")

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if (CMAKE_COMPILER_IS_GNUCXX)
	# High level of warnings.
	# The -Wno-long-long is required in 64bit systems when including sytem headers.
	# The -Wno-variadic-macros was needed for Eigen3, StdVector.h
	add_compile_options(-Wall -Wno-long-long -Wno-variadic-macros)
	# Workaround: Eigen <3.4 produces *tons* of warnings in GCC >=6. See http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1221
	if (NOT ${CMAKE_CXX_COMPILER_VERSION} LESS "6.0")
		add_compile_options(-Wno-ignored-attributes -Wno-int-in-bool-context)
	endif()
endif()

###########
## Build ##
###########

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node
  src/mrpt_tps_astar_planner_node.cpp
)

#target_include_directories(${PROJECT_NAME}_node
#                          PUBLIC
#                          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#                          $<INSTALL_INTERFACE:include>
#)

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  mrpt::nav
  mrpt::kinematics
  mrpt::ros2bridge
  mrpt_path_planning::mrpt_path_planning
)

ament_target_dependencies(
  ${PROJECT_NAME}_node
  rclcpp
  rclcpp_components
  nav_msgs
  sensor_msgs
  geometry_msgs
  mrpt_msgs
  mrpt_nav_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html


## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  configs
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
