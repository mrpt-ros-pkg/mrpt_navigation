cmake_minimum_required(VERSION 3.5)
project(mrpt_pointcloud_pipeline)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

## System dependencies are found with CMake's conventions
find_package(mrpt-maps REQUIRED)
find_package(mrpt-obs REQUIRED)
find_package(mrpt-gui REQUIRED)
find_package(mrpt-ros2bridge REQUIRED)

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
	add_compile_options(-Wall -Wno-long-long -Wno-variadic-macros -Wextra -Wpedantic)
	# Workaround: Eigen <3.4 produces *tons* of warnings in GCC >=6. See http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1221
	if (NOT ${CMAKE_CXX_COMPILER_VERSION} LESS "6.0")
		add_compile_options(-Wno-ignored-attributes -Wno-int-in-bool-context)
	endif()
endif()
IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
  add_compile_options(-O3)
ENDIF()


find_package(mp2p_icp_filters REQUIRED)

add_executable(${PROJECT_NAME}_node
              src/mrpt_pointcloud_pipeline_node.cpp
              include/mrpt_pointcloud_pipeline/mrpt_pointcloud_pipeline_node.h)

target_include_directories(${PROJECT_NAME}_node
                           PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  ${PROJECT_NAME}_node
  mrpt::maps
  mrpt::obs
  mrpt::gui
  mrpt::ros2bridge
  mola::mp2p_icp_filters
)

ament_target_dependencies(
  ${PROJECT_NAME}_node
  rclcpp
  rclcpp_components
  nav_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
)


install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    params
  DESTINATION share/${PROJECT_NAME}
)

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

