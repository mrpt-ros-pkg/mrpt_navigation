cmake_minimum_required(VERSION 3.8)
project(mrpt_localization)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mrpt_msgs REQUIRED)
find_package(pose_cov_ops REQUIRED)
find_package(mrpt_msgs_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(mrpt-ros2bridge REQUIRED)
find_package(mrpt-gui REQUIRED)
find_package(mrpt-slam REQUIRED)

message(STATUS "MRPT_VERSION: ${mrpt-slam_VERSION}")

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

IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
  add_compile_options(-O3)
ENDIF()

###########
## Build ##
###########

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node
    src/${PROJECT_NAME}_node.cpp
    src/${PROJECT_NAME}_node_parameters.cpp
    src/${PROJECT_NAME}/${PROJECT_NAME}_core.cpp
    src/${PROJECT_NAME}/${PROJECT_NAME}.cpp
    src/${PROJECT_NAME}/${PROJECT_NAME}_parameters.cpp
    include/${PROJECT_NAME}/${PROJECT_NAME}_core.h
    include/${PROJECT_NAME}_node.h
)

ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp
    geometry_msgs
    mrpt_msgs
    mrpt_msgs_bridge
    nav_msgs
    pose_cov_ops
    sensor_msgs
    tf2
    tf2_geometry_msgs
)

target_include_directories(${PROJECT_NAME}_node
    PRIVATE
    include
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  mrpt::gui
  mrpt::slam
  mrpt::ros2bridge
)

#############
## Install ##
#############

install(
  TARGETS
    ${PROJECT_NAME}_node
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch tutorial
  DESTINATION share/${PROJECT_NAME}
)


#find_package(ament_cmake_gtest REQUIRED)
#ament_add_gtest(${PROJECT_NAME}-test test/test_pose_cov_ops.cpp)

ament_export_dependencies()

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
