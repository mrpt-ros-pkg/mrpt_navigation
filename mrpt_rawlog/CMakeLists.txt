cmake_minimum_required(VERSION 3.8)
project(mrpt_rawlog)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(mrpt-ros2bridge REQUIRED)
message(STATUS "MRPT_VERSION: ${MRPT_VERSION}")

if (CMAKE_COMPILER_IS_GNUCXX)
	add_compile_options(-Wall)
endif()

IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
  add_compile_options(-O3)
ENDIF()

######################
## rosbag2rawlog    ##
######################
find_package(mrpt-tclap REQUIRED)

add_executable(rosbag2rawlog
              src/rosbag2rawlog.cpp)

target_link_libraries(
  rosbag2rawlog
  mrpt::ros2bridge
  mrpt::tclap
)

ament_target_dependencies(
  rosbag2rawlog
  #mrpt2
  rclcpp
  tf2
  nav_msgs
  sensor_msgs
  rosbag2_cpp
  tf2_msgs
  cv_bridge
  tf2_geometry_msgs
)

install(TARGETS rosbag2rawlog
  DESTINATION bin
)

install(
  DIRECTORY launch
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
