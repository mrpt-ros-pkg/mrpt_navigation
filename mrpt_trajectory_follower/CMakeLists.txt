cmake_minimum_required(VERSION 3.22...3.31)  # Ubuntu 22.04+
project(mrpt_trajectory_follower)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
  set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

## System dependencies are found with CMake's conventions
find_package(mrpt-ros2bridge REQUIRED)
find_package(mrpt-maps REQUIRED)
find_package(mrpt-math REQUIRED)
find_package(mrpt-system REQUIRED)
find_package(mrpt-containers REQUIRED)

find_package(mrpt_path_planning REQUIRED)
if(TARGET mrpt_path_planning AND NOT TARGET mrpt_path_planning::mrpt_path_planning)
  add_library(mrpt_path_planning::mrpt_path_planning ALIAS mrpt_path_planning)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX)
  add_compile_options(-Wall -Wno-long-long -Wno-variadic-macros)
  if(NOT ${CMAKE_CXX_COMPILER_VERSION} LESS "6.0")
    add_compile_options(-Wno-ignored-attributes -Wno-int-in-bool-context)
  endif()
endif()

###########
## Build ##
###########
add_executable(${PROJECT_NAME}_node
  src/mrpt_trajectory_follower_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node PRIVATE
  mrpt::ros2bridge
  mrpt::maps
  mrpt::math
  mrpt::system
  mrpt::containers
  mrpt_path_planning::mrpt_path_planning
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  tf2::tf2
  ${tf2_ros_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
)

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

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
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
