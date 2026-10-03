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
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mrpt_nav_interfaces REQUIRED)

## System dependencies are found with CMake's conventions
find_package(mrpt_libros_bridge REQUIRED)
find_package(mrpt_maps REQUIRED)
find_package(mrpt_math REQUIRED)
find_package(mrpt_system REQUIRED)
find_package(mrpt_containers REQUIRED)

find_package(mrpt_path_planning REQUIRED)

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
# Node implementation, loadable as a composable node:
add_library(${PROJECT_NAME}_component SHARED
  src/${PROJECT_NAME}_component.cpp
)

target_include_directories(${PROJECT_NAME}_component
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(${PROJECT_NAME}_component PUBLIC
  mrpt_libros_bridge::mrpt_libros_bridge
  mrpt::mrpt_maps
  mrpt::mrpt_math
  mrpt::mrpt_system
  mrpt::mrpt_containers
  mpp::mrpt_path_planning
  rclcpp::rclcpp
  ${rclcpp_components_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  tf2::tf2
  ${tf2_ros_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
  ${mrpt_nav_interfaces_TARGETS}
)

rclcpp_components_register_node(
  ${PROJECT_NAME}_component
  PLUGIN "mrpt_trajectory_follower::TrajectoryFollowerNode"
  EXECUTABLE ${PROJECT_NAME}_composable
)

# Standalone executable:
add_executable(${PROJECT_NAME}_node
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}_node PRIVATE
  ${PROJECT_NAME}_component
  rclcpp::rclcpp
)

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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
