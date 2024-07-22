cmake_minimum_required(VERSION 3.5)

project(mrpt_nav_interfaces)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(mrpt_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateGoal.action"
  "action/NavigateWaypoints.action"
  "msg/NavigationFeedback.msg"
  "msg/NavigationFinalStatus.msg"
  "srv/GetLayers.srv"
  "srv/GetGridmapLayer.srv"
  "srv/GetPointmapLayer.srv"
  "srv/MakePlanFromTo.srv"
  "srv/MakePlanTo.srv"
  DEPENDENCIES
  std_msgs
  nav_msgs
  mrpt_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
