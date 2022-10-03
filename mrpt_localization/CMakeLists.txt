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

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_core
  CATKIN_DEPENDS
    roscpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    std_msgs
    nav_msgs
    sensor_msgs
    mrpt_msgs
    mrpt_msgs_bridge
    pose_cov_ops
    dynamic_reconfigure
#  DEPENDS mrpt
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(${PROJECT_NAME}_core
   src/${PROJECT_NAME}/${PROJECT_NAME}_core.cpp
)

add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/${PROJECT_NAME}.cpp
   src/${PROJECT_NAME}/${PROJECT_NAME}_parameters.cpp
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node
    src/${PROJECT_NAME}_node.cpp
    src/${PROJECT_NAME}_node_parameters.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}_core
    rclcpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    visualization_msgs
    tf2
    tf2_geometry_msgs
)



## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME}
   mrpt_msgs_generate_messages_cpp
   ${catkin_EXPORTED_TARGETS}
   ${PROJECT_NAME}_gencfg
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_core
  PUBLIC
  mrpt::gui
  mrpt::slam
  mrpt::ros2bridge
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_core
)

target_link_libraries(${PROJECT_NAME}_node
  PRIVATE
  ${PROJECT_NAME}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

set(CATKIN_PACKAGE_LIB_DESTINATION lib)
set(CATKIN_PACKAGE_BIN_DESTINATION lib/${PROJECT_NAME})
set(CATKIN_PACKAGE_INCLUDE_DESTINATION include)
set(CATKIN_PACKAGE_SHARE_DESTINATION share/${PROJECT_NAME})

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_core ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  tutorial
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

find_package(ament_cmake_xmllint REQUIRED)

#ament_copyright()
#ament_cpplint()
#ament_lint_cmake()
ament_xmllint()

#find_package(ament_cmake_gtest REQUIRED)
#ament_add_gtest(${PROJECT_NAME}-test test/test_pose_cov_ops.cpp)

ament_export_dependencies()

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})

# Export modern CMake targets
ament_export_targets(export_${PROJECT_NAME})

ament_package()