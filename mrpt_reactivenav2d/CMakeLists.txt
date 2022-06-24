cmake_minimum_required(VERSION 3.1)
project(mrpt_reactivenav2d)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  dynamic_reconfigure
  geometry_msgs
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

## System dependencies are found with CMake's conventions
find_package(mrpt-ros1bridge REQUIRED)
find_package(mrpt-nav REQUIRED)
find_package(mrpt-kinematics REQUIRED)

message(STATUS "MRPT_VERSION: ${MRPT_VERSION}")

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
#  INCLUDE_DIRS include
#  LIBRARIES mrpt_reactivenav2d
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    dynamic_reconfigure
    geometry_msgs
    roscpp
    tf2
    tf2_ros
    tf2_geometry_msgs
      visualization_msgs
#  DEPENDS mrpt
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(mrpt_reactivenav2d
#   src/${PROJECT_NAME}/mrpt_reactivenav2d.cpp
# )

## Declare a cpp executable
add_executable(mrpt_reactivenav2d_node src/mrpt_reactivenav2d_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(mrpt_reactivenav2d_node mrpt_reactivenav2d_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(mrpt_reactivenav2d_node
  ${catkin_LIBRARIES}
  mrpt::nav
  mrpt::kinematics
  mrpt::ros1bridge
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html


## Mark executables and/or libraries for installation
install(TARGETS mrpt_reactivenav2d_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  tutorial
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
