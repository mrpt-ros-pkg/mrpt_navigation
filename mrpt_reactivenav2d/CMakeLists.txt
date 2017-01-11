cmake_minimum_required(VERSION 2.8.3)
project(mrpt_reactivenav2d)
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  dynamic_reconfigure
  geometry_msgs
  mrpt_bridge
  roscpp
  tf
  visualization_msgs
)

## System dependencies are found with CMake's conventions
find_package(MRPT REQUIRED base reactivenav kinematics)
MESSAGE( STATUS "MRPT_VERSION: ${MRPT_VERSION}")
MESSAGE( STATUS "MRPT_INCLUDE_DIRS: ${MRPT_INCLUDE_DIRS}")

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
  CATKIN_DEPENDS actionlib actionlib_msgs dynamic_reconfigure geometry_msgs mrpt_bridge roscpp tf visualization_msgs
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
  ${MRPT_LIBS}
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
