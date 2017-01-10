cmake_minimum_required(VERSION 2.8.3)
project(mrpt_rawlog)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp tf std_msgs nav_msgs sensor_msgs mrpt_msgs mrpt_bridge dynamic_reconfigure)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
# MRPT
find_package(MRPT REQUIRED base gui obs)
#include_directories(${MRPT_INCLUDE_DIRS})
#MESSAGE( STATUS "MRPT_INCLUDE_DIRS: " ${MRPT_INCLUDE_DIRS})
#link_directories(${MRPT_LIBRARY_DIRS})
#MESSAGE( STATUS "MRPT_LIBRARY_DIRS: " ${MRPT_LIBS})

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/Motion.cfg
)

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
  LIBRARIES rawlog_record rawlog_play
  CATKIN_DEPENDS roscpp tf std_msgs nav_msgs sensor_msgs mrpt_msgs mrpt_bridge dynamic_reconfigure
#   DEPENDS mrpt
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(rawlog_record
   src/mrpt_rawlog_record/rawlog_record.cpp
   src/mrpt_rawlog_record/rawlog_record_parameters.cpp
)
target_link_libraries(rawlog_record
  ${catkin_LIBRARIES}
  ${MRPT_LIBS}
)

add_library(rawlog_play
   src/mrpt_rawlog_play/rawlog_play.cpp
   src/mrpt_rawlog_play/rawlog_play_parameters.cpp
)
target_link_libraries(rawlog_play
   ${catkin_LIBRARIES}
   ${MRPT_LIBS}
)

## Declare a cpp executable
add_executable(rawlog_record_node
   src/rawlog_record_node.cpp
   src/rawlog_record_node_parameters.cpp
)

add_executable(rawlog_play_node
   src/rawlog_play_node.cpp
   src/rawlog_play_node_parameters.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(template_node template_generate_messages_cpp)
add_dependencies(rawlog_record_node
   mrpt_msgs_generate_messages_cpp
   ${PROJECT_NAME}_gencfg
)

add_dependencies(rawlog_play_node
   mrpt_msgs_generate_messages_cpp
   ${PROJECT_NAME}_gencfg
)

## Specify libraries to link a library or executable target against
target_link_libraries(rawlog_record_node
   ${catkin_LIBRARIES}
   ${MRPT_LIBS}
   rawlog_record
)

target_link_libraries(rawlog_play_node
   ${catkin_LIBRARIES}
   ${MRPT_LIBS}
   rawlog_play
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executables and/or libraries for installation
install(TARGETS rawlog_record_node rawlog_play_node rawlog_record rawlog_play
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
   tutorial
   launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
