cmake_minimum_required(VERSION 2.8.3)
project(mrpt_localization)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp tf std_msgs nav_msgs sensor_msgs mrpt_msgs mrpt_bridge pose_cov_ops dynamic_reconfigure)

## System dependencies are found with CMake's conventions
find_package(MRPT REQUIRED base gui obs slam)
#include_directories(${MRPT_INCLUDE_DIRS})
#MESSAGE( STATUS "MRPT_INCLUDE_DIRS: " ${MRPT_INCLUDE_DIRS})
#link_directories(${MRPT_LIBRARY_DIRS})
#MESSAGE( STATUS "MRPT_LIBRARY_DIRS: " ${MRPT_LIBS})

# Add dynamic reconfigure api
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
  LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_core
  CATKIN_DEPENDS roscpp tf std_msgs nav_msgs sensor_msgs mrpt_msgs mrpt_bridge pose_cov_ops dynamic_reconfigure
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


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME}
   mrpt_msgs_generate_messages_cpp
   ${catkin_EXPORTED_TARGETS}
   ${PROJECT_NAME}_gencfg
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_core
  ${MRPT_LIBS}
)

target_link_libraries(${PROJECT_NAME}
  ${MRPT_LIBS}
  ${PROJECT_NAME}_core
)

target_link_libraries(${PROJECT_NAME}_node
  ${MRPT_LIBS}
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

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
  PATTERN ".svn" EXCLUDE
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  tutorial
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
