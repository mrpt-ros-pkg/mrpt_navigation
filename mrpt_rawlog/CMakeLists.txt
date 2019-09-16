cmake_minimum_required(VERSION 3.1)
project(mrpt_rawlog)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp tf std_msgs nav_msgs sensor_msgs mrpt_msgs mrpt_bridge dynamic_reconfigure marker_msgs rosbag)

find_package(MRPT 1.5 REQUIRED gui obs)
message(STATUS "MRPT_VERSION: ${MRPT_VERSION}")
if(MRPT_VERSION VERSION_LESS "1.9.9")
  if(${CMAKE_VERSION} VERSION_LESS "3.8.0")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++1y")
  else()
	set (CMAKE_CXX_STANDARD 17)
  endif()
endif()

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

# add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/RawLogRecord.cfg
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
#  INCLUDE_DIRS include
  LIBRARIES rawlog_record rawlog_play
  CATKIN_DEPENDS roscpp tf std_msgs nav_msgs sensor_msgs mrpt_msgs mrpt_bridge marker_msgs dynamic_reconfigure
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
)
target_link_libraries(rawlog_record
  ${catkin_LIBRARIES}
  ${MRPT_LIBRARIES}
)

add_library(rawlog_play
   src/mrpt_rawlog_play/rawlog_play.cpp
   src/mrpt_rawlog_play/rawlog_play_parameters.cpp
)
target_link_libraries(rawlog_play
   ${catkin_LIBRARIES}
   ${MRPT_LIBRARIES}
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
   ${MRPT_LIBRARIES}
   rawlog_record
)

target_link_libraries(rawlog_play_node
   ${catkin_LIBRARIES}
   ${MRPT_LIBRARIES}
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

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
   tutorial
   launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
