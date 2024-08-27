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
find_package(mrpt_msgs REQUIRED)

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
  rclcpp
  tf2
  nav_msgs
  sensor_msgs
  rosbag2_cpp
  tf2_msgs
  cv_bridge
  tf2_geometry_msgs
  mrpt_msgs
)


# ===================================================================================================
# Credits: https://gist.github.com/jtanx/96ded5e050d5ee5b19804195ee5cf5f9
function(pad_string output str padchar length)
  string(LENGTH "${str}" _strlen)
  math(EXPR _strlen "${length} - ${_strlen}")

  if(_strlen GREATER 0)
    string(REPEAT ${padchar} ${_strlen} _pad)
    string(PREPEND str ${_pad})
  endif()

  set(${output} "${str}" PARENT_SCOPE)
endfunction()
# 2 hex digits for each version part:
# For example: "0.5.1"  => "0x000501"
macro(mrpt_version_to_hex VER TARGET_VAR_NAME)
	string(REGEX MATCHALL "[0-9]+" __parts "${${VER}}")

	if(__parts)
		list(GET __parts 0 __VERSION_NUMBER_MAJOR)
		list(GET __parts 1 __VERSION_NUMBER_MINOR)
		list(GET __parts 2 __VERSION_NUMBER_PATCH)
		pad_string(__VERSION_NUMBER_MAJOR ${__VERSION_NUMBER_MAJOR} "0" 2)
		pad_string(__VERSION_NUMBER_MINOR ${__VERSION_NUMBER_MINOR} "0" 2)
		pad_string(__VERSION_NUMBER_PATCH ${__VERSION_NUMBER_PATCH} "0" 2)
		set(${TARGET_VAR_NAME} "0x${__VERSION_NUMBER_MAJOR}${__VERSION_NUMBER_MINOR}${__VERSION_NUMBER_PATCH}")
	else()
		set(${TARGET_VAR_NAME} "0x000")
	endif()
endmacro()

# Convert package versions to hex so they can be used in preprocessor for wider
# versions compatibility of "one-source for all":
mrpt_version_to_hex(cv_bridge_VERSION  cv_bridge_VERSION_HEX)
target_compile_definitions(rosbag2rawlog PRIVATE CV_BRIDGE_VERSION=${cv_bridge_VERSION_HEX})
# ===================================================================================================


install(TARGETS rosbag2rawlog
  DESTINATION bin
)

#install(
#  DIRECTORY launch
#  DESTINATION share/${PROJECT_NAME}
#)

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
