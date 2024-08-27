cmake_minimum_required(VERSION 3.5)
project(mrpt_map_server)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mrpt_msgs REQUIRED)
find_package(mrpt_nav_interfaces REQUIRED)

find_package(mrpt-maps REQUIRED)
find_package(mrpt-ros2bridge REQUIRED)
find_package(mp2p_icp_map REQUIRED)

message(STATUS "MRPT_VERSION: ${MRPT_VERSION}")

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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

###########
## Build ##
###########

## Declare a cpp executable
add_executable(map_server_node
				src/map_server_node.cpp
				include/${PROJECT_NAME}/map_server_node.h)

target_include_directories(map_server_node
				PUBLIC
				$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
				$<INSTALL_INTERFACE:include>
)

## Specify libraries to link a library or executable target against
target_link_libraries(map_server_node
  mrpt::maps
  mrpt::ros2bridge
  mola::mp2p_icp_map
)

ament_target_dependencies(
  map_server_node
  rclcpp
  rclcpp_components
  nav_msgs
  mrpt_msgs
  mrpt_nav_interfaces
)

#############
## Install ##
#############

install(TARGETS map_server_node
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}
)

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
