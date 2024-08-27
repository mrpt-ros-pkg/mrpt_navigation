cmake_minimum_required(VERSION 3.8)
project(mrpt_msgs_bridge)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(mrpt_msgs REQUIRED)

# Optional:
find_package(marker_msgs QUIET)
if(marker_msgs_FOUND)
    set(MARKER_SRC src/marker_msgs.cpp)
    set(MARKER_DEP marker_msgs)
endif()

find_package(mrpt-ros2bridge REQUIRED)
find_package(mrpt-obs REQUIRED)

message(STATUS "MRPT_VERSION: ${mrpt-obs_VERSION}")

if(CMAKE_COMPILER_IS_GNUCXX)
  # High level of warnings.
  # The -Wno-long-long is required in 64bit systems when including sytem headers.
  # The -Wno-variadic-macros was needed for Eigen3, StdVector.h
  add_compile_options(-Wall -Wno-long-long -Wno-variadic-macros)
  # Workaround: Eigen <3.4 produces *tons* of warnings in GCC >=6. See http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1221
  if(NOT ${CMAKE_CXX_COMPILER_VERSION} LESS "6.0")
    add_compile_options(-Wno-ignored-attributes -Wno-int-in-bool-context)
  endif()
endif()

if(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
  add_compile_options(-O3)
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare a cpp library
add_library(${PROJECT_NAME}
  src/beacon.cpp
  ${MARKER_SRC}
  src/landmark.cpp
  src/network_of_poses.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  mrpt::obs
  mrpt::ros2bridge
)


#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)

#############
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tf2
  mrpt_msgs
  ${MARKER_DEP}
)

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME})

ament_export_dependencies()

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
