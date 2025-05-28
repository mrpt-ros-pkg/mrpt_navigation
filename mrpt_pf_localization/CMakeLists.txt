cmake_minimum_required(VERSION 3.8)
project(mrpt_pf_localization)

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
find_package(mp2p_icp_map REQUIRED)
find_package(mp2p_icp_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mola_relocalization) # OPTIONAL

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


message(STATUS "mola_relocalization_FOUND: ${mola_relocalization_FOUND}")

###########
## Build ##
###########

# non-ROS C++ library:
add_library(${PROJECT_NAME}_core SHARED
    src/${PROJECT_NAME}/${PROJECT_NAME}_core.cpp
    include/${PROJECT_NAME}/${PROJECT_NAME}_core.h
)

target_include_directories(${PROJECT_NAME}_core
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>  
    $<INSTALL_INTERFACE:include/>
)

target_link_libraries(${PROJECT_NAME}_core
  mrpt::gui
  mrpt::slam
  mrpt::ros2bridge
  mola::mp2p_icp_map
  mola::mp2p_icp_filters
)
if (mola_relocalization_FOUND)
  target_link_libraries(${PROJECT_NAME}_core
    mola::mola_relocalization
  )
  target_compile_definitions(${PROJECT_NAME}_core PRIVATE HAVE_MOLA_RELOCALIZATION)
endif()


# ROS node:
add_executable(${PROJECT_NAME}_node
  src/main.cpp
  include/${PROJECT_NAME}_node.h
)

target_link_libraries(${PROJECT_NAME}_node PRIVATE
  ${PROJECT_NAME}_core
  mrpt::gui
  mrpt::slam
  mrpt::ros2bridge
  rclcpp::rclcpp
  ${rclcpp_components_TARGETS}
  tf2::tf2
  ${pose_cov_ops_TARGETS}
  ${geometry_msgs_TARGETS}
  ${mrpt_msgs_TARGETS}
  ${mrpt_msgs_bridge_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
)

target_include_directories(${PROJECT_NAME}_node
  PRIVATE
  include
)

#######################
# ROS composable node #
#######################

add_library(${PROJECT_NAME}_component SHARED
    src/${PROJECT_NAME}_component.cpp
    include/${PROJECT_NAME}_node.h
)

target_link_libraries(${PROJECT_NAME}_component PRIVATE
  ${PROJECT_NAME}_core
  mrpt::gui
  mrpt::slam
  mrpt::ros2bridge
  rclcpp::rclcpp
  ${rclcpp_components_TARGETS}
  tf2::tf2
  ${pose_cov_ops_TARGETS}
  ${geometry_msgs_TARGETS}
  ${mrpt_msgs_TARGETS}
  ${mrpt_msgs_bridge_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
)

target_include_directories(${PROJECT_NAME}_component
  PRIVATE
  include
)

rclcpp_components_register_node(
  ${PROJECT_NAME}_component
  PLUGIN "PFLocalizationNode"
  EXECUTABLE ${PROJECT_NAME}_composable
)

ament_export_targets(export_${PROJECT_NAME})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_component ${PROJECT_NAME}_core ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(
    ${PROJECT_NAME}-test test/test_pf_localization.cpp
  )
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_core)
  target_compile_definitions(${PROJECT_NAME}-test PRIVATE MRPT_LOCALIZATION_SOURCE_DIR=\"${CMAKE_SOURCE_DIR}\")
  install(
    TARGETS ${PROJECT_NAME}-test
    DESTINATION bin
  )

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
