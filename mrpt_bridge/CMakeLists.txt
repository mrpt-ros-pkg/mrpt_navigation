cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


# We need the MRPT libraries installed in the system
# and anywhere where CMake finds them (/usr, /usr/local, ...)

rosbuild_add_library(mrpt_bridge src/pose_conversions.cpp src/point_cloud.cpp src/point_cloud2.cpp src/laser_scan.cpp)

rosbuild_add_gtest(test_mrpt_bridge src/test/test_main.cpp)
rosbuild_add_gtest(test_mrpt_bridge src/test/test_pose_conversions.cpp)
rosbuild_add_gtest(test_mrpt_bridge src/test/test_pointcloud2.cpp)
target_link_libraries(test_mrpt_bridge mrpt_bridge)

