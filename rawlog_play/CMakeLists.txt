cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_gensrv()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


# Declare the rawlog_play executable
# ----------------------------------------
FIND_PACKAGE(MRPT REQUIRED base obs)
rosbuild_add_executable(rawlog_play src/rawlog_play.cpp)
target_link_libraries(rawlog_play ${MRPT_LIBS})
