cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(xsens_lib src/xsens_lib/xsens_driver.cpp src/xsens_lib/cmt1.cpp src/xsens_lib/cmt2.cpp src/xsens_lib/cmt3.cpp src/xsens_lib/cmtmessage.cpp src/xsens_lib/cmtpacket.cpp src/xsens_lib/cmtscan.cpp src/xsens_lib/xsens_std.cpp src/xsens_lib/xsens_time.cpp)
#rosbuild_add_library(${PROJECT_NAME} include/cmt1.cpp)
#rosbuild_add_library(${PROJECT_NAME} include/cmt2.cpp)
#rosbuild_add_library(${PROJECT_NAME} include/cmt3.cpp)
#rosbuild_add_library(${PROJECT_NAME} include/cmtmessage.cpp)
#rosbuild_add_library(${PROJECT_NAME} include/cmtpacket.cpp)
#rosbuild_add_library(${PROJECT_NAME} include/cmtscan.cpp)
#rosbuild_add_library(${PROJECT_NAME} include/xsens_std.cpp)
#rosbuild_add_library(${PROJECT_NAME} include/xsens_time.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(xsens_node src/xsens_node.cpp)
target_link_libraries(xsens_node xsens_lib)
