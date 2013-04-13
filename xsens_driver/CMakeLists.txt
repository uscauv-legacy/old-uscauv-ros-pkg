cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# uncomment if you have defined messages
#rosbuild_genmsg()
# uncomment if you have defined services
#rosbuild_gensrv()
# uncomment if you have defined dynamic reconfigure files
# Note: requires "rosbuild_include( quickdev_cpp dynamic_reconfigure )"
#quickdev_gencfg()

# add_subdirectory( src )
rosbuild_add_library( xsens_driver src/xsens_driver.cpp src/cmt1.cpp src/cmt2.cpp src/cmt3.cpp src/cmtmessage.cpp src/cmtpacket.cpp src/cmtscan.cpp src/xsens_std.cpp src/xsens_time.cpp )
rosbuild_add_executable( xsens_driver  nodes/xsens_driver_node.cpp )

