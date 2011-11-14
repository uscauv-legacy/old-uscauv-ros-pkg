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
rosbuild_include( quickdev_cpp dynamic_reconfigure )

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Auto-generated files for this package are under version control.
# To make changes to these files, uncomment the following section,
# build, then re-comment and commit the changes. Depending on the
# state of this package, it may also be necessary to temporarily
# rename the 'ROS_NOBUILD' file before the package will build.
#
rosbuild_genmsg()
rosbuild_gensrv()
quickdev_gencfg()

add_subdirectory( nodes )
