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

rosbuild_genmsg()
rosbuild_gensrv()

#link_directories("${PROJECT_SOURCE_DIR}/../libseabee3_beestem/lib")
rosbuild_add_executable(seabee3_driver_base src/seabee3_driver_base.cpp)
#if (UNIX AND NOT WIN32)
#  if (CMAKE_SIZEOF_VOID_P MATCHES "8")
#    target_link_libraries(seabee3_driver_base seabee3_beestem_x64)
#  else (CMAKE_SIZEOF_VOID_P MATCHES "8")
#	target_link_libraries(seabee3_driver_base seabee3_beestem_i386)
#  endif (CMAKE_SIZEOF_VOID_P MATCHES "8")
#endif (UNIX AND NOT WIN32)
