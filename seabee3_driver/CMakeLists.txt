# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(seabee3_driver)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS quickdev_cpp nodelet seabee3_msgs seabee3_common uscauv_common)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

# CATKIN_MIGRATION
# use include(module) after finding the path
# rosbuild_include( quickdev_build add_library_auto )

# CATKIN_MIGRATION
# use include(module) after finding the path
# rosbuild_include( quickdev_build dynamic_reconfigure )

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# uncomment if you have defined messages
#add_message_files(
  FILES
  # TODO: List your msg files here
)
# uncomment if you have defined services
#add_service_files(
  FILES
  # TODO: List your msg files here
)
# uncomment if you have defined dynamic reconfigure files
# Note: requires "
# CATKIN_MIGRATION
# use include(module) after finding the path
# rosbuild_include( quickdev_build dynamic_reconfigure )"
quickdev_gencfg()

add_subdirectory( src )
add_subdirectory( nodes )
add_subdirectory( nodelets )
## Generate added messages and services with any dependencies listed here
generate_messages(
    #TODO DEPENDENCIES geometry_msgs std_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS quickdev_cpp nodelet seabee3_msgs seabee3_common uscauv_common
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)