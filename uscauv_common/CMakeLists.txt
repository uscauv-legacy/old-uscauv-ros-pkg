# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(uscauv_common)

# Load catkin and all dependencies required for this package
# Only catkin packages may be called as components (system dependencies must be found separately)
find_package(catkin REQUIRED COMPONENTS roscpp tf cpp11 sensor_msgs cv_bridge image_transport image_geometry auv_msgs dynamic_reconfigure uscauv_build)

find_package(OpenCV REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

# Generate Config
generate_dynamic_reconfigure_options(
  cfg/RANSAC.cfg
  )


# CATKIN_MIGRATION: removed during catkin migration
# Generates uscauv_common.h header including everything else in this package
# rosbuild_include(uscauv_build generate_common_header)
generate_common_header(${PROJECT_SOURCE_DIR}/include uscauv_common uscauv_common.h)

# # Include auto-generated dynamic reconfigure headers
include_directories(${PROJECT_SOURCE_DIR}/cfg/cpp)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    DEPENDS OpenCV
    CATKIN_DEPENDS roscpp tf cpp11 sensor_msgs cv_bridge image_transport image_geometry auv_msgs 
    INCLUDE_DIRS include cfg/cpp
    LIBRARIES ${PROJECT_NAME}
)

add_library( ${PROJECT_NAME} src/base_node.cpp src/image_transceiver.cpp src/multi_reconfigure.cpp src/graphics.cpp src/image_loader.cpp src/timing.cpp src/pose_integrator.cpp src/simple_math.cpp src/param_loader.cpp src/image_geometry.cpp src/tic_toc.cpp src/defaults.cpp src/color_codec.cpp src/action_token.cpp src/lookup_table.cpp src/transform_utils.cpp src/serial.cpp src/macros.cpp src/param_writer.cpp src/param_loader_conversions.cpp )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)