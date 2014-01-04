# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(uscauv_utilities)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cv_bridge image_transport)

find_package(OpenCV REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

catkin_package(
  DEPENDS OpenCV
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport
  INCLUDE_DIRS include
  LIBRARIES # TODO
)

add_executable( static_image_publisher nodes/static_image_publisher_node.cpp )
target_link_libraries( static_image_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Copies this script into devel space so that it is in our PYTHONPATH
install(PROGRAMS bin/hist_view
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)