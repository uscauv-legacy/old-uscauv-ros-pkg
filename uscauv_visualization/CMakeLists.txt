cmake_minimum_required(VERSION 2.8.3)
project(uscauv_visualization)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS uscauv_common visualization_msgs tf)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS  uscauv_common visualization_msgs tf
)

add_executable( object_visualizer nodes/object_visualizer_node.cpp )
target_link_libraries( object_visualizer ${catkin_LIBRARIES})