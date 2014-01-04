# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(uscauv_joystick)

find_package(catkin REQUIRED COMPONENTS sensor_msgs uscauv_common roscpp)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS sensor_msgs uscauv_common roscpp
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

add_library( ${PROJECT_NAME} src/joystick_policy.cpp )
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable( joystick_client nodes/joystick_client_node.cpp )
target_link_libraries( joystick_client ${PROJECT_NAME} ${catkin_LIBRARIES})