# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(seabee3_common)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS seabee3_msgs seabee3_actions visualization_msgs image_geometry cpp11)

include_directories(include ${catkin_INCLUDE_DIRS})

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS seabee3_msgs seabee3_actions visualization_msgs image_geometry cpp11
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME} src/colors.cpp src/movement.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})