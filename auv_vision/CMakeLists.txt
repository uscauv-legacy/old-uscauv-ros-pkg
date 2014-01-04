# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(auv_vision)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp uscauv_common dynamic_reconfigure )
find_package(OpenCV REQUIRED)

include_directories(include cfg/cpp ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

generate_dynamic_reconfigure_options(
  cfg/OpticalFlow.cfg
)


#
# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS OpenCV
    CATKIN_DEPENDS roscpp uscauv_common dynamic_reconfigure 
    INCLUDE_DIRS include cfg/cpp
)

add_executable( optical_flow nodes/optical_flow_node.cpp )
target_link_libraries(optical_flow ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )
add_dependencies( optical_flow ${PROJECT_NAME}_gencfg )

# TODO: Figure out how to find leptonica and tesseract so we can build this
# add_executable( bin_differentiator nodes/bin_differentiator_node.cpp )


