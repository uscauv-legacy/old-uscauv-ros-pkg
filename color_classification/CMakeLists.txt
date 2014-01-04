# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(color_classification)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cv_bridge image_transport cpp11 uscauv_common)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

catkin_package(
    DEPENDS Boost OpenCV
    CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport cpp11 uscauv_common
    INCLUDE_DIRS include
    LIBRARIES
)

# Color classifier node and trainer program
add_executable( svm_trainer src/svm_trainer.cpp )
target_link_libraries(svm_trainer ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

add_executable( color_classifier nodes/color_classifier_node.cpp )
target_link_libraries(color_classifier ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})