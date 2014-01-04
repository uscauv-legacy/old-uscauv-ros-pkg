# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(image_segmentation)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS uscauv_common dynamic_reconfigure)

find_package(OpenCV REQUIRED)

include_directories(include cfg/cpp ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

generate_dynamic_reconfigure_options(
  cfg/GraphBasedSegmentation.cfg
)

catkin_package(
    DEPENDS OpenCV
    CATKIN_DEPENDS uscauv_common  dynamic_reconfigure
    INCLUDE_DIRS include cfg/cpp
    LIBRARIES ${PROJECT_NAME}
)

add_library( ${PROJECT_NAME} src/graph_based_segmentation.cpp )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)


add_executable( ${PROJECT_NAME}_node  nodes/image_segmentation_node.cpp )
# We can't have a library and executable with the same name in CMake, but this forces the name of the generated
# executable to be the same
set_target_properties( ${PROJECT_NAME}_node
  PROPERTIES OUTPUT_NAME ${PROJECT_NAME} )

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})