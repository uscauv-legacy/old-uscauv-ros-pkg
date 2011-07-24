rosbuild_add_library( ${PROJECT_NAME} contour.cpp )
rosbuild_add_executable(color_blob_finder_node color_blob_finder_node.cpp)
add_subdirectory( nodelets )
