rosbuild_add_library( ${PROJECT_NAME} BeeStem3Driver.cpp BeeStem3.cpp )
rosbuild_add_executable( seabee3_node seabee3_node.cpp )
target_link_libraries( seabee3_node ${PROJECT_NAME} )
