rosbuild_add_library( ${PROJECT_NAME} tritech_micron_driver.cpp )

rosbuild_add_executable( tritech_micron_node tritech_micron_node.cpp )
target_link_libraries( tritech_micron_node ${PROJECT_NAME} )

rosbuild_add_executable( test_tritech_micron_driver test_tritech_micron_driver.cpp )
target_link_libraries( test_tritech_micron_driver ${PROJECT_NAME} )

rosbuild_add_executable( scan_line_converter_node scan_line_converter_node.cpp )
rosbuild_add_compile_flags(${PROJECT_NAME} -std=c++0x)

