rosbuild_add_executable( test_multi_publisher multi_publisher.cpp )
rosbuild_add_compile_flags( test_multi_publisher -std=c++0x )
target_link_libraries( test_multi_publisher ${PROJECT_NAME} )

rosbuild_add_executable( test_node node.cpp )
rosbuild_add_compile_flags( test_node -std=c++0x )
target_link_libraries( test_node ${PROJECT_NAME} )

rosbuild_add_executable( test_publisher_policy publisher_policy.cpp )
rosbuild_add_compile_flags( test_publisher_policy -std=c++0x )
target_link_libraries( test_publisher_policy ${PROJECT_NAME} )

rosbuild_add_executable( test_image_proc_policy image_proc_policy.cpp )
rosbuild_add_compile_flags( test_image_proc_policy -std=c++0x )
target_link_libraries( test_image_proc_policy ${PROJECT_NAME} )

rosbuild_add_executable( test_multi_subscriber multi_subscriber.cpp )
rosbuild_add_compile_flags( test_multi_subscriber -std=c++0x )
target_link_libraries( test_multi_subscriber ${PROJECT_NAME} )

rosbuild_add_executable( test_tf_tranceiver_policy tf_tranceiver_policy.cpp )
rosbuild_add_compile_flags( test_tf_tranceiver_policy -std=c++0x )
target_link_libraries( test_tf_tranceiver_policy ${PROJECT_NAME} )
