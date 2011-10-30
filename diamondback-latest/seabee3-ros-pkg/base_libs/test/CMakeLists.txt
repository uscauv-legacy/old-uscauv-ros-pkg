include(${base_libs_PACKAGE_PATH}/compile_tests.cmake)

add_compile_test( test_multi_publisher multi_publisher.cpp )
rosbuild_add_compile_flags( test_multi_publisher -std=c++0x )
target_link_libraries( test_multi_publisher ${PROJECT_NAME} )

add_compile_test( test_node node.cpp )
rosbuild_add_compile_flags( test_node -std=c++0x )
target_link_libraries( test_node ${PROJECT_NAME} )

add_compile_test( test_publisher_policy publisher_policy.cpp )
rosbuild_add_compile_flags( test_publisher_policy -std=c++0x )
target_link_libraries( test_publisher_policy ${PROJECT_NAME} )

add_compile_test( test_image_proc_policy image_proc_policy.cpp )
rosbuild_add_compile_flags( test_image_proc_policy -std=c++0x )
target_link_libraries( test_image_proc_policy ${PROJECT_NAME} )

add_compile_test( test_multi_subscriber multi_subscriber.cpp )
rosbuild_add_compile_flags( test_multi_subscriber -std=c++0x )
target_link_libraries( test_multi_subscriber ${PROJECT_NAME} )

add_compile_test( test_tf_tranceiver_policy tf_tranceiver_policy.cpp )
rosbuild_add_compile_flags( test_tf_tranceiver_policy -std=c++0x )
target_link_libraries( test_tf_tranceiver_policy ${PROJECT_NAME} )

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

add_compile_test( test_reconfigure_policy reconfigure_policy.cpp )
rosbuild_add_compile_flags( test_reconfigure_policy -std=c++0x )
target_link_libraries( test_reconfigure_policy ${PROJECT_NAME} )

add_compile_test( test_type_utils type_utils.cpp )
rosbuild_add_compile_flags( test_type_utils -std=c++0x )
target_link_libraries( test_type_utils ${PROJECT_NAME} )

add_compile_test( test_robot_controller_policy robot_controller_policy.cpp )
rosbuild_add_compile_flags( test_robot_controller_policy -std=c++0x )
target_link_libraries( test_robot_controller_policy ${PROJECT_NAME} )

add_compile_test( test_robot_driver_policy robot_driver_policy.cpp )
rosbuild_add_compile_flags( test_robot_driver_policy -std=c++0x )
target_link_libraries( test_robot_driver_policy ${PROJECT_NAME} )

add_compile_test( test_joystick_policy joystick_policy.cpp )
rosbuild_add_compile_flags( test_joystick_policy -std=c++0x )
target_link_libraries( test_joystick_policy ${PROJECT_NAME} )

rosbuild_gensrv()

add_compile_test( test_service_server_policy service_server_policy.cpp )
rosbuild_add_compile_flags( test_service_server_policy -std=c++0x )
target_link_libraries( test_service_server_policy ${PROJECT_NAME} )

add_compile_test( test_service_client_policy service_client_policy.cpp )
rosbuild_add_compile_flags( test_service_client_policy -std=c++0x )
target_link_libraries( test_service_client_policy ${PROJECT_NAME} )
