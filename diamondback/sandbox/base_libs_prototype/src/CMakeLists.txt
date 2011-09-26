rosbuild_add_library( ${PROJECT_NAME} macros.cpp
									  types.cpp
									  container.cpp
									  auto_bind.cpp
									  generic_policy_adapter.cpp
									  param_reader.cpp
									  multi_subscriber.cpp
									  multi_publisher.cpp
									  node.cpp
									  nodelet.cpp
									  policy.cpp
									  reconfigure_policy.cpp
									  service_server_policy.cpp
									  publisher_policy.cpp
									  subscriber_policy.cpp
									  image_proc_policy.cpp
									  tf_tranceiver_policy.cpp
									  joystick_policy.cpp
									  robot_driver_policy.cpp )
									 
#rosbuild_add_executable( test_base_image_proc test_base_image_proc.cpp )
rosbuild_add_compile_flags( ${PROJECT_NAME} -std=c++0x )

#add_subdirectory( nodelets )
