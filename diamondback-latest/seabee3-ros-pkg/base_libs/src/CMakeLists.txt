rosbuild_add_library( ${PROJECT_NAME} console.cpp
									  macros.cpp
									  types.cpp
									  type_utils.cpp
									  container.cpp
									  multityped_linked_list.cpp
									  auto_bind.cpp
									  generic_policy_adapter.cpp
									  param_reader.cpp
									  image_loader.cpp
									  multi_subscriber.cpp
									  multi_publisher.cpp
									  node.cpp
									  nodelet.cpp
									  policy.cpp
									  node_handle_policy.cpp
									  runable_policy.cpp
									  updateable_policy.cpp
									  timed_policy.cpp
									  reconfigure_policy.cpp
									  service_server_policy.cpp
									  service_client_policy.cpp
									  publisher_policy.cpp
									  subscriber_policy.cpp
									  image_proc_policy.cpp
									  tf_tranceiver_policy.cpp
									  joystick_policy.cpp
									  robot_controller_policy.cpp )
									 
#rosbuild_add_executable( test_base_image_proc test_base_image_proc.cpp )
target_link_libraries( ${PROJECT_NAME} pthread boost_thread )
rosbuild_add_compile_flags( ${PROJECT_NAME} -std=c++0x )

#add_subdirectory( nodelets )
