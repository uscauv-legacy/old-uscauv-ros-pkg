rosbuild_add_library(${PROJECT_NAME} base_node.cpp
									 base_image_proc_core.cpp
									 base_image_proc.cpp
									 base_stereo_image_proc_core.cpp
									 base_stereo_image_proc.cpp
									 base_tf_tranceiver.cpp
									 base_robot_driver.cpp
									 base_teleop.cpp
									 timeout_monitor.cpp
									 colors.cpp
									 image_loader.cpp)
									 
rosbuild_add_executable(test_base_image_proc test_base_image_proc.cpp)