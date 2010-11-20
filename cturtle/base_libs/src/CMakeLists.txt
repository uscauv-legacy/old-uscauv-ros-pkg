rosbuild_add_library(${PROJECT_NAME} base_node.cpp
									 base_image_proc_core.cpp
									 base_image_proc.cpp
									 base_tf_tranceiver.cpp
									 base_robot_driver.cpp
									 base_teleop.cpp
									 mathy_math.cpp
									 timeout_monitor.cpp
									 colors.cpp)