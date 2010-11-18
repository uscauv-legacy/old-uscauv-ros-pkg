rosbuild_add_library(${PROJECT_NAME} activation_function.cpp
									 node.cpp
									 artificial_neural_network.cpp
									 ann_parser.cpp)

rosbuild_add_executable(test_ann test_ann.cpp)