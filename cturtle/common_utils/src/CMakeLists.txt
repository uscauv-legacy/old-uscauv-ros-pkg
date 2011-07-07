rosbuild_add_library (${PROJECT_NAME} core.cpp serial.cpp )
rosbuild_add_compile_flags(${PROJECT_NAME} -std=c++0x)
