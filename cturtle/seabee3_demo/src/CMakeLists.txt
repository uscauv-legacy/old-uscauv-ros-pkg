#rosbuild_add_executable( square_movement square_movement.cpp )
rosbuild_add_executable( gate_demo gate_demo.cpp )
rosbuild_add_executable( competition_demo competition_demo.cpp )
rosbuild_add_compile_flags(competition_demo -std=c++0x)
