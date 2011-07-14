rosbuild_add_executable(sparse_stereo_flow sparse_stereo_flow.cpp)
rosbuild_add_compile_flags(sparse_stereo_flow "-g")

rosbuild_add_executable(point_cloud_tester point_cloud_tester.cpp)
rosbuild_add_compile_flags(point_cloud_tester "-g")

#rosbuild_add_executable(sparse_stereo sparse_stereo.cpp)
#rosbuild_add_compile_flags(sparse_stereo "-g")

#rosbuild_add_executable(optical_flow_tester optical_flow_tester.cpp)
#rosbuild_add_compile_flags(optical_flow_tester "-g")