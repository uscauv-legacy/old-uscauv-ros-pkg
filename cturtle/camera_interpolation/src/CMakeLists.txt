rosbuild_add_library(${PROJECT_NAME} core.cpp)
rosbuild_add_executable(test-camera_interpolation test-camera_interpolation.cpp)
target_link_libraries(test-camera_interpolation ${PROJECT_NAME})