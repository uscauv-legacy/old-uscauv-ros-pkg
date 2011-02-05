rosbuild_add_library(${PROJECT_NAME} landmark_projector.cpp)

rosbuild_add_executable(landmark_projector_tester landmark_projector_tester.cpp)

target_link_libraries(landmark_projector_tester ${PROJECT_NAME})