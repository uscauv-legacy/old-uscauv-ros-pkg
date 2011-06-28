rosbuild_add_executable( color_classifier_node color_classifier_node.cpp )
rosbuild_add_executable( trainer trainer.cpp )
add_subdirectory( nodelets )

#rosbuild_add_executable(thread_tester thread_tester.cpp)
#
#include(FindPkgConfig)
#if (PKG_CONFIG_FOUND AND UNIX AND NOT APPLE)
#   pkg_check_modules(GTK gtk+-2.0)
#   if (GTK_FOUND)
#      include_directories(${GTK_INCLUDE_DIRS})
#      link_directories(${GTK_LIBRARY_DIRS})
#      add_definitions(-DHAVE_GTK)
#   endif(GTK_FOUND)
#endif(PKG_CONFIG_FOUND AND UNIX AND NOT APPLE)
#
#rosbuild_add_executable(trainer trainer.cpp)
#target_link_libraries(trainer ${GTK_LIBRARIES})