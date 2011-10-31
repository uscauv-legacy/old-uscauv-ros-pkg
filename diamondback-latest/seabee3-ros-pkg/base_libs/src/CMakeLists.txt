# gather all sources in current dir using relative filenames
file( GLOB BASE_LIBS_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp *.cc *.c )
rosbuild_add_library( ${PROJECT_NAME} ${BASE_LIBS_SOURCES} )

target_link_libraries( ${PROJECT_NAME} pthread boost_thread )
rosbuild_add_compile_flags( ${PROJECT_NAME} -std=c++0x )
