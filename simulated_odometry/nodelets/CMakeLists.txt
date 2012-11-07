# gather all sources in current dir using relative filenames
file( GLOB NODELET_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp *.cc *.c )
rosbuild_add_library( ${PROJECT_NAME}_nodelets ${NODELET_SOURCES} )
