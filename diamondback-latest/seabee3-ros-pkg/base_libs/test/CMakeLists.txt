include(${base_libs_PACKAGE_PATH}/compile_tests.cmake)

# gather all sources in current dir using relative filenames
file( GLOB TEST_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp *.cc *.c )

set( TEST_COMPILE_FLAGS -std=c++0x )

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

rosbuild_gensrv()

foreach( test ${TEST_SOURCES} )
	# TEST_SOURCES = foo.cpp;bar.cpp
	# test = foo.cpp
	# test_name_base = foo
	# test_src = foo.cpp
	# test_name = test_foo
	get_filename_component( test_name_base ${test} NAME_WE )
	set( test_name test_${test_name_base} )
	set( test_src ${test} )
	
	# add_compile_test( test_foo foo.cpp )
	add_compile_test( ${test_name} ${test_src} )
	rosbuild_add_compile_flags( ${test_name} ${TEST_COMPILE_FLAGS} )
	target_link_libraries( ${test_name} ${PROJECT_NAME} )
endforeach( test )
