macro( add_library_auto lib )
	# gather all sources in current dir using relative filenames
	file( GLOB LIB_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${ARGN} )
	rosbuild_add_library( ${lib} ${LIB_SOURCES} )
endmacro( add_library_auto )
