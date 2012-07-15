# gather all sources in current dir using relative filenames
file( GLOB ALL_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp *.cc *.c )

foreach( source ${ALL_SOURCES} )
    get_filename_component( source_name_base ${source} NAME_WE )
    set( source_name ${source_name_base} )
    set( source_src ${source} )

    rosbuild_add_executable( ${source_name} ${source_src} )

endforeach( source )
