add_custom_target( compile_tests )

# exclude exe from all but make compile_tests depend on it so it only gets build with make compile_tests
macro( add_compile_test exe )
  rosbuild_add_executable( ${exe};EXCLUDE_FROM_ALL;${ARGN} )
  add_dependencies( compile_tests ${exe} )
endmacro( add_compile_test )
