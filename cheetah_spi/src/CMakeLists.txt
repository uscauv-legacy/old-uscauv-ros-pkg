add_library_auto( ${PROJECT_NAME} *.cpp *.cc *.c )
target_link_libraries( ${PROJECT_NAME} dl )
