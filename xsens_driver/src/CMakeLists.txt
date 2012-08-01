rosbuild_add_library( ${PROJECT_NAME} xsens_driver.cpp cmt1.cpp cmt2.cpp cmt3.cpp cmtmessage.cpp cmtpacket.cpp cmtscan.cpp xsens_std.cpp xsens_time.cpp )
#rosbuild_add_executable( xsens_node xsens_node.cpp )
#target_link_libraries( xsens_node ${PROJECT_NAME} )
