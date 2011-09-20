#ifndef BASE_LIBS_BASE_LIBS_MACROS_H_
#define BASE_LIBS_BASE_LIBS_MACROS_H_

#include <ros/ros.h>

// use: BASE_LIBS_DECLARE_NODE( SomeNode, "some_node" )
#define BASE_LIBS_DECLARE_NODE( __Class, node_name_string ) \
int main( int argc, char ** argv ) \
{ \
	ros::init( argc, argv, node_name_string ); \
	ros::NodeHandle nh( "~" ); \
	\
	__Class class_inst( nh ); \
	class_inst.spin(); \
	return 0; \
} \

#define IMAGE_PROC_PROCESS_IMAGE( image_ptr_name ) \
void processImage( cv_bridge::CvImagePtr & image_ptr_name ) \

#endif // BASE_LIBS_BASE_LIBS_MACROS_H_
