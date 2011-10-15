#ifndef BASE_LIBS_BASE_LIBS_CONSOLE_H_
#define BASE_LIBS_BASE_LIBS_CONSOLE_H_

#include <ros/console.h>

#define PRINT_INFO( args... ) ROS_INFO( args )
#define PRINT_WARN( args... ) ROS_WARN( args )
#define PRINT_DEBUG( args... ) ROS_DEBUG( args )

#endif // BASE_LIBS_BASE_LIBS_CONSOLE_H_
