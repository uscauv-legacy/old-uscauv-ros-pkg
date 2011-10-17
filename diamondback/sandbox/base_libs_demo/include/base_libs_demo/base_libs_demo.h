
// <PACKAGE_NAME>_<_INCLUDE_SUBDIR>_<FILE_NAME>_H_
#ifndef BASE_LIBS_DEMO_BASE_LIBS_DEMO_BASE_LIBS_DEMO_H_
#define BASE_LIBS_DEMO_BASE_LIBS_DEMO_BASE_LIBS_DEMO_H_

#include <base_libs/node.h>

BASE_LIBS_DECLARE_NODE( BaseLibsDemo )

BASE_LIBS_DECLARE_NODE_CLASS( BaseLibsDemo )
{
	BASE_LIBS_DECLARE_NODE_CONSTRUCTOR( BaseLibsDemo )
	{
		//
	}
	
	void spinFirst()
	{
		initAll();
	}
	
	void spinOnce()
	{
		PRINT_INFO( "Spinning!" );
	}
};

#endif // BASE_LIBS_DEMO_BASE_LIBS_DEMO_BASE_LIBS_DEMO_H_
