#include <base_libs/macros.h>
#include <base_libs/node.h>

BASE_LIBS_DECLARE_NODE( Test )

BASE_LIBS_DECLARE_NODE_CLASS( Test )
{
	BASE_LIBS_DECLARE_NODE_CONSTRUCTOR( Test )
	{
		
	}
	
	void spinOnce()
	{
		ROS_INFO( "Spinning!" );
	}
};

BASE_LIBS_INST_NODE( TestNode, "test_node" )
