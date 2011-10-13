#include <base_libs/macros.h>
#include <base_libs/node.h>

class TestNode : public base_libs::Node<>
{
public:
	TestNode( ros::NodeHandle & nh ) : base_libs::Node<>( nh )
	{
		
	}
	
	void spinOnce()
	{
		ROS_INFO( "Spinning!" );
	}
};

BASE_LIBS_DECLARE_NODE( TestNode, "test_node" )
