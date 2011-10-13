#include <base_libs/macros.h>
#include <base_libs/node.h>
#include <base_libs/publisher_policy.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>

BASE_LIBS_DECLARE_NODE( TestPublisherPolicy, base_libs::PublisherPolicy<> )

BASE_LIBS_DECLARE_NODE_CLASS( TestPublisherPolicy )
{
public:
	TestPublisherPolicyNode( ros::NodeHandle & nh ) : _TestPublisherPolicyNodeAdapterType( nh )
	{
		publishers_.addPublishers<
			std_msgs::String,
			geometry_msgs::Point>(
				nh,
				{ "string", "point" } );
		
		publishers_.addPublishers<
			std_msgs::String,
			geometry_msgs::Point>(
				nh,
				{ "string2", "point2" } );
	}
	
	void spinOnce()
	{
		std_msgs::String string_msg;
		string_msg.data = "hello";
		
		geometry_msgs::Point point_msg;
		point_msg.x = 1;
		point_msg.y = 2;
		point_msg.z = 3;
		
		// it is possible to publish 1 or more key-value pairs with a
		// single call to publish( ... )
		publishers_.publish(
			"string", string_msg,
			"point", point_msg );
		
		string_msg.data = "goodbye";
		
		publishers_.publish( "string2", string_msg );
		
		point_msg.z = 5;
		
		publishers_.publish( "point2", point_msg );
		
		string_msg.data = "whatever";
		point_msg = geometry_msgs::Point();
		
		publishers_.publish(
			"string", string_msg,
			"point", point_msg,
			"string2", string_msg,
			"point2", point_msg );
	}
};

BASE_LIBS_INST_NODE( TestPublisherPolicyNode, "test_publisher_policy_node" )
