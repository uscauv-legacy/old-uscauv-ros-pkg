#include <base_libs/macros.h>
#include <base_libs/node.h>
#include <base_libs/publisher_policy.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>

class TestPublisherPolicy : public base_libs::Node<base_libs::PublisherPolicy<> >
{
public:
	TestPublisherPolicy( ros::NodeHandle & nh ) : base_libs::Node<base_libs::PublisherPolicy<> >( nh )
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

BASE_LIBS_DECLARE_NODE( TestPublisherPolicy, "test_publisher_policy" )
