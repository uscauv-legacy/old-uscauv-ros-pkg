#include <base_libs/macros.h>
#include <base_libs/multi_publisher.h>
#include <ros/rate.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>

class TestMultiPublisherNode
{
public:
	ros::MultiPublisher<> multi_pub_;
	ros::Rate loop_rate_;
	
	TestMultiPublisherNode( ros::NodeHandle & nh )
	:
		multi_pub_(),
		loop_rate_( 10 )
	{
		multi_pub_.addPublishers<
			std_msgs::String,
			geometry_msgs::Point>(
				nh,
				{ "string", "point" } );
		
		multi_pub_.addPublishers<
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
		multi_pub_.publish(
			"string", string_msg,
			"point", point_msg );
		
		string_msg.data = "goodbye";
		
		multi_pub_.publish( "string2", string_msg );
		
		point_msg.z = 5;
		
		multi_pub_.publish( "point2", point_msg );
		
		string_msg.data = "whatever";
		point_msg = geometry_msgs::Point();
		
		multi_pub_.publish(
			"string", string_msg,
			"point", point_msg,
			"string2", string_msg,
			"point2", point_msg );
	}
	
	void spin()
	{
		while( ros::ok() )
		{
			spinOnce();
			ros::spinOnce();
			loop_rate_.sleep();
		}
	}
};

BASE_LIBS_INST_NODE( TestMultiPublisherNode, "test_multi_publisher_node" )
