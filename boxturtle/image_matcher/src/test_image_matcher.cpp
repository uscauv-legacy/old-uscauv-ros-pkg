#include <ros/ros.h>
#include <unistd.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include "image_matcher/MatchImage.h"
#include <iostream>

using namespace std;

ros::ServiceClient client;
image_transport::Publisher pub;
sensor_msgs::ImageConstPtr itsCurrentImg;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cout << "callback" << endl;
	itsCurrentImg = msg;
}

void test()
{
	for (int i = 0; i < 10; i++)
	{
		cout << "Testing image matcher" << endl;
		//pub.publish(itsCurrentImg);
		image_matcher::MatchImage srv;
		if (client.call(srv))
		{
			ROS_INFO("Probabilities --- Axe: %f Clippers: %f Hammer: %f Machete: %f", srv.response.axe_probability, srv.response.clippers_probability, srv.response.hammer_probability, srv.response.machete_probability);
		}	
		else
		{
			ROS_ERROR("Service call to MatchImage failed");
		}
		sleep(5);
	}
	else
	{
		cout << "service call failed" << endl;
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_image_matcher");
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);

	// Publish a message
	pub = it.advertise("/test_matcher/image_out", 1);
	
	// Subscribe to an image topic
	image_transport::Subscriber sub = it.subscribe("image", 1, imageCallback);

	// Register the service
	client = n.serviceClient<image_matcher::MatchImage>("MatchImage");
	ROS_INFO("Starting");
	test();
	ros::spin();
}

