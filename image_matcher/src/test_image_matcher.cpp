#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include "image_matcher/MatchImage.h"

ros::ServiceClient client;
sensor_msgs::ImageConstPtr itsImage1;
sensor_msgs::ImageConstPtr itsImage2;
bool isSetImg2;

void imageCallback1(const sensor_msgs::ImageConstPtr& msg)
{
	itsImage1 = msg;
	if (!isSetImg2)	return;
	image_matcher::MatchImage srv;
	srv.request.object = *itsImage1;
	srv.request.image = *itsImage2;
	if (client.call(srv))
	{
		ROS_INFO("ImageMatch probability: %4.2f", srv.response.probability);
	}
}


void imageCallback2(const sensor_msgs::ImageConstPtr& msg)
{
	itsImage2 = msg;
	isSetImg2 = true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_image_matcher");
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	isSetImg2 = false;
	
	// Register publisher for image with bins highlighted
	image_transport::Publisher testPub = it.advertise("image_match", 1);

	// Subscribe to an image topic
	image_transport::Subscriber sub1 = it.subscribe("image1", 1, imageCallback1);
	image_transport::Subscriber sub2 = it.subscribe("image2", 1, imageCallback2);

	// Register the service
	client = n.serviceClient<image_matcher::MatchImage>("image_match");
	ros::spin();
}

