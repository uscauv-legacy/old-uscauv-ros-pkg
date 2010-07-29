// Use the image_transport classes instead.
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include "cv.h"
#include "highgui.h"
#include <cv_bridge/CvBridge.h>
#include <iostream>
#include <iomanip>
#include <sstream>

using namespace cv;

image_transport::Publisher *img_pub;

sensor_msgs::CvBridge bridge;

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "image_stream");
	ros::NodeHandle nh("~");
	image_transport::ImageTransport it(nh);

	std::string file_prefix;
	std::string file_ext;
	int start;
	int end;
	int digits;
	int rate;
	bool loop;

	nh.param("prefix", file_prefix, std::string(""));
	nh.param("start",  start,       0);
	nh.param("end",    end,         0);
	nh.param("digits", digits,      0);
	nh.param("ext",    file_ext,    std::string(""));
	nh.param("rate",   rate,        15);
	nh.param("loop",   loop,        false);

	//Register a publisher for an image
	img_pub = new image_transport::Publisher(it.advertise("image_stream/image_color", 1));

	int curr_frame = start;
	while(ros::ok() && curr_frame <= end)
	{
		std::stringstream filename;
		filename << file_prefix << std::setfill('0') << std::setw(digits) << curr_frame << file_ext;
		ROS_INFO("Opening %s",filename.str().c_str());

		Mat img = imread(filename.str().c_str());
		
		//Memory leak? Probably.
		IplImage iplImg = img;
		img_pub->publish(bridge.cvToImgMsg(&iplImg));

		ros::spinOnce();
		ros::Rate(rate).sleep();
		curr_frame++;

		if(curr_frame > end && loop)
		  curr_frame = start;
	}

	return 0;
}

