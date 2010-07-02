// Use the image_transport classes instead.
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>
#include <opencv/cv.h>
#include <cv_bridge/CvBridge.h>
#include "cvutility.h"

using namespace cv;

ros::Publisher *pos_pub; 
image_transport::Publisher *dbg_img_pub;

sensor_msgs::CvBridge bridge;

int show_dbg_img;
int target_hue;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	ROS_INFO("GOT AN IMAGE!");

	Mat img(bridge.imgMsgToCv(msg));
	IplImage iplImg = img;
	
	Mat redImg = cvFilterHS(&iplImg,40,220,0,255,0);
	dilate(redImg, redImg, Mat(), Point(-1,-1), 3);
	erode(redImg, redImg, Mat(), Point(-1,-1), 3);

	//Mat redImg;
	
	//Run a circle hough transform on the image to detect circular outlines
	vector<Vec3f> circles;
	HoughCircles(redImg, circles, CV_HOUGH_GRADIENT,
			2, redImg.rows/4, 200, 100 );

	//Show the debug image if requested
	if(show_dbg_img != 0)
	{
		Mat dbgImg = redImg;
		//Print out an annoying message so that we remember to disable the debug image
		ROS_INFO("Showing Debug Image - got %lu circles", circles.size());
		for( size_t i = 0; i < circles.size(); i++ )
		{
			Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			int radius = cvRound(circles[i][2]);
			// draw the circle center
			circle( dbgImg, center, 3, Scalar(0,255,0), -1, 8, 0 );
			// draw the circle outline
			circle( dbgImg, center, radius, Scalar(0,0,255), 3, 8, 0 );
		}

		//Convert our debug image to an IplImage and send it out
		IplImage iplImg = dbgImg;
		dbg_img_pub->publish(bridge.cvToImgMsg(&iplImg));
	}

	geometry_msgs::Vector3 buoy_pos;
	buoy_pos.x = -1;
	buoy_pos.y = -1;
	buoy_pos.z = -1;
	pos_pub->publish(buoy_pos);
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "buoy_finder");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	ROS_INFO("STARTING UP");

	//Register the show debug image parameter
	nh.param("show_dbg_img", show_dbg_img, 1);
	nh.param("target_hue", target_hue, 60);

	//Register a publisher for the 3D position of the buoy
	pos_pub = new ros::Publisher(nh.advertise<geometry_msgs::Vector3>("perception/buoy_pos", 1));

	//Register a publisher for a debug image
	dbg_img_pub = new image_transport::Publisher(it.advertise("perception/buoy_debug_image", 1));

	//Register a subscriber to an input image
	image_transport::Subscriber sub = it.subscribe("image", 1, imageCallback);

	ROS_INFO("SPINNNING UP");
	ros::spin();

	return 0;
}
