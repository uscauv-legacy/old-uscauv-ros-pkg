// Use the image_transport classes instead.
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>
#include <opencv/cv.h>
#include <cv_bridge/CvBridge.h>

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
	Mat hsvImg;
	Mat hImg;
	Mat redImg;

	cvtColor(img, redImg, CV_BGR2GRAY);
	////Convert image to hsv
	//cvtColor(img, hsvImg, CV_BGR2HSV);

	////Extract the hue channel
	//vector<Mat> hsvChannels;
	//split(hsvImg, hsvChannels);
	//hImg = hsvChannels[0];

	////FIXXXXX ME!!!
	//// 255 - abs(hImg-target_hue)
	//subtract(hImg, Scalar(target_hue), hImg);
	//hImg = abs(hImg);
	//subtract(Scalar(255), hImg, redImg);

	////Threshold the image for red
	////	adaptiveThreshold(hImg, redImg, 

	// Smooth the image to reduce false detections from speckles
	GaussianBlur( redImg, redImg, Size(9, 9), 2, 2 );

	//Run a circle hough transform on the image to detect circular outlines
	vector<Vec3f> circles;
	HoughCircles(redImg, circles, CV_HOUGH_GRADIENT,
			2, redImg.rows/4, 200, 100 );


	//Show the debug image if requested
	if(show_dbg_img != 0)
	{
		Mat dbgImg = redImg;
		//Print out an annoying message so that we remember to disable the debug image
		ROS_INFO("Showing Debug Image");
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
	nh.param("target_hue", target_hue, 128);

	//Register a publisher for the 3D position of the buoy
	pos_pub = new ros::Publisher(nh.advertise<geometry_msgs::Vector3>("perception/buoy_pos", 1));

	//Register a publisher for a debug image
	dbg_img_pub = new image_transport::Publisher(it.advertise("perception/buoy_debug_image", 1));

	//Register a subscriber to an input image
	image_transport::Subscriber sub = it.subscribe("image", 1, imageCallback);

	ros::spin();

	return 0;
}
