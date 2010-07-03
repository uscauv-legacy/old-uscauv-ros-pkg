// Use the image_transport classes instead.
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>
#include <opencv/cv.h>
#include <cv_bridge/CvBridge.h>
#include "cvutility.h"
#include <dynamic_reconfigure/server.h>
#include <buoy_finder/BuoyFinderConfig.h>

using namespace cv;

ros::Publisher *pos_pub; 
image_transport::Publisher *dbg_img_pub;

sensor_msgs::CvBridge bridge;

#define BUOY_H_MIN 75
#define BUOY_H_MAX 255
#define BUOY_S_MIN 0
#define BUOY_S_MAX 255

int show_dbg_img;
int h_min, h_max,s_min,s_max,mode;

void reconfigureCallback(buoy_finder::BuoyFinderConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request : %d %d %d %d %d",
           config.h_min, config.h_max, config.s_min, config.s_max, config.mode);  

  

  h_min = config.h_min;
  h_max = config.h_max;
  s_min = config.s_min;
  s_max = config.s_max;
  //mode = config.mode;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO("Received image.");

  Mat img(bridge.imgMsgToCv(msg));
  IplImage iplImg = img;
	
  Mat redImg = cvFilterHS(&iplImg,h_min,h_max,s_min,s_max,mode);

  dilate(redImg, redImg, Mat(), Point(-1,-1), 3);
  erode(redImg, redImg, Mat(), Point(-1,-1), 3);

  //Mat redImg;
	
  //Run a circle hough transform on the image to detect circular outlines
  /*  vector<Vec3f> circles;
  HoughCircles(redImg, circles, CV_HOUGH_GRADIENT,
	       2, redImg.rows/4, 200, 100 );
  */

  //Show the debug image if requested
  if(show_dbg_img != 0)
    {
      Mat dbgImg = redImg;
      //Print out an annoying message so that we remember to disable the debug image
      //ROS_INFO("Showing Debug Image - got %lu circles", circles.size());
 //      for( size_t i = 0; i < circles.size(); i++ )
// 	{
// 	  Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
// 	  int radius = cvRound(circles[i][2]);
// 	  // draw the circle center
// 	  circle( dbgImg, center, 3, Scalar(0,255,0), -1, 8, 0 );
// 	  // draw the circle outline
// 	  circle( dbgImg, center, radius, Scalar(0,0,255), 3, 8, 0 );
// 	}

      //Convert our debug image to an IplImage and send it out
      IplImage iplImg = dbgImg;
      dbg_img_pub->publish(bridge.cvToImgMsg(&iplImg));
    }

  /* geometry_msgs::Vector3 buoy_pos;
  buoy_pos.x = -1;
  buoy_pos.y = -1;
  buoy_pos.z = -1;
  pos_pub->publish(buoy_pos);*/
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "buoy_finder");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  //Register the show debug image parameter
  nh.param("show_dbg_img", show_dbg_img, 1);

  //Register a publisher for the 3D position of the buoy
  pos_pub = new ros::Publisher(nh.advertise<geometry_msgs::Vector3>("perception/buoy_pos", 1));

  //Register a publisher for a debug image
  dbg_img_pub = new image_transport::Publisher(it.advertise("perception/buoy_debug_image", 1));

  //Register a subscriber to an input image
  image_transport::Subscriber sub = it.subscribe("image", 1, imageCallback);

  dynamic_reconfigure::Server<buoy_finder::BuoyFinderConfig> srv;
  dynamic_reconfigure::Server<buoy_finder::BuoyFinderConfig>::CallbackType f = boost::bind(&reconfigureCallback, _1, _2);
  srv.setCallback(f);

  h_min = BUOY_H_MIN;
  h_max = BUOY_H_MAX;
  s_min = BUOY_S_MIN;
  s_max = BUOY_S_MAX;
  mode = 0;

  ros::spin();

  return 0;
}
