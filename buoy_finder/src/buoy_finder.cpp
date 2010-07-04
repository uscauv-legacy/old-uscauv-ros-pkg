// Use the image_transport classes instead.
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>
#include <opencv/cv.h>
#include <cv_bridge/CvBridge.h>
#include <dynamic_reconfigure/server.h>
#include <buoy_finder/BuoyFinderConfig.h>
#include <buoy_finder/ColorBlobArray.h>
#include "cvutility.h"
#include "cvBlob/Blob.h"
#include "cvBlob/BlobResult.h"

using namespace cv;

ros::Publisher *buoy_pub; 
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
  Mat img(bridge.imgMsgToCv(msg));
  IplImage iplImg = img;
	
  Mat redImg = cvFilterHS(&iplImg,h_min,h_max,s_min,s_max,mode);

  
  dilate(redImg, redImg, Mat(), Point(-1,-1), 10);
  erode(redImg, redImg, Mat(), Point(-1,-1), 3);


  IplImage blobImg = redImg;
  cvFlipBinaryImg(&blobImg);

  CBlobResult blobs;
  blobs = CBlobResult(&blobImg, NULL, 0, false);

  blobs.Filter(blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, 5);
  blobs.Filter(blobs, B_EXCLUDE, CBlobGetArea(), B_GREATER, 10000);


  //Show the debug image if requested
  if(show_dbg_img != 0)
    {
      Mat dbgImg;
      cvtColor(redImg, dbgImg, CV_GRAY2RGB);

      if(blobs.GetNumBlobs() > 0)
	{

	  CBlob biggestBlob;
	  blobs.GetNthBlob(CBlobGetArea(), 0, biggestBlob);
	  
	  float bArea = biggestBlob.Area();
	  float bX = (biggestBlob.MinX() + biggestBlob.MaxX()) / 2.0;
	  float bY = (biggestBlob.MinY() + biggestBlob.MaxY()) / 2.0;

	  //Print out an annoying message so that we remember to disable the debug image
	  ROS_INFO("Showing Debug Image - max blob at %f,%f area %f (%d total)", bX, bY, bArea, blobs.GetNumBlobs());

	  Point center(bX, bY);
	  rectangle( dbgImg, Point(biggestBlob.MinX(),biggestBlob.MinY()), Point(biggestBlob.MaxX(),biggestBlob.MaxY()), Scalar(0,255,0));
 	  // draw the circle center
	  circle( dbgImg, center, 3, Scalar(0,255,0), -1, 8, 0 );
	  // draw the circle outline
	  circle( dbgImg, center, 5, Scalar(0,0,255), 3, 8, 0 );
	}

      //Convert our debug image to an IplImage and send it out
      IplImage iplImg = dbgImg;
      dbg_img_pub->publish(bridge.cvToImgMsg(&iplImg));
    }

  buoy_finder::ColorBlobArray msgBlobs;

  CBlob currentBlob;

  for(int i = 0; i < blobs.GetNumBlobs(); i++)
    {
      blobs.GetNthBlob(CBlobGetArea(), i, currentBlob);
      
      float blobArea = currentBlob.Area();
      float blobX = (currentBlob.MinX() + currentBlob.MaxX()) / 2.0;
      float blobY = (currentBlob.MinY() + currentBlob.MaxY()) / 2.0;

      buoy_finder::ColorBlob msgBlob;
      msgBlob.X = blobX;
      msgBlob.Y = blobY;
      msgBlob.Area = blobArea;

      
      msgBlobs.push_back(msgBlob);
    }

  buoy_pub->publish(msgBlobs);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "buoy_finder");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  //Register the show debug image parameter
  nh.param("show_dbg_img", show_dbg_img, 1);

  //Register a publisher for the position of the buoy
  buoy_pub = new ros::Publisher(nh.advertise<buoy_finder::ColorBlobArray>("perception/buoy_pos", 1));

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
