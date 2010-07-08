#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

#include <cv_bridge/CvBridge.h>
#include <sensor_msgs/Image.h>
#include <dynamic_reconfigure/server.h>

#include <color_segmenter/ColorSegmenterConfig.h>
#include <color_segmenter/ColorBlobArray.h>
#include <color_segmenter/SegmentImage.h>
#include <vector>
#include <ctype.h>

#include "cvutility.h"
#include "cvBlob/Blob.h"
#include "cvBlob/BlobResult.h"

#define RED_H_MIN 75
#define RED_H_MAX 255
#define RED_S_MIN 0
#define RED_S_MAX 255

#define ORANGE_H_MIN 75
#define ORANGE_H_MAX 255
#define ORANGE_S_MIN 0
#define ORANGE_S_MAX 255

#define YELLOW_H_MIN 75
#define YELLOW_H_MAX 255
#define YELLOW_S_MIN 0
#define YELLOW_S_MAX 255

#define GREEN_H_MIN 75
#define GREEN_H_MAX 255
#define GREEN_S_MIN 0
#define GREEN_S_MAX 255

#define BLUE_H_MIN 75
#define BLUE_H_MAX 255
#define BLUE_S_MIN 0
#define BLUE_S_MAX 255

#define NUM_COLORS 5

using namespace cv;
using namespace color_segmenter;

ros::ServiceServer * SegmentImage_srv;
image_transport::Publisher *dbg_img_pub; 
sensor_msgs::CvBridge bridge;

struct ColorRange
{
  int h_min, h_max, s_min, s_max;
};

std::vector<ColorRange> colors;

int show_dbg_img;
int show_dbg_code;

std::string show_dbg_color;

Mat itsCurrentImage;
boost::mutex image_mutex;

struct BlobColorDefs
{
	const static int RED = 0;
	const static int ORANGE = 1;
	const static int YELLOW = 2;
	const static int GREEN = 3;
	const static int BLUE = 4;
};

string stringToLowercase(std::string strToConvert)
{
  for(unsigned int i=0;i<strToConvert.length();i++)
    strToConvert[i] = tolower(strToConvert[i]);
  
  return strToConvert;
}

int getColorId(std::string color)
{
  color = stringToLowercase(color);

  if(!color.compare("red"))
    return BlobColorDefs::RED;
  else if(!color.compare("orange"))
    return BlobColorDefs::ORANGE;
  else if(!color.compare("yellow"))
    return BlobColorDefs::YELLOW;
  else if(!color.compare("green"))
    return BlobColorDefs::GREEN;
  else if(!color.compare("blue"))
    return BlobColorDefs::BLUE;
  else 
    return -1;
}

void reconfigureCallback(color_segmenter::ColorSegmenterConfig &config, uint32_t level)
{
  //   ROS_INFO("Reconfigure request : %d %d %d %d / %d %d %d %d",
  //            config.red_h_min, config.red_h_max, config.red_s_min, config.red_s_max,
  // 	   config.orange_h_min, config.orange_h_max, config.orange_s_min, config.orange_s_max);
  
  colors[BlobColorDefs::RED].h_min = config.red_h_min;
  colors[BlobColorDefs::RED].h_max = config.red_h_max;
  colors[BlobColorDefs::RED].s_min = config.red_s_min;
  colors[BlobColorDefs::RED].s_max = config.red_s_max;

  colors[BlobColorDefs::ORANGE].h_min = config.orange_h_min;
  colors[BlobColorDefs::ORANGE].h_max = config.orange_h_max;
  colors[BlobColorDefs::ORANGE].s_min = config.orange_s_min;
  colors[BlobColorDefs::ORANGE].s_max = config.orange_s_max;

  colors[BlobColorDefs::YELLOW].h_min = config.yellow_h_min;
  colors[BlobColorDefs::YELLOW].h_max = config.yellow_h_max;
  colors[BlobColorDefs::YELLOW].s_min = config.yellow_s_min;
  colors[BlobColorDefs::YELLOW].s_max = config.yellow_s_max;

  colors[BlobColorDefs::GREEN].h_min = config.green_h_min;
  colors[BlobColorDefs::GREEN].h_max = config.green_h_max;
  colors[BlobColorDefs::GREEN].s_min = config.green_s_min;
  colors[BlobColorDefs::GREEN].s_max = config.green_s_max;

  colors[BlobColorDefs::BLUE].h_min = config.blue_h_min;
  colors[BlobColorDefs::BLUE].h_max = config.blue_h_max;
  colors[BlobColorDefs::BLUE].s_min = config.blue_s_min;
  colors[BlobColorDefs::BLUE].s_max = config.blue_s_max;
}

color_segmenter::ColorBlobArray filterColor(int color)
{
  IplImage img = itsCurrentImage;
	
  int h_min = colors[color].h_min;
  int h_max = colors[color].h_max;
  int s_min = colors[color].s_min;
  int s_max = colors[color].s_max;

  Mat filterImg = cvFilterHS(&img,h_min,h_max,s_min,s_max,0);
  
  dilate(filterImg, filterImg, Mat(), Point(-1,-1), 5);
  erode(filterImg, filterImg, Mat(), Point(-1,-1), 3);

  IplImage fImg = filterImg;

  cvFlipBinaryImg(&fImg);

  CBlobResult blobs;
  blobs = CBlobResult(&fImg, NULL, 0, false);

  blobs.Filter(blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, 5);
  blobs.Filter(blobs, B_EXCLUDE, CBlobGetArea(), B_GREATER, 15000);

  color_segmenter::ColorBlobArray msgBlobs;

  CBlob currentBlob;
  Mat dbgImg;
  cvtColor(filterImg, dbgImg, CV_GRAY2RGB);

  for(int i = 0; i < blobs.GetNumBlobs(); i++)
    {
      blobs.GetNthBlob(CBlobGetArea(), i, currentBlob);
      
      float blobArea = currentBlob.Area();
      float blobX = (currentBlob.MinX() + currentBlob.MaxX()) / 2.0;
      float blobY = (currentBlob.MinY() + currentBlob.MaxY()) / 2.0;
      
      // normalize buoy position relative to image center
      blobX = (blobX / fImg.width) - 0.5;
      blobY = (blobY / fImg.height) - 0.5;
      
      color_segmenter::ColorBlob msgBlob;
      msgBlob.X = blobX;
      msgBlob.Y = blobY;
      msgBlob.Area = blobArea;
      msgBlob.Color = color;
      
      msgBlobs.ColorBlobs.push_back(msgBlob);
      

      // Draw blob information on debug image
      if(show_dbg_img)
	{	  
	  float blobArea = currentBlob.Area();
	  float blobX = (currentBlob.MinX() + currentBlob.MaxX()) / 2.0;
	  float blobY = (currentBlob.MinY() + currentBlob.MaxY()) / 2.0;
	  
	  //Print out an annoying message so that we remember to disable the debug image
	  ROS_INFO("Showing Debug Image - blob #%d at %f,%f area %f", i+1, blobX, blobY, blobArea);
	  
	  Point center(blobX, blobY);
	  rectangle( dbgImg, Point(currentBlob.MinX(),currentBlob.MinY()), Point(currentBlob.MaxX(),currentBlob.MaxY()), Scalar(0,255,0));
	  // draw the circle center
	  circle( dbgImg, center, 3, Scalar(0,255,0), -1, 8, 0 );
	  // draw the circle outline
	  circle( dbgImg, center, 5, Scalar(0,0,255), 3, 8, 0 );
	}      
    }

  // Publish debug image if requested
  if(show_dbg_img)
    {
      //Convert our debug image to an IplImage and send it out
      IplImage dImg = dbgImg;
      dbg_img_pub->publish(bridge.cvToImgMsg(&dImg));
    }

  return msgBlobs;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  boost::mutex::scoped_lock lock(image_mutex); 
  itsCurrentImage = bridge.imgMsgToCv(msg);
  
  if(show_dbg_img)
    filterColor(show_dbg_code);  
}

bool segmentImageCallback(color_segmenter::SegmentImage::Request & req, color_segmenter::SegmentImage::Response & resp)
{
  boost::mutex::scoped_lock lock(image_mutex); 
  int colorId = req.DesiredColor; //getColorId(req.DesiredColor);

  if(colorId < 0)
    return false;		
 
  resp.BlobArray = filterColor(colorId);

  if(resp.BlobArray.ColorBlobs.size() > 0)
    return true;
  else
    return false;
}

void initColorRanges()
{
  colors.reserve(NUM_COLORS);
  
  ColorRange red;
  red.h_min = RED_H_MIN;
  red.h_max = RED_H_MAX;
  red.s_min = RED_S_MIN;
  red.s_max = RED_S_MAX;
  colors[BlobColorDefs::RED] = red;

  ColorRange orange;
  orange.h_min = ORANGE_H_MIN;
  orange.h_max = ORANGE_H_MAX;
  orange.s_min = ORANGE_S_MIN;
  orange.s_max = ORANGE_S_MAX;
  colors[BlobColorDefs::ORANGE] = orange;

  ColorRange yellow;
  yellow.h_min = YELLOW_H_MIN;
  yellow.h_max = YELLOW_H_MAX;
  yellow.s_min = YELLOW_S_MIN;
  yellow.s_max = YELLOW_S_MAX;
  colors[BlobColorDefs::YELLOW] = yellow;

  ColorRange green;
  green.h_min = GREEN_H_MIN;
  green.h_max = GREEN_H_MAX;
  green.s_min = GREEN_S_MIN;
  green.s_max = GREEN_S_MAX;
  colors[BlobColorDefs::GREEN] = green;

  ColorRange blue;
  blue.h_min = BLUE_H_MIN;
  blue.h_max = BLUE_H_MAX;
  blue.s_min = BLUE_S_MIN;
  blue.s_max = BLUE_S_MAX;
  colors[BlobColorDefs::BLUE] = blue;
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "color_segmenter");
  ros::NodeHandle n;

  image_transport::ImageTransport it(n);

  //Register the show debug image parameter
  n.param("show_dbg_img", show_dbg_img, 1);
  n.param("show_dbg_color", show_dbg_color, std::string("red"));

  show_dbg_code = getColorId(show_dbg_color);
  if(show_dbg_code < 0)
    {
      ROS_INFO("Invalid color for debug specified: %s",show_dbg_color.c_str());
      show_dbg_img = 0;
    }

  initColorRanges();

  ros::ServiceServer segmentImage_srv = n.advertiseService("segmentImage", segmentImageCallback);

  //Register a publisher for a debug image
  dbg_img_pub = new image_transport::Publisher(it.advertise("perception/color_seg_dbg", 1));

  //Register a subscriber to an input image
  image_transport::Subscriber sub = it.subscribe("image", 1, imageCallback);

  dynamic_reconfigure::Server<color_segmenter::ColorSegmenterConfig> srv;
  dynamic_reconfigure::Server<color_segmenter::ColorSegmenterConfig>::CallbackType f = boost::bind(&reconfigureCallback, _1, _2);
  srv.setCallback(f);

  ros::spin();

  return 0;
}
