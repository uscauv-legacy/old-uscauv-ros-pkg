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
#include <color_segmenter/FindBlobs.h>
#include <vector>
#include <ctype.h>

#include "cvutility.h"
#include "cvBlob/Blob.h"
#include "cvBlob/BlobResult.h"

#include <iostream>
#include <stdio.h>

#define RED_H_MIN 0
#define RED_H_MAX 85
#define RED_S_MIN 15
#define RED_S_MAX 75

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

#define BLUE_H_MIN 52
#define BLUE_H_MAX 127
#define BLUE_S_MIN 0
#define BLUE_S_MAX 255

#define NUM_COLORS 5

using namespace cv;
using namespace color_segmenter;

ros::ServiceServer * FindBlobs_srv;
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

//Mat itsCurrentImage;
sensor_msgs::ImageConstPtr itsCurrentImage;
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
	ROS_INFO("filtering image for color: %d", color);
  //IplImage img = itsCurrentImage;
  IplImage* img = bridge.imgMsgToCv(itsCurrentImage);
	
  int h_min = colors[color].h_min;
  int h_max = colors[color].h_max;
  int s_min = colors[color].s_min;
  int s_max = colors[color].s_max;

  Mat filterImg = cvFilterHS(img,h_min,h_max,s_min,s_max, 0);
  
  dilate(filterImg, filterImg, Mat(), Point(-1,-1), 5);
  erode(filterImg, filterImg, Mat(), Point(-1,-1), 2);
  dilate(filterImg, filterImg, Mat(), Point(-1,-1), 5);
  erode(filterImg, filterImg, Mat(), Point(-1,-1), 2);

  IplImage fImg = filterImg;

  cvFlipBinaryImg(&fImg);

  CBlobResult blobs;
  blobs = CBlobResult(&fImg, NULL, 0, false);

  blobs.Filter(blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, 500);
  blobs.Filter(blobs, B_EXCLUDE, CBlobGetArea(), B_GREATER, 10000);

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


//cv::Rect findTheFuckingRedBuoy(cv::Mat img)
color_segmenter::ColorBlobArray findTheFuckingRedBuoy(int color)
{
  //Miscellaneous parameters...
  int GaussianSize = 5;   //The size of the gaussian used in making the DOG pyramid
  float csGain = 80;      //A huge offset for the center surround pyramid levels
  int PyramidLevels = 6;  //The number of pyramid levels to compute
  int delta = 1;          //The number of octaves between subtracted pyramid levels

  //get image
  IplImage* iplImg = bridge.imgMsgToCv(itsCurrentImage);

  //Convert IplImage to cv::Mat and copy data
  cv::Mat img(iplImg);

  //Extract the raw red, green, and blue channels
  std::vector<cv::Mat> imgChannels;
  cv::split(img, imgChannels);
  cv::Mat b = imgChannels[0];
  cv::Mat g = imgChannels[1];
  cv::Mat r = imgChannels[2];

  //Produce the 'pure' color components
  //On second thought, don't.
  cv::Mat red   = r;// - ((b + g) * 0.5);
  cv::Mat blue  = b;// - ((r + g) * 0.5);
  cv::Mat green = g;// - ((r + b) * 0.5);

  //Convert BGR to HSV space
//   cv::Mat hsvImg;
//   cvtColor(img, hsvImg, CV_BGR2HSV);

//   cv::Mat img_hue_ = cv::Mat::zeros(hsvImg.rows, hsvImg.cols, CV_8U);
//   cv::Mat img_sat_ = cv::Mat::zeros(hsvImg.rows, hsvImg.cols, CV_8U);
//   cv::Mat img_bin_ = cv::Mat::zeros(hsvImg.rows, hsvImg.cols, CV_8U);

//   // HSV Channel 0 -> img_hue_ & HSV Channel 1 -> img_sat_
//   int from_to[] = { 0,0, 1,1};
//   cv::Mat img_split[] = { img_hue_, img_sat_};
//   cv::mixChannels(&hsvImg, 3,img_split,2,from_to,2);
  
//   //Get orange
//   cv::Mat orange = img.clone();
  
//   for(int i = 0; i < orange.rows; i++)
//     {
//       for(int j = 0; j < orange.cols; j++)
// 	{
// 	  // The output pixel is white if the input pixel
// 	  // hue is orange and saturation is reasonable
// 	  if(img_hue_.at<int>(i,j) > 4 &&
// 	     img_hue_.at<int>(i,j) < 28 &&
// 	     img_sat_.at<int>(i,j) > 128) 
// 	       {
// 		 img_bin_.at<int>(i,j) = 255;
// 	       } else {
// 		 img_bin_.at<int>(i,j) = 0;
// 		 // Clear pixel blue output channel
// 		 orange.at<int>(i,j*3+0) = 0;
// 		 // Clear pixel green output channel
// 		 orange.at<int>(i,j*3+1) = 0;
// 		 // Clear pixel red output channel
// 		 orange.at<int>(i,j*3+2) = 0;
// 	       }
//          }
//     }


//Build a gaussian pyramid for the red and green components
std::vector<cv::Mat> redPyramid(PyramidLevels);

//   switch(color){
//   case 0:
//     redPyramid[0] = red;
//     break;
//   case 1:
//     //redPyramid[0] = orange;
//     break;
//   case 2:
//     //redPyramid[0] = yellow;
//     break;
//   case 3:
//     redPyramid[0] = green;
//     break;
//   case 4:
//     redPyramid[0] = blue;
//     break;
//   default:
//     break;
//   }

  redPyramid[0] = red;
  
  for(int i=1; i<PyramidLevels; i++)
    cv::GaussianBlur(redPyramid[i-1],   redPyramid[i],   cv::Size(GaussianSize, GaussianSize), 0);

  //Create a center surround pyramid by subtracting various octaves from they pyramid
  cv::Mat gatedImg = red;
  std::vector<cv::Mat> rgPyramid;
  for(int i=0; i<PyramidLevels-delta; ++i)
  {
    rgPyramid.push_back( (redPyramid[i]-redPyramid[i+delta])*csGain );

    //At each level of the CS pyramid, find the minimum between that level, and a gating image.
    cv::min(gatedImg,rgPyramid[rgPyramid.size()-1], gatedImg);
  }

  //Dilate and erode a few times to fill in the holes
  cv::dilate(gatedImg, gatedImg, cv::Mat(), cv::Point(-1,-1), 5 );
  cv::erode(gatedImg, gatedImg, cv::Mat(), cv::Point(-1,-1), 2 );
  cv::dilate(gatedImg, gatedImg, cv::Mat(), cv::Point(-1,-1), 5 );
  cv::erode(gatedImg, gatedImg, cv::Mat(), cv::Point(-1,-1), 2 );

  //Threshold this guy so we end up with a binary image
  cv::threshold(gatedImg, gatedImg, 200, 255, cv::THRESH_BINARY);

  //Find the connected components in the image
  std::vector<cv::Vec4i> hierarchy;
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(gatedImg, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  //Find the largest connected component
  double maxArea=-1;
  std::vector<cv::Point> maxContour;
  int maxContourIdx = 0;
  for(int i=0; i<contours.size(); i++)
  {
    double area = -cv::contourArea(contours[i]);
    if(area > maxArea)
    {
      maxArea = area;
      maxContour = contours[i];
      maxContourIdx = i;
    }
  }

  //Return the bounding rectangle around the largest connected component, or a blank rectangle
  cv::Rect maxRect;
  if(maxArea > 0)
    maxRect = cv::boundingRect(maxContour);
  //return maxRect;
  //Convert cv::Rect to colorBlobArray
  color_segmenter::ColorBlobArray msgBlobs;
  color_segmenter::ColorBlob msgBlob;
  msgBlob.X = maxRect.x;
  msgBlob.Y = maxRect.y;
  msgBlob.Area = maxRect.width*maxRect.height;
  msgBlob.Color = color;
  msgBlobs.ColorBlobs.push_back(msgBlob);


  // Publish debug image if requested
  if(show_dbg_img)
    {
      cv::Mat dbgImg = img;
      
      //Draw the buoy
      if(maxArea > 0)
	cv::rectangle(dbgImg, cv::Point(maxRect.x, maxRect.y), cv::Point(maxRect.x+maxRect.width, maxRect.y+maxRect.height), cv::Scalar(255), 3);      
      //Convert our debug image to an IplImage and send it out
      IplImage dImg = dbgImg;

      dbg_img_pub->publish(bridge.cvToImgMsg(&dImg));
    }
  
  return msgBlobs;
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO("Got image update");
  boost::lock_guard<boost::mutex> lock(image_mutex); 
  //itsCurrentImage = bridge.imgMsgToCv(msg);
  itsCurrentImage = msg;
  
  if(show_dbg_img)
    filterColor(show_dbg_code);  
}

bool FindBlobsCallback(color_segmenter::FindBlobs::Request & req, color_segmenter::FindBlobs::Response & resp)
{
  ROS_INFO("got a request to segment an image for color: %d", req.DesiredColor);
  boost::lock_guard<boost::mutex> lock(image_mutex); 
  int colorId = req.DesiredColor; //getColorId(req.DesiredColor);

  if(colorId < 0)
    return false;		
  resp.BlobArray =  findTheFuckingRedBuoy(colorId);
  //	resp.BlobArray = filterColor(colorId);
	// rand's function goes in here
	// run rands code, give it input image
	// take bounding rectangle he produces and create blob array
	
  ROS_INFO("found %d blobs after filtering image", (int)resp.BlobArray.ColorBlobs.size() );

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
  ros::NodeHandle n("~");

  image_transport::ImageTransport it(n);

  std::string transport;

  //Register the show debug image parameter
  n.param("show_debug_img", show_dbg_img, 1);
  n.param("debug_color", show_dbg_color, std::string("red"));
  n.param("image_transport", transport, std::string("raw"));

  ROS_INFO("show_dbg_img: %d",show_dbg_img);
  ROS_INFO("show_dbg_color: %s",show_dbg_color.c_str());
  ROS_INFO("image_transport: %s",transport.c_str());

  show_dbg_code = getColorId(show_dbg_color);
  if(show_dbg_code < 0)
    {
      ROS_INFO("Invalid color for debug specified: %s",show_dbg_color.c_str());
      show_dbg_img = 0;
    }

  initColorRanges();

  ros::ServiceServer FindBlobs_srv = n.advertiseService("FindBlobs", FindBlobsCallback);

  //Register a publisher for a debug image
  dbg_img_pub = new image_transport::Publisher(it.advertise("debug/image_color", 1));

  //Register a subscriber to an input image
  image_transport::Subscriber sub = it.subscribe(n.resolveName("image"), 1, &imageCallback, transport);

  dynamic_reconfigure::Server<color_segmenter::ColorSegmenterConfig> srv;
  dynamic_reconfigure::Server<color_segmenter::ColorSegmenterConfig>::CallbackType f = boost::bind(&reconfigureCallback, _1, _2);
  srv.setCallback(f);

  ros::spin();

  return 0;
}
