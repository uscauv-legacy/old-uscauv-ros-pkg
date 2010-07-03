#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include "cvutility.h"

#define RED_H_MIN 60
#define RED_H_MAX 180
#define RED_S_MIN 100
#define RED_S_MAX 255

ros::ServiceServer * SegmentImage_srv;

int blobDetect() {
  IplImage* input;
  IplImage* img;
  IplImage *hsv_img;
  //  IplImage *bw_img;
  IplImage* i1;
  
  ROS_INFO("Trying to open image");

  // Initialize image, allocate memory
  input = cvLoadImage("buoy.png", 1);
  img = cvCloneImage(input);
  hsv_img = cvCloneImage(img);

  ROS_INFO("Opened image");

  // Smooth input image using a Gaussian filter, assign HSV, BW image
  cvSmooth(input, img, CV_GAUSSIAN, 7, 9, 0 ,0);

  i1 = cvFilterHS(img, RED_H_MIN, RED_H_MAX, RED_S_MIN, RED_S_MAX, 0);

  // Detect Blobs using the mask and a threshold for area. Sort blobs according to area

//   blobs = CBlobResult(bw_img, i1, 0);
//   blobs.Filter(blobs, B_INCLUDE, CBlobGetArea(), B_GREATER, 500);
//   blobs.GetNthBlob(CBlobGetArea(), blobs.GetNumBlobs() - 1, blobArea); // example

//   for (int i = 0; i < blobs.GetNumBlobs(); i++ )
//     {
//       blobArea = blobs.GetBlob(i);
//       blobArea.FillBlob(img, cvScalar(255, 0, 0));
//     }  

  cvNamedWindow("Output", 1);
  cvShowImage("Output", i1);

  // Display blobs
  cvNamedWindow("Input", 1);
  cvShowImage("Input", input);

  cvWaitKey(0); // Wait till windows are closed
  // Cleanup
  cvReleaseImage(&i1);
  //  cvReleaseImage(&bw_img);
  cvReleaseImage(&hsv_img);
  cvReleaseImage(&img);
  cvReleaseImage(&input);
  return 0;
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "color_segmenter");
  ros::NodeHandle n;

  blobDetect();

  // 	ros::ServiceServer SegmentImage_srv = n.advertiseService("/color_segmenter/SegmentImage", SegmentImageCallback);
	
  // 	segmenter = new imageSegmenter2( 2 );
	
}
