/*******************************************************************************
 *
 *      image_matcher_new
 * 
 *      Copyright (c) 2011, sunzun
 *      All rights reserved.
 *
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions are
 *      met:
 *      
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following disclaimer
 *        in the documentation and/or other materials provided with the
 *        distribution.
 *      * Neither the name of "seabee3-ros-pkg" nor the names of its
 *        contributors may be used to endorse or promote products derived from
 *        this software without specific prior written permission.
 *      
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *      A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_matcher/MatchImage.h>
#include <cv_bridge/CvBridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <image_transport/image_transport.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <fstream>
#include <time.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

using namespace std

// File names for the bin objects
string axeFile;
string clippersFile;
string hammerFile;
string macheteFile;

sensor_msgs::ImageConstPtr itsCurrentImage;
sensor_msgs::ImageConstPtr testImage;
CvMemStorage* objectStorage;
CvSeq axeMoments;
CvSeq axeHuMoments;
CvSeq clippersMoments;
CvSeq clippersHuMoments;
CvSeq hammerMoments;
CvSeq hammerHuMoments;
CvSeq macheteMoments;
CvSeq macheteHuMoments;
bool goodToGo = false;

boost::mutex image_mutex;

void imageCallback( const sensor_msgs::ImageConstPtr& msg )
{
	ROS_INFO( "Image received" );
	boost::lock_guard<boost::mutex> lock( image_mutex );
	itsCurrentImage = msg;
	goodToGo = true;
}

bool matchImageCall ( image_matcher::MatchImage::Request &req, image_matcher::MatchImage::Response &res )
{
	boost::lock_guard<boost::mutex> lock( image_mutex );

	if ( !goodToGo ) return false;

	sensor_msgs::CVBridge bridge;
	CvSeq* imageMoments;
	CvSeq* imageHuMoments;
	CvMemStorage* storage = cvCreateMemStorage( 0 );

	IplImage* image = bridge.imgMsgToCv( itsCurrentImage );
	IplImage* grayImage = cvCreateImage( cvGetSize( image ), 8, 1 );
	cvCvtColor( image, grayImage, CV_BGR2GRAY);

	ROS_INFO( "Calculating Moments and Hu Moments" );
	cvMoments( grayImage, &imageMoments, 1);
	cvMoments( &imageMoments, &imageHuMoments, 1);
}


void loadPlusConvertImages()
{
	IplImage* axeImg;
	IplImage* clippersImg;
	IplImage* hammerImg;
	IplImage* macheteImg;
	CvMemStorage* objectStorage = cvCreateMemStorage( 0 );


	// Extract images from their respective files
	axeImg = cvLoadImage( axeFile.c_str() );
	clippersImg = cvLoadImage( clippersFile.c_str() );
	hammerImg = cvLoadImage( hammerFile.c_str() );
	macheteImg = cvLoadImage( macheteFile.c_str() );


	// Create grayscale image templates.
	IplImage* grayAxe = cvCreateImage( cvGetSize( axeImg ), 8, 1 );
	IplImage* grayClippers = cvCreateImage( cvGetSize( clippersImg ), 8, 1 );
	IplImage* grayHammer = cvCreateImage( cvGetSize( hammerImg ), 8, 1 );
	IplImage* grayMachete = cvCreateImage( cvGetSize( macheteImg ), 8, 1 );


	// Convert all images to grayscale
	cvCvtColor( axeImg, grayAxe, CV_BGR2GRAY );
	cvCvtColor( clippersImg, grayClippers, CV_BGR2GRAY );
	cvCvtColor( hammerImg, grayHammer, CV_BGR2GRAY );
	cvCvtColor( macheteImg, grayMachete, CV_BGR2GRAY );

	// Calculate Moments for the images
	cvMoments( grayAxe, &axeMoments, 1);
	cvMoments( grayClippers, &clippersMoments, 1);
	cvMoments( grayHammer, &hammerMoments, 1);
	cvMoments( grayMachete, &macheteMoments, 1);

	//Calculate Hu Moments for the images
	cvGetHuMoments( &axeMoments, &axeHuMoments);
	cvGetHuMoments( &clippersMoments, &clippersHuMoments);
	cvGetHuMoments( &hammerMoments, &hammerHuMoments);
	cvGetHuMoments( &macheteMoments, &macheteHuMoments);

}

int main( int argc, char** argv )
{
	// Initialize ros structures
	ros::init( argc, argv, "image_matcher" );
	ros::NodeHandle nh;//("~");
	image_transport::ImageTransport it( nh );
	std::string transport;


	// File paths for objects
	//nh.param("image_transport", transport, std::string("raw"));
	nh.param( "axe", axeFile, std::string( "axe.png" ) );
	nh.param( "clippers", clippersFile, std::string( "clippers.png" ) );
	nh.param( "hammer", hammerFile, std::string( "hammer.png" ) );
	nh.param( "machete", macheteFile, std::string( "machete.png" ) );

	loadPlusConvertImages();
	ROS_INFO( "Finished loading and converting competition templates");

	//Subscribe to image topic
	image_transport::Subscriber sub = it.subscribe( "image", 1, &imageCallback );

	//Register service
	ros::ServiceServer image_match_srv = nh.advertiseService( "MatchImage", matchImageCall );
}
