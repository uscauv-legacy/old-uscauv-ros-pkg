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

using namespace std;

typedef BaseNodeTypes::_DefaultReconfigureType _ReconfigureType;
typedef image_matcher::MatchImage _ServiceType;

class ImageMatcher: public BaseImageProc<_ReconfigureType, _ServiceType>
{

public:
	ImageMatcher( ros::NodeHandle & nh ) :
		BaseImageProc<_ReconfigureType, _ServiceType> ( nh, "match_images" ), image_size_initialized_( false )
	{

	}

	ImageMatcher::~ImageMatcher()
	{
		cvReleaseImage( &gray_axe_ );
		cvReleaseImage( &gray_clippers_ );
		cvReleaseImage( &gray_hammer_ );
		cvReleaseImage( &gray_machete_ );
	}
	// File names for the bin objects
	string axe_file_;
	string clippers_file_;
	string hammer_file_;
	string machete_file_;

	// Create grayscale image templates.
	IplImage * gray_axe_;
	IplImage * gray_clippers_;
	IplImage * gray_hammer_;
	IplImage * gray_machete_;
	IplImage * gray_image_;
	bool image_size_initialized_;
	bool images_loaded_ = false;
	const uint match_method_ = CV_CONTOURS_MATCH_I3;

	std::vector<double> matchImageCall( IplImage * image, _ServiceRequest &req )
	{
		std::vector<double> result;
		cvCvtColor( image, gray_image_, CV_BGR2GRAY);
		if ( req.desired_image == _ServiceType::IMAGE_ALL || req.desired_image == _ServiceType::IMAGE_AXE )
		{
			result.push_back( cvMatchShapes( gray_axe_, gray_image_, match_method_, 0 ) );
		}
		else if ( req.desired_image == _ServiceType::IMAGE_ALL || req.desired_image == _ServiceType::IMAGE_AXE )
		{
			result.push_back( cvMatchShapes( gray_clippers_, gray_image_, match_method_, 0 ) );
		}
		else if ( req.desired_image == _ServiceType::IMAGE_ALL || req.desired_image == _ServiceType::IMAGE_AXE )
		{
			result.push_back( cvMatchShapes( gray_hammer_, gray_image_, match_method_, 0 ) );
		}
		else if ( req.desired_image == _ServiceType::IMAGE_ALL || req.desired_image == _ServiceType::IMAGE_AXE )
		{
			result.push_back( cvMatchShapes( gray_machete_, gray_image_, match_method_, 0 ) );
		}

		ROS_INFO( "Match Value::: Axe: %d Clippers: %d Hammer: %d Machete: %d", axe_match, clippers_match, hammer_match, machete_match );

		return result;

	}

	void loadPlusConvertImages()
	{
		if ( !images_loaded_ )
		{
			images_loaded_ = true;

			IplImage* axe_img;
			IplImage* clippers_img;
			IplImage* hammer_img;
			IplImage* machete_img;

			// Extract images from their respective files
			axe_img = cvLoadImage( axeFile.c_str() );
			clippers_img = cvLoadImage( clippersFile.c_str() );
			hammer_img = cvLoadImage( hammerFile.c_str() );
			machete_img = cvLoadImage( macheteFile.c_str() );

			// Create grayscale image templates.
			gray_axe_ = cvCreateImage( cvGetSize( axe_img ), IPL_DEPTH_8U, 1 );
			gray_clippers_ = cvCreateImage( cvGetSize( clippers_img ), IPL_DEPTH_8U, 1 );
			gray_hammer_ = cvCreateImage( cvGetSize( hammer_img ), IPL_DEPTH_8U, 1 );
			gray_machete_ = cvCreateImage( cvGetSize( machete_img ), IPL_DEPTH_8U, 1 );


			// Convert all images to grayscale
			cvCvtColor( axe_img, gray_axe_, CV_BGR2GRAY );
			cvCvtColor( clippers_img, gray_clippers_, CV_BGR2GRAY );
			cvCvtColor( hammer_img, gray_hammer_, CV_BGR2GRAY );
			cvCvtColor( machete_img, gray_machete_, CV_BGR2GRAY );

			cvReleaseImage( &axe_img );
			cvReleaseImage( &clippers_img );
			cvReleaseImage( &hammer_img );
			cvReleaseImage( &machete_img );
		}

	}

	cv::Mat processImage( IplImage * ipl_img, _ServiceRequest & req, _ServiceResponse & resp )
	{
		if ( !image_size_initialized_ )
		{
			gray_image_ = cvCreateImage( cvGetSize( ipl_img ), IPL_DEPTH_8U, 1 );
		}

		resp.PercentMatch = matchImageCall( ipl_img, req );
		return cv_img_;
	}
};

int main( int argc, char** argv )
{
	// Initialize ros structures
	ros::init( argc, argv, "image_matcher" );
	ros::NodeHandle nh;//("~");

	ImageMatcher image_matcher( nh );


	// File paths for objects
	//nh.param("image_transport", transport, std::string("raw"));
	nh.param( "axe", axeFile, std::string( "axe.png" ) );
	nh.param( "clippers", clippersFile, std::string( "clippers.png" ) );
	nh.param( "hammer", hammerFile, std::string( "hammer.png" ) );
	nh.param( "machete", macheteFile, std::string( "machete.png" ) );

	image_matcher.loadPlusConvertImages();
	ROS_INFO( "Finished loading and converting competition templates" );

	image_matcher.spin();
}
