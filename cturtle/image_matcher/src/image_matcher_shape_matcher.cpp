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
#include <sensor_msgs/Image.h>
#include <image_matcher/MatchImage.h>
#include <yaml-cpp/yaml.h>
#include <map>
#include <string>
#include <fstream>
#include <time.h>
#include <base_image_proc/base_image_proc.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <image_matcher/ImageTemplate.h>

//using namespace std;
//using namespace cv;

typedef BaseNodeTypes::_DefaultReconfigureType _ReconfigureType;
typedef image_matcher::MatchImage _ServiceType;

class ImageMatcher: public BaseImageProc<_ReconfigureType, _ServiceType>
{

public:
	typedef std::pair<std::string, IplImage *> _ImageTemplate;
	typedef std::vector<_ImageTemplate> _ImageTemplateCache;
	// File names for the bin objects

	// Create grayscale image templates.
	_ImageTemplateCache image_cache_;
	ros::Subscriber image_template_sub_;

	ImageMatcher( ros::NodeHandle & nh ) :
		BaseImageProc<_ReconfigureType, _ServiceType> ( nh, "match_images" ), image_size_initialized_( false ), images_loaded_( false )
	{
		image_template_sub_ = nh.subscribe( nh.resolveName("image_template"), 1, &ImageMatcher::imageTemplateCB, this );


		/*CvMat axe_mat;
		 CvMat clippers_mat;
		 CvMat hammer_mat;
		 CvMat machete_mat;

		 axe_mat = imread(axe.jpeg, 1);
		 clippers_mat = imread(clippers.jpeg, 1);
		 hammer_mat = imread(hammer.jpeg, 1);
		 machete_mat = imread(machete.jpeg, 1);*/

		/*axe_file_ = "axe.jpeg";
		 clippers_file_ = "clippers.jpeg";
		 hammer_file_ = "hammer.jpeg";
		 machete_file_ = "machete.jpeg";*/

		nh.param( "axe", axe_file_, std::string( "../axe.jpeg" ) );
		nh.param( "clippers", clippers_file_, std::string( "../clippers.jpeg" ) );
		nh.param( "hammer", hammer_file_, std::string( "../hammer.jpeg" ) );
		nh.param( "machete", machete_file_, std::string( "../machete.jpeg" ) );
		/*axe_file_ = "axe.jpeg";
		 clippers_file_ = "clippers.jpeg";
		 hammer_file_ = "hammer.jpeg";
		 machete_file_ = "machete.jpeg";*/
		//ROS_ERROR("machete_file_ is %s", machete_file_);


		loadPlusConvertImages();
		ROS_INFO( "Finished loading and converting competition templates" );
	}

	~ImageMatcher()
	{
		cvReleaseImage(&gray_axe_);
		cvReleaseImage(&gray_clippers_);
		cvReleaseImage(&gray_hammer_);
		cvReleaseImage(&gray_machete_);
	}

	bool image_size_initialized_;
	bool images_loaded_;
	const static uint match_method_ = CV_CONTOURS_MATCH_I3;

	void imageTemplateCB( const image_matcher::ImageTemplateConstPtr & msg )
	{
		image_cache_.push_back( _ImageTemplate( msg->name, bridge_.imgMsgToCv( msg->image_template ) ) );
	}

	std::vector<double> matchImageCall( IplImage * image, _ServiceRequest &req )
	{
		std::vector<double> result;
		ROS_INFO( "%i 2", gray_image_->height );
		cvCvtColor( image, gray_image_, CV_BGR2GRAY);
		ROS_INFO( "%i 3", gray_image_->height );
		if ( req.desired_image == _ServiceRequest::IMAGE_ALL || req.desired_image == _ServiceRequest::IMAGE_AXE )
		{
			result.push_back( cvMatchShapes( gray_axe_, gray_image_, match_method_, 0 ) );
		}
		if ( req.desired_image == _ServiceRequest::IMAGE_ALL || req.desired_image == _ServiceRequest::IMAGE_CLIPPERS )
		{
			result.push_back( cvMatchShapes( gray_clippers_, gray_image_, match_method_, 0 ) );
		}
		if ( req.desired_image == _ServiceRequest::IMAGE_ALL || req.desired_image == _ServiceRequest::IMAGE_HAMMER )
		{
			result.push_back( cvMatchShapes( gray_hammer_, gray_image_, match_method_, 0 ) );
		}
		if ( req.desired_image == _ServiceRequest::IMAGE_ALL || req.desired_image == _ServiceRequest::IMAGE_MACHETE )
		{
			result.push_back( cvMatchShapes( gray_machete_, gray_image_, match_method_, 0 ) );
		}
		ROS_INFO( "%i 4", gray_image_->height );


		//ROS_INFO( "Match Value::: Axe: %d Clippers: %d Hammer: %d Machete: %d", axe_match, clippers_match, hammer_match, machete_match );

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


			// THIS IS ONLY A TEST!!!!!!!!!!
			//axe_file_ = "axe.jpeg";
			//ROS_ERROR ("axe file is %s, clippers file is %s", axe_file_, clippers_file_);
			//END TEST

			// Extract images from their respective files
			axe_img = cvLoadImage( axe_file_.c_str(), 1 );
			clippers_img = cvLoadImage( clippers_file_.c_str(), 1 );
			hammer_img = cvLoadImage( hammer_file_.c_str(), 1 );
			machete_img = cvLoadImage( machete_file_.c_str(), 1 );

			if ( !axe_img || !hammer_img || !clippers_img || !machete_img )
			{
				ROS_ERROR( "COULD NOT LOAD IMAGE!" );
			}

			ROS_INFO( "%i", cvSize( axe_img -> width, axe_img -> height ) );


			//ROS_ERROR ("axe file is %s, clippers file is %s", axe_file_, clippers_file_);

			//ROS_ERROR ("pointer clippers_img is %p", clippers_img);
			/*
			 IplImage* gray_axe_;
			 IplImage* gray_clippers_;
			 IplImage* gray_hammer_;
			 IplImage* gray_machete_;*/
			// Create grayscale image templates.
			//ROS_ERROR ("the width of axe image is %i", axe_img->width);

			gray_axe_ = cvCreateImage( cvSize( axe_img->width, axe_img->height ), IPL_DEPTH_8U, 1 );
			gray_clippers_ = cvCreateImage( cvSize( clippers_img->width, clippers_img->height ), IPL_DEPTH_8U, 1 );
			gray_hammer_ = cvCreateImage( cvSize( hammer_img->width, hammer_img->height ), IPL_DEPTH_8U, 1 );
			gray_machete_ = cvCreateImage( cvSize( machete_img->width, machete_img->height ), IPL_DEPTH_8U, 1 );


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
			gray_image_ = cvCreateImage( cvSize( ipl_img->width, ipl_img->height ), IPL_DEPTH_8U, 1 );
		}
		ROS_INFO( "%i 1", gray_image_->height );
		resp.percent_match = matchImageCall( ipl_img, req );
		ROS_INFO( "%d", cv_img_.size().width );
		return cv_img_;
	}
};

int main( int argc, char** argv )
{
	// Initialize ros structures
	ros::init( argc, argv, "image_matcher" );
	ros::NodeHandle nh;//("~");

	ImageMatcher image_matcher( nh );

	image_matcher.spin();
}
