/***************************************************************************
 *  include/base_libs/image_loader.h
 *  --------------------
 * 
 *  Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *  
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer
 *    in the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of seabee3-ros-pkg nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *  
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 **************************************************************************/

#ifndef BASE_LIBS_BASE_LIBS_IMAGE_LOADER_H_
#define BASE_LIBS_BASE_LIBS_IMAGE_LOADER_H_

#include <vector>
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <ros/ros.h>
// for ImageTransport, Publisher
#include <image_transport/image_transport.h>
// for Mat
#include <opencv/cv.h>
#include <opencv/cxcore.h>
// for image utilities like imread()
#include <opencv/highgui.h>
//for CvBridge
#include <cv_bridge/CvBridge.h>
#include <base_libs/param_reader.h>

class ImageLoader
{
private:
	ros::NodeHandle nh_priv_;

public:
	typedef std::vector<IplImage *> _ImageCache;

	_ImageCache image_cache_;
	bool images_loaded_;

	std::string file_prefix_, file_ext_;
	int start_, end_, digits_, width_, height_;
	int load_flag_;

	ImageLoader( ros::NodeHandle nh, int load_flag = CV_LOAD_IMAGE_COLOR ) :
		images_loaded_( false ),
		load_flag_( load_flag )
	{
		PRINT_INFO( "Setting up image_loader..." );
		
		file_prefix_ = ros::ParamReader<std::string, 1>::readParam( nh, "prefix", "image" );
		file_ext_ = ros::ParamReader<std::string, 1>::readParam( nh, "ext", ".png" );
		start_ = ros::ParamReader<int, 1>::readParam( nh, "start", 0 );
		end_ = ros::ParamReader<int, 1>::readParam( nh, "end", 0 );
		digits_ = ros::ParamReader<int, 1>::readParam( nh, "digits", 0 );
		width_ = ros::ParamReader<int, 1>::readParam( nh, "width", -1 );
		height_ = ros::ParamReader<int, 1>::readParam( nh, "height", -1 );

		PRINT_INFO( "Done setting up image loader" );
	}

	~ImageLoader()
	{
		PRINT_INFO( "Releasing image cache..." );
		for( unsigned int i = 0; i < image_cache_.size(); ++i )
		{
			cvReleaseImage( &image_cache_[i] );
		}
		PRINT_INFO( "Done releasing image cache" );
	}

	_ImageCache loadImages( const bool & force = false )
	{
		if( !force && images_loaded_ ) return image_cache_;
		
		PRINT_INFO( "Loading images %s %d %d %d %s ...", file_prefix_.c_str(), start_, end_, digits_, file_ext_.c_str() );
		IplImage * img = NULL;
		for ( int i = start_; i <= end_; i++ )
		{
			std::stringstream filename;
			if( digits_ > 0 ) filename << file_prefix_ << std::setfill( '0' ) << std::setw( digits_ ) << i << file_ext_;
			else filename << file_prefix_ << file_ext_;


			PRINT_INFO( "Attempting to open %s", filename.str().c_str() );
			img = cvLoadImage( filename.str().c_str(), load_flag_ );

			if ( img && img->width * img->height > 0 )
			{
				PRINT_INFO( "Success: %dx%d", img->width, img->height );
				if( width_ > 0 || height_ > 0 )
				{
					double width = width_ > 0 ? width_ : img->width;
					double height = height_ > 0 ? height_ : img->height;
					// scale image, maintain aspect ratio
					cv::Point2d scales( width / img->width, height / img->height );
					double scale = img->width * scales.x <= width && img->height * scales.x <= height ? scales.x : scales.y;
					IplImage * scaled_img = cvCreateImage( cvSize( img->width * scale, img->height * scale ), img->depth, img->nChannels );
					PRINT_INFO( "Scaling image to %dx%d", scaled_img->width, scaled_img->height );
					cvResize( img, scaled_img, CV_INTER_CUBIC );

					cvReleaseImage( &img );
					image_cache_.push_back( scaled_img );
				}
				else
				{
					image_cache_.push_back( img );
				}
			}
			else
			{
				PRINT_WARN( "Ignoring %s; does not exist in filesystem", filename.str().c_str() );
			}
		}

		images_loaded_ = true;

		PRINT_INFO( "Done loading images" );

		return image_cache_;
	}
};

#endif // BASE_LIBS_BASE_LIBS_IMAGE_LOADER_H_
