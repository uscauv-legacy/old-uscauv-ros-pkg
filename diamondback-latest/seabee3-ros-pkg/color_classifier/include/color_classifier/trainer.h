/*******************************************************************************
 *
 *      trainer
 * 
 *      Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
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
 *      * Neither the name of "color_classifier-RelWithDebInfo@color_classifier" nor the names of its
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

#ifndef TRAINER_H_
#define TRAINER_H_

#include <base_node/base_node.h>
#include <common_utils/colors.h>
#include <common_utils/opencv.h>
#include <common_utils/math.h>
#include <list>
#include <string>

// given a set of pixels from one color, calculate the parameters needed to recognize all of those pixels later on
// calculate the mean and radius of the color components
// find the minimum radius and maximum weights such that all pixels are found

typedef BaseNode<> _BaseNode;
typedef Color<double, 3> _ColorType;

class Trainer : public _BaseNode
{
public:
	typedef std::list<_ColorType> _PixelList;

	_PixelList pixels_;
	_ColorType mean_;
	_ColorType variance_;

	std::string src_image_;
	std::string mask_image_;
	std::string color_name_;
	std::vector<std::string> channel_names_;

	ros::NodeHandle nh_reconfig_;

	Trainer( ros::NodeHandle & nh ) :
		_BaseNode( nh ), nh_reconfig_( ros::NodeHandle( nh_local_, "reconfigure" ) )
	{
		nh_local_.param("src_image", src_image_, std::string( "" ) );
		nh_local_.param("mask_image", mask_image_, std::string( "" ) );
		nh_local_.param("color_name", color_name_, std::string( "" ) );

		std::string current_channel_name;
		bool param_found = true;

		for( unsigned int i = 0; param_found; ++i )
		{
			std::stringstream ss;
			ss << "channel" << i << "_name";
			param_found = nh_local_.getParam( ss.str(), current_channel_name );
			if( !param_found ) current_channel_name = "";
			channel_names_.push_back( current_channel_name );
		}
	}

	// NOTE: modifies src_image
	void calculateStatistics( IplImage * src_image,
	                          IplImage * mask )
	{
		cvCvtColor( src_image,
		            src_image,
		            CV_BGR2HSV );

		unsigned char * src_image_pixel;
		unsigned char * mask_pixel;
		_ColorType origin;

		// extract all pixels (as indicated by the mask) from src_image
		// sum up the color components
		for ( unsigned int y = 0; y < src_image->height; y++ )
		{
			for ( unsigned int x = 0; x < src_image->width; x++ )
			{
				mask_pixel = opencv_utils::getIplPixel<unsigned char>( mask,
				                                                       x,
				                                                       y );
				if ( mask_pixel[0] < 255 / 2 ) continue;

				// get pixel data
				src_image_pixel = opencv_utils::getIplPixel<unsigned char>( src_image,
				                                                            x,
				                                                            y );

				_ColorType src_image_color( src_image_pixel );

				pixels_.push_back( src_image_color );

				for ( unsigned int i = 0; i < src_image->nChannels; ++i )
				{
					mean_.data_[i].data_ += origin.data_[i].distanceTo( src_image_color.data_[i], i == 0 ? 90.0 : 0.0 );
				}
			}
		}

		ROS_INFO( "Calculating statistics on %u pixels", pixels_.size() );

		// calculate the mean for each color component
		for ( unsigned int i = 0; i < src_image->nChannels; ++i )
		{
			mean_.data_[i].data_ /= pixels_.size();
		}
		//if( mean_.data_[0].data_ < 0 ) mean_.data_[0].data_ += 180;

		// calculate the raw variance for each color component
		for( _PixelList::iterator pixel_it = pixels_.begin(); pixel_it != pixels_.end(); ++pixel_it )
		{
			for ( unsigned int i = 0; i < src_image->nChannels; ++i )
			{
				FeatureBase<_ColorType::_DataType> color_rel_origin( origin.data_[i].distanceTo( pixel_it->data_[i], i == 0 ? 90.0 : 0.0 ) );
				variance_.data_[i].data_ += pow( mean_.data_[i].distanceTo( color_rel_origin, i == 0 ? 90.0 : 0.0 ), 2 );
			}
		}

		// calculate the variance for each color component
		for ( unsigned int i = 0; i < src_image->nChannels; ++i )
		{
			variance_.data_[i].data_ /= pixels_.size();
		}

		// ensure the mean is within [0,180] for easy readability
		if( mean_.data_[0].data_ < 0 ) mean_.data_[0].data_ += 180;
	}

	void spinOnce()
	{
		ROS_INFO( "Attempting to open source image: %s...", src_image_.c_str() );
		IplImage * src_image = cvLoadImage( src_image_.c_str() );
		if( src_image ) ROS_INFO( "Success." );
		else ROS_INFO( "Failed." );

		ROS_INFO( "Attempting to open mask image: %s", mask_image_.c_str() );
		IplImage * mask = cvLoadImage( mask_image_.c_str(), CV_LOAD_IMAGE_GRAYSCALE );
		if( mask ) ROS_INFO( "Success." );
		else ROS_INFO( "Failed." );

		//ROS_INFO( "Attempting to convert mask image to grayscale..." )
		//cvCvtColor( mask, mask, CV_BGR2GRAY );

		calculateStatistics( src_image, mask );

		double suggested_threshold = 1.0;

		// print statistics
		for ( unsigned int i = 0; i < src_image->nChannels; ++i )
		{
			printf( "----\n[%u]\n", i );
			printf( "mean: %.15f\n", mean_.data_[i].data_ );
			printf( "variance: %.15f\n", variance_.data_[i].data_ );

			suggested_threshold *= math_utils::gaussian( sqrt( variance_.data_[i].data_ ), variance_.data_[i].data_ );

			std::string current_param = color_name_ + "_" + channel_names_[i];
			printf("Setting param: %s\n", current_param.c_str() );
			current_param = color_name_ + "_" + channel_names_[i] + "_variance";
			printf("Setting param: %s\n", current_param.c_str() );

			nh_reconfig_.setParam( color_name_ + "_" + channel_names_[i], mean_.data_[i].data_ );
			nh_reconfig_.setParam( color_name_ + "_" + channel_names_[i] + "_variance", variance_.data_[i].data_ );
		}

		printf( "Suggested threshold: %.15f\n", suggested_threshold );

		interrupt();
	}

};

#endif /* TRAINER_H_ */
