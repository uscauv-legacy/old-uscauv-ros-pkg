/*******************************************************************************
 *
 *      image_loader_nodelet
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

#include <vector>
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
// for ImageTransport, Publisher
#include <image_transport/image_transport.h>
// for Mat
#include <opencv/cv.h>
#include <cxcore.h>
// for image utilities like imread()
#include "highgui.h"
//for CvBridge
#include <cv_bridge/CvBridge.h>

namespace base_libs
{
	class ImageLoader : public nodelet::Nodelet
	{
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nh_local_;

	public:
		typedef std::vector<IplImage *> _ImageCache;

		_ImageCache image_cache_;
		bool images_loaded_;

		std::string file_prefix_, file_ext_;
		int start_, end_, digits_;

		ImageLoader() :
			nodelet::Nodelet(), images_loaded_( false )
		{
			nh_local_.param( "prefix", file_prefix_, std::string( "image" ) );
			nh_local_.param( "start", start_, 0 );
			nh_local_.param( "end", end_, 0 );
			nh_local_.param( "digits", digits_, 1 );
			nh_local_.param( "ext", file_ext_, std::string( ".png" ) );

			ROS_INFO( "constructed" );
		}

		void onInit()
		{
			nh_ = getNodeHandle();
			nh_local_ = getPrivateNodeHandle();
		}

		_ImageCache loadImages()
		{
			ROS_INFO( "Loading images %s %d %d %d %s", file_prefix_.c_str(), start_, end_, digits_, file_ext_.c_str() );
			IplImage * img = NULL;
			for ( int i = start_; i < end_; i++ )
			{
				std::stringstream filename;
				filename << file_prefix_ << std::setfill( '0' ) << std::setw( digits_ ) << i << file_ext_;

				img = cvLoadImage( filename.str().c_str() );

				if ( img && img->width * img->height > 0 )
				{
					ROS_INFO( "Opening %s %dx%d", filename.str().c_str(), img->width, img->height );
					image_cache_.push_back( img );
				}
				else
				{
					ROS_WARN( "Ignoring %s; does not exist in filesystem", filename.str().c_str() );
				}
			}

			images_loaded_ = true;

			ROS_INFO( "done" );

			return image_cache_;
		}
	};
}

// Register the nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(base_libs, image_loader, base_libs::ImageLoader, nodelet::Nodelet)
