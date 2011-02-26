/*******************************************************************************
 *
 *      image_loader
 * 
 *      Copyright (c) 2011, edward
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

#include <image_loader/image_loader.h>

ImageLoader::ImageLoader( ros::NodeHandle & nh ) : images_loaded_( false )
{
	nh.param( "prefix", file_prefix_, std::string( "image" ) );
	nh.param( "start", start_, 0 );
	nh.param( "end", end_, 0 );
	nh.param( "digits", digits_, 1 );
	nh.param( "ext", file_ext_, std::string( ".png" ) );

	ROS_INFO("constructed");
}

std::vector<IplImage *> ImageLoader::loadImages()
{
	ROS_INFO("Loading images %s %d %d %d %s", file_prefix_.c_str(), start_, end_, digits_, file_ext_.c_str() );
	cv::Mat img;
	for ( int i = start_; i < end_; i++ )
	{
		std::stringstream filename;
		filename << file_prefix_ << std::setfill( '0' ) << std::setw( digits_ ) << i << file_ext_;

		img = cv::imread( filename.str().c_str() );

		if ( img.data != NULL && img.size().width * img.size().height > 0 )
		{
			ROS_INFO( "Opening %s %dx%d", filename.str().c_str(), img.size().width, img.size().height );
			IplImage * ipl_img = new IplImage( img );
			image_cache_.push_back( ipl_img );
		}
		else
		{
			ROS_WARN( "Ignoring %s; does not exist in filesystem", filename.str().c_str() );
			end_--;
			i--;
		}
	}

	images_loaded_ = true;

	ROS_INFO("done");

	return image_cache_;
}
