/*******************************************************************************
 *
 *      image_server
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
 *      * Neither the name of "image_server-RelWithDebInfo@image_server" nor the names of its
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

#ifndef IMAGE_SERVER_H_
#define IMAGE_SERVER_H_

#include <base_image_proc/base_image_proc.h>
#include <image_loader/image_loader.h>

class ImageServer : public BaseImageProc<>
{
private:
	ImageLoader image_loader_;

	bool loop_;

public:
	double rate_;

	ImageServer( ros::NodeHandle & nh ) :
		BaseImageProc<> ( nh ), image_loader_( nh_local_ )
	{
		nh_local_.param( "rate", rate_, 15.0 );
		nh_local_.param( "loop", loop_, false );

		image_loader_.loadImages();
	}

	virtual ~ImageServer()
	{
	}

	void spinOnce()
	{
		static int current_frame = 0;

		if ( image_loader_.images_loaded_ && current_frame < image_loader_.image_cache_.size() )
		{
			ROS_INFO( "Publishing image %d", current_frame );
			publishCvImage( image_loader_.image_cache_[current_frame] );
			++current_frame;
		}
		else if( loop_ )
		{
			current_frame = 0;
		}
	}
};

#endif /* IMAGE_SERVER_H_ */
