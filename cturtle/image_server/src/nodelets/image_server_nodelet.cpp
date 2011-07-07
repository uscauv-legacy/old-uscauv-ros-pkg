/*******************************************************************************
 *
 *      image_server_nodelet
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

#include <base_nodelet/base_nodelet.h>
#include <image_server/image_server.h>
#include <thread>

typedef BaseNodelet<ImageServer> _BaseNodelet;

namespace image_server
{
	class ImageServerNodelet : public _BaseNodelet
	{
	public:
		ImageServerNodelet() :
			_BaseNodelet()
		{
			registerInterrupt( &ImageServerNodelet::interrupt, this );
		}

		void constructData()
		{
			if( data_ ) delete data_;
			data_ = new ImageServer( nh_local_ );
		}

		void spin()
		{
			if( data_ ) data_->spin();
		}

		void interrupt()
		{
			ROS_INFO( "image_server_nodelet interrupted" );
			data_->interrupt();
		}
	};

}

// Register the nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(image_server, image_server_nodelet, image_server::ImageServerNodelet, nodelet::Nodelet)
