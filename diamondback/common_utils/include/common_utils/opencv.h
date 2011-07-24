/*******************************************************************************
 *
 *      opencv_utils
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
 *      * Neither the name of "interaction-ros-pkg" nor the names of its
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

#ifndef OPENCV_UTILS_H_
#define OPENCV_UTILS_H_

#include <opencv/cv.h>
#include <cxcore.h>
#include "highgui.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>
// for ImageTransport, Publisher
#include <image_transport/image_transport.h>

namespace opencv_utils
{
	// _DataType * pixel = getIplPixel( image, y, x );
	// returns a reference to the pixel at (x, y) in image
	// access channels in the pixel via pixel[channel]
	template<class _DataType>
	static _DataType * getIplPixel( IplImage * image, const unsigned int & x, const unsigned int & y )
	{
		return (_DataType *) ( image->imageData + y * image->widthStep + image->nChannels * x * image->depth / 8 );
	}

	template<class _DataType>
	static _DataType * getIplPixel( IplImage * image, const cv::Point & pixel )
	{
		return getIplPixel<_DataType> ( image, pixel.x, pixel.y );
	}

	static void publishCvImage( const IplImage * ipl_img, image_transport::Publisher * img_pub, sensor_msgs::CvBridge * bridge = NULL )
	{
		sensor_msgs::CvBridge * temp_bridge = new sensor_msgs::CvBridge();
		if ( !bridge ) bridge = temp_bridge;

		sensor_msgs::Image::Ptr image_message;
		image_message = bridge->cvToImgMsg( ipl_img );
		image_message->header.stamp = ros::Time::now();

		img_pub->publish( image_message );
	}

	static void publishCvImage( const cv::Mat & img, image_transport::Publisher * img_pub, sensor_msgs::CvBridge * bridge = NULL )
	{
		static IplImage * ipl_img = NULL;
		ipl_img = new IplImage( img );

		publishCvImage( ipl_img, img_pub, bridge );
	}

	class ImagePublisher
	{
	public:
		IplImage * image_;
		sensor_msgs::CvBridge bridge_;
		image_transport::Publisher publisher_;

		ImagePublisher( IplImage * image = NULL, image_transport::Publisher publisher = image_transport::Publisher() ) :
			image_( image ), publisher_( publisher )
		{
			//
		}

		~ImagePublisher()
		{
			if( image_ ) cvReleaseImage( &image_ );
		}

		void publish()
		{
			opencv_utils::publishCvImage( image_, &publisher_, &bridge_ );
		}
	};
}

#endif /* OPENCV_UTILS_H_ */
