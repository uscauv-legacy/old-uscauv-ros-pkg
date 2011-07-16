/*******************************************************************************
 *
 *      image_scaler
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

#ifndef IMAGE_SCALER_H_
#define IMAGE_SCALER_H_

#include <base_image_proc/base_image_proc.h>

typedef BaseImageProc<> _BaseImageProc;

class ImageScaler: public _BaseImageProc
{
public:
	int width_;
	int height_;

	IplImage * scaled_img_;
	ros::Publisher camera_info_pub_;

	ImageScaler( ros::NodeHandle & nh ) :
			_BaseImageProc( nh ), scaled_img_( NULL )
	{
		nh_local_.param( "width", width_, 0 );
		nh_local_.param( "height", height_, 0 );

		camera_info_pub_ = nh_local_.advertise<sensor_msgs::CameraInfo>( "output_camera_info", 1 );
	}

	IplImage * processImage( IplImage * img )
	{
		if ( img && img->width * img->height > 0 )
		{
			if ( width_ > 0 || height_ > 0 )
			{
				double width = width_ > 0 ? width_ :
				                            img->width;
				double height = height_ > 0 ? height_ :
				                              img->height;
				// scale image, maintain aspect ratio
				cv::Point2d scales( width / img->width,
				                    height / img->height );
				double scale = img->width * scales.x <= width && img->height * scales.x <= height ? scales.x :
				                                                                                    scales.y;

				if( !scaled_img_ )
				{
					scaled_img_ = cvCreateImage( cvSize( img->width * scale,
				                                         img->height * scale ),
				                                         img->depth,
				                                         img->nChannels );
				}


				camera_info_msg_.width = scaled_img_->width;
				camera_info_msg_.height = scaled_img_->height;
			
				sensor_msgs::CameraInfo::Ptr camera_info_msg( new sensor_msgs::CameraInfo( camera_info_msg_ ) );
				camera_info_pub_.publish( camera_info_msg );


				ROS_INFO( "Scaling image to %dx%d",
				          scaled_img_->width,
				          scaled_img_->height );
				cvResize( img,
				          scaled_img_,
				          CV_INTER_CUBIC );

//				cvReleaseImage(&img);
				return scaled_img_;
			}
		}

		return img;
	}
};

#endif /* IMAGE_SCALER_H_ */
