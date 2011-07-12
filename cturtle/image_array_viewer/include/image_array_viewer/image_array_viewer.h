/*******************************************************************************
 *
 *      image_array_viewer
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
 *      * Neither the name of "image_array_viewer-RelWithDebInfo@image_array_viewer" nor the names of its
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

#ifndef IMAGE_ARRAY_VIEWER_H_
#define IMAGE_ARRAY_VIEWER_H_

#include <base_node/base_node.h>
#include <base_libs/ComponentImageArray.h>
#include <common_utils/opencv.h>
#include <common_utils/math.h>

typedef BaseNode<> _BaseNode;
typedef unsigned int _DimType;
typedef base_libs::ComponentImageArray _ImageArrayMsgType;

class ImageArrayViewer : public _BaseNode
{
public:
	ros::Subscriber image_array_sub_;

	ImageArrayViewer( ros::NodeHandle & nh ) : _BaseNode( nh )
	{
		image_array_sub_ = nh_local_.subscribe( "images", 1, &ImageArrayViewer::imageArrayCB, this );
		cv::namedWindow( "output", 0 );
	}

	void imageArrayCB( const _ImageArrayMsgType::ConstPtr & image_array_msg )
	{
		printf( "Got %zu images\n", image_array_msg->images.size() );
		int rows = floor( sqrt( image_array_msg->images.size() ) );
		int cols = ceil( sqrt( image_array_msg->images.size() ) );

		if( rows * cols < image_array_msg->images.size() ) ++rows;

		IplImage * display_image = NULL;

		for( _DimType i = 0; i < image_array_msg->images.size(); ++i )
		{
			sensor_msgs::CvBridge cv_bridge;
			cv_bridge.fromImage( image_array_msg->images[i].image );

			const IplImage * current_image_ptr = cv_bridge.toIpl();

			if( !display_image )
			{
				display_image = cvCreateImage( cvSize( cols * current_image_ptr->width, rows * current_image_ptr->height ), IPL_DEPTH_8U, 3 );
				cvSet( display_image, cvScalar( 127, 127, 127 ) );
			}

			//printf( "%d %d %d %d\n", int( current_image_ptr->width * ( i % cols ) ), int( current_image_ptr->height * floor( i / cols ) ), current_image_ptr->width, current_image_ptr->height );

			cvSetImageROI( display_image, cvRect( int( current_image_ptr->width * ( i % cols ) ), int( current_image_ptr->height * floor( i / cols ) ), current_image_ptr->width, current_image_ptr->height ) );

			IplImage * current_image_rgb = cvCreateImage( cvGetSize( current_image_ptr ), IPL_DEPTH_8U, 3 );
			cvConvertImage( current_image_ptr, current_image_rgb );

			cvCopy( current_image_rgb, display_image );

			cvReleaseImage( &current_image_rgb );
		}

		printf( "%d %d\n", display_image->width, display_image->height );

		cvResetImageROI( display_image );
		cv::imshow( "output", display_image );
		cvWaitKey( 20 );
		cvReleaseImage( &display_image );
	}
};

#endif /* IMAGE_ARRAY_VIEWER_H_ */
