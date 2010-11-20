/*******************************************************************************
 *
 *      color_segmenter
 * 
 *      Copyright (c) 2010
 *
 *      Mike Gerow
 *
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

#include <base_image_proc/base_image_proc.h>
#include <color_defs/colors.h>
#include <time.h>
#include <color_segmenter/FindBlobs.h>

typedef BaseImageProc<BaseNodeTypes::_DefaultReconfigureType, color_segmenter::FindBlobs> _BaseImageProc;

class ColorSegmenter: public _BaseImageProc
{
public:
	ColorSegmenter( ros::NodeHandle & nh ) :
		_BaseImageProc ( nh )
	{
		initCfgParams();

	}

	cv::Mat processImage( IplImage * ipl_img, _ServiceRequest & req, _ServiceResponse & resp )
	{
		//req.blob_descriptor.min_mass
		//req.blob_descriptor.color will be some number in ColorIds::*

		color_segmenter::ColorBlob blob;
		resp.blob_array.color_blobs.push_back( blob );

		cv_img_ = cv::Mat( ipl_img );

		cv::Size size;
		cv::Point center;
		long radius, time;
		cv::Scalar color;

		time = ( clock() * 10 ) / CLOCKS_PER_SEC;

		size = cv_img_.size();

		center.x = size.width / 2;
		center.y = size.height / 2;
		if ( size.width > size.height )
		{
			radius = size.height / 4;
		}
		else
		{
			radius = size.width / 4;
		}

		radius = radius / ( ( time % 10 ) + 1 );

		if ( ( time % 3 ) == 0 )
		{
			color = cv::Scalar( 0, 255, 0 );
		}
		else if ( ( time % 3 ) == 2 )
		{
			color = cv::Scalar( 0, 0, 255 );
		}
		else
		{
			color = cv::Scalar( 255, 0, 0 );
		}

		cv::circle( cv_img_, center, radius, color, 4 );
		cv::rectangle( cv_img_, cv::Point( 5, 5 ), cv::Point( 20, 20 ), cv::Scalar( 0, 255, 0 ) );
		cv::putText( cv_img_, ":)", center, 2, 1, cv::Scalar( 255, 0, 0 ) );
		//cv::line(cv_img_, cv::Point( 0, 0 ), cv::Point(50, 50), cv::Scalar( 255, 0, 0));

		return cv_img_;
	}

};

int main( int argc, char **argv )
{
	ros::init( argc, argv, "demo1_gerow" );
	ros::NodeHandle nh;

	ColorSegmenter color_segmenter( nh );
	color_segmenter.spin();

	return 0;
}
