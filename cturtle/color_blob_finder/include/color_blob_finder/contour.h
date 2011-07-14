/*******************************************************************************
 *
 *      contour
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
 *      * Neither the name of "color_blob_finder-RelWithDebInfo@color_blob_finder" nor the names of its
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

#ifndef CONTOUR_H_
#define CONTOUR_H_

#include <vector>
#include <opencv/cv.h>
#include <cxcore.h>

/* msgs */
#include <color_blob_finder/Contour.h>

typedef unsigned int _DimType;
typedef std::vector<cv::Point> _Contour;
typedef color_blob_finder::Contour _ContourMessage;
typedef color_blob_finder::Point2D _PointMsgType;
typedef cv::Point _PointType;

void operator<<( _Contour & contour,
                 _ContourMessage & contour_msg )
{
	contour.reserve( contour_msg.points.size() );
	for ( _DimType i = 0; i < contour_msg.points.size(); ++i )
	{
		_PointType point;

		point.x = contour_msg.points[i].x;
		point.y = contour_msg.points[i].y;

		contour.push_back( point );
	}
}

void operator<<( _ContourMessage & contour_msg,
                 _Contour & contour )
{
	contour_msg.points.reserve( contour.size() );

	for ( _DimType i = 0; i < contour.size(); ++i )
	{
		_PointMsgType point;

		point.x = contour[i].x;
		point.y = contour[i].y;

		contour_msg.points.push_back( point );
	}
}

static void findContours( cv::Mat & image, std::vector<_Contour> & contours )
{
		cv::findContours( image,
		                  contours,
		                  CV_RETR_LIST,
		                  CV_CHAIN_APPROX_SIMPLE );
}

#endif /* CONTOUR_H_ */
