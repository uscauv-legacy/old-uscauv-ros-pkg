/***************************************************************************
 *  include/contour_matcher/contour.h
 *  --------------------
 *
 *  Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer
 *    in the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of seabee3-ros-pkg nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************/

#ifndef CONTOURMATCHER_CONTOUR_H_
#define CONTOURMATCHER_CONTOUR_H_

#include <opencv/cv.hpp>
#include <vector>
#include <seabee3_msgs/Contour.h>
#include <quickdev/unit.h>

typedef cv::Point _Point;
typedef seabee3_msgs::Point2D _ContourPointMsg;
typedef std::vector<_Point> _Contour;
typedef seabee3_msgs::Contour _ContourMsg;

DECLARE_UNIT_CONVERSION_LAMBDA( _Contour, _ContourMsg, contour, _ContourMsg contour_msg; for( auto point_it = contour.cbegin(); point_it != contour.cend(); ++point_it ){ _ContourPointMsg point_msg; point_msg.x = point_it->x; point_msg.y = point_it->y; contour_msg.points.push_back( point_msg ); } return contour_msg; )
DECLARE_UNIT_CONVERSION_LAMBDA( _ContourMsg, _Contour, contour_msg, _Contour contour; for( auto point_it = contour_msg.points.cbegin(); point_it != contour_msg.points.cend(); ++point_it ){ contour.push_back( _Point( point_it->x, point_it->y ) ); } return contour; )

#endif // CONTOURMATCHER_CONTOUR_H_
