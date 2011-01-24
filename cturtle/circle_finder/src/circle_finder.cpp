/*******************************************************************************
 *
 *      circle_finder
 * 
 *      Copyright (c) 2011, toki303
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
#include <geometry_msgs/Point.h>
#include <opencv/highgui.h>
#include <vector>
#include <utility>
#include <cmath>

#include <circle_finder/CircleArrayMsg.h>
#include <circle_finder/FindCircles.h>

typedef BaseNodeTypes::_DefaultReconfigureType _ReconfigureType;
typedef circle_finder::FindCircles _ServiceType;

class CircleFinder: public BaseImageProc<_ReconfigureType, _ServiceType>
{

public:
	CircleFinder( ros::NodeHandle & nh ) :
		BaseImageProc<_ReconfigureType, _ServiceType> ( nh, "find_circles" )
	{

	}

	circle_finder::CircleArrayMsg findCircles (IplImage * ipl_img, _ServiceRequest & req)
	{
		circle_finder::CircleArrayMsg circles;

		return circles;
	}

	cv::Mat processImage( IplImage * ipl_img, _ServiceRequest & req, _ServiceResponse & resp )
		{
			resp.circles = findCircles( ipl_img, req );
			return cv_img_;
		}
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "circle_finder" );
	ros::NodeHandle nh;

	CircleFinder circle_finder( nh );
	circle_finder.spin();

	return 0;
}

