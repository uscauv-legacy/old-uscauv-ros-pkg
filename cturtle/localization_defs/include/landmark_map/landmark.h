/*******************************************************************************
 *
 *      landmark
 * 
 *      Copyright (c) 2010,
 *
 *      Edward T. Kaszubski (ekaszubski@gmail.com)
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
 *      * Neither the name of the USC Underwater Robotics Team nor the names of its
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

#ifndef LANDMARK_H_
#define LANDMARK_H_

#include <opencv/cv.h>
#include <visualization_msgs/Marker.h>
#include <localization_defs/Landmark.h>
#include <common_utils/math.h>
#include <string>
#include <common_utils/tf.h>
#include <sstream>

class Landmark
{
public:
	
	struct LandmarkType
	{
		const static int NumTypes = 5;

		const static int None = -1;
		const static int Buoy = 0;
		const static int Pinger = 1;
		const static int Pipe = 2;
		const static int Bin = 3;
		const static int Window = 4;
		const static int Waypoint = 5;
		const static int Gate = 6;
	};

	struct ImageIds
	{
		const static int axe = 0;
		const static int clippers = 1;
		const static int hammer = 2;
		const static int machete = 3;
	};

	struct ColorIds
	{
		const static int red = 0;
		const static int orange = 1;
		const static int yellow = 2;
		const static int green = 3;
		const static int blue = 4;

		const static int lred = 5;
		const static int lrgreen = 6;
		const static int lgreen = 7;
		const static int lbgreen = 8;
		const static int blugreen = 9;
		const static int lgblue = 10;
		const static int lblue = 11;
		const static int lrblue = 12;
		const static int purple = 13;
		const static int pink = 14;
		const static int black = 15;
	};

	struct ColorDefs
	{
		static cv::Vec3b getColor( int colorId )
		{
			switch ( colorId )
			{
			case ColorIds::red:
				return cv::Vec3b( 255, 0, 0 );
			case ColorIds::lred:
				return cv::Vec3b( 255, 127, 127 );
			case ColorIds::orange:
				return cv::Vec3b( 255, 127, 0 );
			case ColorIds::yellow:
				return cv::Vec3b( 255, 255, 0 );
			case ColorIds::lrgreen:
				return cv::Vec3b( 127, 255, 0 );
			case ColorIds::green:
				return cv::Vec3b( 0, 255, 0 );
			case ColorIds::lgreen:
				return cv::Vec3b( 127, 255, 127 );
			case ColorIds::lbgreen:
				return cv::Vec3b( 0, 255, 127 );
			case ColorIds::blugreen:
				return cv::Vec3b( 0, 255, 255 );
			case ColorIds::lgblue:
				return cv::Vec3b( 0, 127, 255 );
			case ColorIds::blue:
				return cv::Vec3b( 0, 0, 255 );
			case ColorIds::lblue:
				return cv::Vec3b( 127, 127, 255 );
			case ColorIds::lrblue:
				return cv::Vec3b( 127, 0, 255 );
			case ColorIds::purple:
				return cv::Vec3b( 255, 0, 255 );
			case ColorIds::pink:
				return cv::Vec3b( 255, 0, 127 );
			case ColorIds::black:
				return cv::Vec3b( 0, 0, 0 );
			}
			return cv::Vec3b( 0, 0, 0 );
		}
	};

	Landmark( cv::Point3d center = cv::Point3d( 0.0, 0.0, 0.0 ), double orientation = 0.0, cv::Point3d dim = cv::Point3d( 1.0, 1.0, 1.0 ), int shapeType = visualization_msgs::Marker::ARROW );

	static Landmark parseMessage( const localization_defs::Landmark & msg );

	visualization_msgs::Marker createMarker( const std::string & frame, const int id, const std::string & ns_ext = "" ) const;
	localization_defs::Landmark createMsg() const;

	cv::Point3d center_; //position relative to the center of the map (which is always the origin, <0, 0, 0>)
	double orientation_;
	cv::Point3d dim_;
	int color_;

	int shape_type_; //how to draw this object
	int landmark_type_;
	int id_;
};

namespace LandmarkTypes
{
	class Buoy: public Landmark
	{
	public:
		Buoy( cv::Point3d center, double orientation, int color );
		//		visualization_msgs::Marker createMarker(std::string ns, std::string frame);
	};

	class Pinger: public Landmark
	{
	public:
		Pinger( cv::Point3d center, double orientation, int id );
		//		visualization_msgs::Marker createMarker(std::string ns, std::string frame);
	};

	class Bin: public Landmark
	{
	public:
		Bin( cv::Point3d center, double orientation, int id );
		//		visualization_msgs::Marker createMarker(std::string ns, std::string frame);
	};

	class Pipe: public Landmark
	{
	public:
		Pipe( cv::Point3d center, double orientation );
		//		visualization_msgs::Marker createMarker(std::string ns, std::string frame);
	};
	
	class Window: public Landmark
	{
	public:
		Window( cv::Point3d center, double orientation, int color );
	};
	
	class Waypoint: public Landmark
	{
	public:
		Waypoint( cv::Point3d center, double orientation, int id );
	};
}

#endif /* LANDMARK_H_ */
