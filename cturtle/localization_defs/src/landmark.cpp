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

#include <landmark_map/landmark.h>

Landmark::Landmark( cv::Point3d center, double orientation, cv::Point3d dim, int shapeType )
{
	landmark_type_ = Landmark::LandmarkType::None;
	center_ = center;
	orientation_ = orientation;
	dim_ = dim;
	shape_type_ = shapeType;
	color_ = Landmark::ColorIds::black;
	id_ = -1;
}

visualization_msgs::Marker Landmark::createMarker( const std::string & frame, const int id, const std::string & ns_ext ) const
{
	visualization_msgs::Marker marker;
	
	marker.header.frame_id = frame.c_str();
	marker.header.stamp = ros::Time::now();
	
	std::stringstream ss;
	
	switch ( landmark_type_ )
	{
	case LandmarkType::None:
		ss << "landmark";
		break;
	case LandmarkType::Waypoint:
		ss << "waypoint";
		break;
	case LandmarkType::Buoy:
		ss << "buoy";
		break;
	case LandmarkType::Pinger:
		ss << "pinger";
		break;
	case LandmarkType::Pipe:
		ss << "pipe";
		break;
	case LandmarkType::Window:
		ss << "window";
		break;
	case LandmarkType::Bin:
		ss << "bin";
		break;
	}

	ss << ns_ext << id;
	
	marker.ns = ss.str();
	marker.id = 0;
	marker.type = shape_type_;
	marker.action = visualization_msgs::Marker::ADD;
	
	marker.pose.position.x = center_.x;
	marker.pose.position.y = center_.y;
	marker.pose.position.z = center_.z;
	
	tf::Quaternion fromEuler( MathyMath::degToRad( orientation_ ), 0, 0 );
	
	marker.pose.orientation.x = fromEuler.x();
	marker.pose.orientation.y = fromEuler.y();
	marker.pose.orientation.z = fromEuler.z();
	marker.pose.orientation.w = fromEuler.w();
	
	marker.scale.x = dim_.x;
	marker.scale.y = dim_.y;
	marker.scale.z = dim_.z;
	
	const cv::Vec3b & color = Landmark::ColorDefs::getColor( color_ );
	
	marker.color.r = ( (double) color[0] / 255.0 );
	marker.color.g = ( (double) color[1] / 255.0 );
	marker.color.b = ( (double) color[2] / 255.0 );
	marker.color.a = 1.0;
	
	marker.lifetime = ros::Duration();
	
	return marker;
}

localization_defs::LandmarkMsg Landmark::createMsg() const
{
	localization_defs::LandmarkMsg msg;
	
	msg.center.x = center_.x;
	msg.center.y = center_.y;
	msg.center.z = center_.z;
	
	msg.ori = orientation_;
	
	msg.color = color_;
	
	msg.type = landmark_type_;
	
	msg.id = id_;
	
	return msg;
}

Landmark Landmark::parseMessage( const localization_defs::LandmarkMsg & msg )
{
	if ( msg.type == Landmark::LandmarkType::Buoy )
	{
		return LandmarkTypes::Buoy( cv::Point3d( msg.center.x, msg.center.y, msg.center.z ), msg.ori, msg.color );
	}
	else if ( msg.type == Landmark::LandmarkType::Pinger )
	{
		return LandmarkTypes::Pinger( cv::Point3d( msg.center.x, msg.center.y, msg.center.z ), msg.ori, msg.id );
	}
	else if ( msg.type == Landmark::LandmarkType::Pipe )
	{
		return LandmarkTypes::Pipe( cv::Point3d( msg.center.x, msg.center.y, msg.center.z ), msg.ori );
	}
	return Landmark();
}

LandmarkTypes::Buoy::Buoy( cv::Point3d center, double orientation, int color ) :
	Landmark( center, orientation, cv::Point3d( 0.3048, 0.3048, 0.3048 ), visualization_msgs::Marker::SPHERE )
{
	color_ = color;
	landmark_type_ = Landmark::LandmarkType::Buoy;
}

/*visualization_msgs::Marker LandmarkTypes::Buoy::createMarker(std::string ns, std::string frame)
 {
 const visualization_msgs::Marker & theMarker = Landmark::createMarker(ns, frame);

 theMarker.type = visualization_msgs::Marker::SPHERE;

 return theMarker;
 }*/

LandmarkTypes::Pinger::Pinger( cv::Point3d center, double orientation, int id ) :
	Landmark( center, orientation, cv::Point3d( 0.0762, 0.0762, 0.1524 ), visualization_msgs::Marker::CYLINDER )
{
	id_ = id;
	color_ = Landmark::ColorIds::green;
	landmark_type_ = Landmark::LandmarkType::Pinger;
}

/*visualization_msgs::Marker LandmarkTypes::Pinger::createMarker(std::string ns, std::string frame)
 {
 const visualization_msgs::Marker & theMarker = Landmark::createMarker(ns, frame);

 theMarker.type = visualization_msgs::Marker::CUBE;

 return theMarker;
 }*/

LandmarkTypes::Bin::Bin( cv::Point3d center, double orientation, int id ) :
	Landmark( center, orientation, cv::Point3d( 0.6096, 0.9144, 0.0254 ), visualization_msgs::Marker::CUBE )
{
	id_ = id;
	color_ = Landmark::ColorIds::black;
	landmark_type_ = Landmark::LandmarkType::Bin;
}

/*visualization_msgs::Marker LandmarkTypes::Bin::createMarker(std::string ns, std::string frame)
 {
 const visualization_msgs::Marker & theMarker = Landmark::createMarker(ns, frame);

 theMarker.type = visualization_msgs::Marker::CUBE;

 return theMarker;
 }*/

LandmarkTypes::Pipe::Pipe( cv::Point3d center, double orientation ) :
	Landmark( center, orientation, cv::Point3d( 1.2192, 0.6096, 0.1524 ), visualization_msgs::Marker::CUBE )
{
	color_ = Landmark::ColorIds::orange;
	landmark_type_ = Landmark::LandmarkType::Pipe;
}

/*visualization_msgs::Marker LandmarkTypes::Pipe::createMarker(std::string ns, std::string frame)
 {
 const visualization_msgs::Marker & theMarker = Landmark::createMarker(ns, frame);

 theMarker.type = visualization_msgs::Marker::CUBE;

 return theMarker;
 }*/

LandmarkTypes::Waypoint::Waypoint( cv::Point3d center, double orientation, int id ) :
	Landmark( center, orientation, cv::Point3d( 1.0, 1.0, 1.0 ), visualization_msgs::Marker::ARROW )
{
	id_ = id;
	color_ = Landmark::ColorIds::green;
	landmark_type_ = Landmark::LandmarkType::Waypoint;
}

LandmarkTypes::Window::Window( cv::Point3d center, double orientation, int color ) :
	Landmark( center, orientation, cv::Point3d( 0.05, 0.61, 0.61 ), visualization_msgs::Marker::CUBE )
{
	color_ = color;
	landmark_type_ = Landmark::LandmarkType::Window;
}
