/*******************************************************************************
 *
 *      Landmark
 * 
 *      Copyright (c) 2010, Edward T. Kaszubski (ekaszubski@gmail.com)
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

#include <landmark_map/Landmark.h>
#include <tf/tf.h>
#include <sstream>
#include <localization_tools/Util.h>

Landmark::Landmark(cv::Point3d center, double orientation, cv::Point3d dim, int shapeType)
{
	mLandmarkType = Landmark::LandmarkType::None;
	mCenter = center;
	mOrientation = orientation;
	mDim = dim;
	mShapeType = shapeType;
	mColor = Landmark::ColorIds::black;
	mId = -1;
}

visualization_msgs::Marker Landmark::createMarker(const std::string & frame, const int id, const std::string & ns_ext) const
{
	visualization_msgs::Marker marker;
	
	marker.header.frame_id = frame.c_str();
	marker.header.stamp = ros::Time::now();
	
	std::stringstream ss;
	
	switch (mLandmarkType)
	{
		case LandmarkType::None: ss << "landmark"; break;
		case LandmarkType::Waypoint: ss << "waypoint"; break;
		case LandmarkType::Buoy: ss << "buoy"; break;
		case LandmarkType::Pinger: ss << "pinger"; break;
		case LandmarkType::Pipe: ss << "pipe"; break;
		case LandmarkType::Window: ss << "window"; break;
		case LandmarkType::Bin: ss << "bin"; break;
	}
	
	ss << ns_ext << id;
	
	marker.ns = ss.str();
	marker.id = 0;
	marker.type = mShapeType;
	marker.action = visualization_msgs::Marker::ADD;
	
	marker.pose.position.x = mCenter.x;
	marker.pose.position.y = mCenter.y;
	marker.pose.position.z = mCenter.z;
	
	tf::Quaternion fromEuler ( LocalizationUtil::degToRad( mOrientation ) , 0, 0);
	
	marker.pose.orientation.x = fromEuler.x();
	marker.pose.orientation.y = fromEuler.y();
	marker.pose.orientation.z = fromEuler.z();
	marker.pose.orientation.w = fromEuler.w();
	
	marker.scale.x = mDim.x;
	marker.scale.y = mDim.y;
	marker.scale.z = mDim.z;
	
	const cv::Vec3b & color = Landmark::ColorDefs::getColor( mColor );
	
	marker.color.r = ((double)color[0] / 255.0);
	marker.color.g = ((double)color[1] / 255.0);
	marker.color.b = ((double)color[2] / 255.0);
	marker.color.a = 1.0;
	
	marker.lifetime = ros::Duration();
	
	return marker;
}

localization_defs::LandmarkMsg Landmark::createMsg() const
{
	localization_defs::LandmarkMsg msg;
	
	msg.Center.x = mCenter.x;
	msg.Center.y = mCenter.y;
	msg.Center.z = mCenter.z;
	
	msg.Ori = mOrientation;
	
	msg.Color = mColor;
	
	msg.Type = mLandmarkType;
	
	msg.Id = mId;
	
	return msg;
}

Landmark Landmark::parseMessage( const localization_defs::LandmarkMsg & msg )
{
	if(msg.Type == Landmark::LandmarkType::Buoy)
	{
		return LandmarkTypes::Buoy( cv::Point3d( msg.Center.x, msg.Center.y, msg.Center.z ), msg.Ori, msg.Color );
	}
	else if(msg.Type == Landmark::LandmarkType::Pinger)
	{
		return LandmarkTypes::Pinger( cv::Point3d( msg.Center.x, msg.Center.y, msg.Center.z), msg.Ori, msg.Id );
	}
	else if(msg.Type == Landmark::LandmarkType::Pipe)
	{
		return LandmarkTypes::Pipe( cv::Point3d( msg.Center.x, msg.Center.y, msg.Center.z), msg.Ori );
	}
	return Landmark();
}

LandmarkTypes::Buoy::Buoy(cv::Point3d center, double orientation, int color) : 
Landmark(center, orientation, cv::Point3d(0.3048, 0.3048, 0.3048), visualization_msgs::Marker::SPHERE)
{
	mColor = color;
	mLandmarkType = Landmark::LandmarkType::Buoy;
}

/*visualization_msgs::Marker LandmarkTypes::Buoy::createMarker(std::string ns, std::string frame)
{
	const visualization_msgs::Marker & theMarker = Landmark::createMarker(ns, frame);
	
	theMarker.type = visualization_msgs::Marker::SPHERE;
	
	return theMarker;
}*/

LandmarkTypes::Pinger::Pinger(cv::Point3d center, double orientation, int id) : 
Landmark(center, orientation, cv::Point3d(0.0762, 0.0762, 0.1524), visualization_msgs::Marker::CYLINDER)
{
	mId = id;
	mColor = Landmark::ColorIds::green;
	mLandmarkType = Landmark::LandmarkType::Pinger;
}

/*visualization_msgs::Marker LandmarkTypes::Pinger::createMarker(std::string ns, std::string frame)
{
	const visualization_msgs::Marker & theMarker = Landmark::createMarker(ns, frame);
	
	theMarker.type = visualization_msgs::Marker::CUBE;
	
	return theMarker;
}*/

LandmarkTypes::Bin::Bin(cv::Point3d center, double orientation, int id) : 
Landmark(center, orientation, cv::Point3d(0.6096, 0.9144, 0.0254), visualization_msgs::Marker::CUBE)
{
  mId = id;
  mColor = Landmark::ColorIds::black;
  mLandmarkType = Landmark::LandmarkType::Bin;
}

/*visualization_msgs::Marker LandmarkTypes::Bin::createMarker(std::string ns, std::string frame)
{
	const visualization_msgs::Marker & theMarker = Landmark::createMarker(ns, frame);
	
	theMarker.type = visualization_msgs::Marker::CUBE;
	
	return theMarker;
}*/


LandmarkTypes::Pipe::Pipe(cv::Point3d center, double orientation) : 
Landmark(center, orientation, cv::Point3d(1.2192, 0.6096, 0.1524), visualization_msgs::Marker::CUBE)
{
	mColor = Landmark::ColorIds::orange;
	mLandmarkType = Landmark::LandmarkType::Pipe;
}

/*visualization_msgs::Marker LandmarkTypes::Pipe::createMarker(std::string ns, std::string frame)
{
	const visualization_msgs::Marker & theMarker = Landmark::createMarker(ns, frame);
	
	theMarker.type = visualization_msgs::Marker::CUBE;
	
	return theMarker;
}*/

LandmarkTypes::Waypoint::Waypoint(cv::Point3d center, double orientation, int id) : 
Landmark(center, orientation, cv::Point3d(1.0, 1.0, 1.0), visualization_msgs::Marker::ARROW)
{
	mId = id;
	mColor = Landmark::ColorIds::green;
	mLandmarkType = Landmark::LandmarkType::Waypoint;
}

LandmarkTypes::Window::Window(cv::Point3d center, double orientation, int color) : 
Landmark(center, orientation, cv::Point3d(0.05, 0.61, 0.61), visualization_msgs::Marker::CUBE)
{
	mColor = color;
	mLandmarkType = Landmark::LandmarkType::Window;
}
