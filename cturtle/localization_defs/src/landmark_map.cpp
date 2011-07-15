/*******************************************************************************
 *
 *      landmark_map
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

#include <landmark_map/landmark_map.h>

LandmarkMap::LandmarkMap( cv::Point2d dim )
{
	dim_ = dim;
}

LandmarkMap::LandmarkMap( std::vector<Landmark> landmarks, cv::Point2d dim )
{
	landmarks_ = landmarks;
	dim_ = dim;
}

LandmarkMap::LandmarkMap( const localization_defs::LandmarkMap & msg )
{
	dim_.x = msg.dim.x;
	dim_.y = msg.dim.y;
	
	landmarks_.resize( msg.map.size() );
	
	for ( unsigned int i = 0; i < landmarks_.size(); i++ )
	{
		landmarks_[i] = Landmark::parseMessage( msg.map[i] );
	}
}

LandmarkMap::LandmarkMap( Landmark l )
{
	addLandmark( l );
}

LandmarkMap::LandmarkMap( std::vector<Landmark> l )
{
	for ( unsigned int i = 0; i < l.size(); i++ )
	{
		addLandmark( l[i] );
	}
}

void LandmarkMap::addLandmark( const Landmark l )
{
	landmarks_.push_back( l );
}

std::vector<Landmark> LandmarkMap::fetchLandmarksByType( const int type ) const
{
	std::vector<Landmark> landmarks;
	for ( unsigned int i = 0; i < landmarks_.size(); i++ )
	{
		if ( landmarks_[i].landmark_type_ == type )
		{
			landmarks.push_back( landmarks_[i] );
		}
	}
	return landmarks;
}

std::vector<Landmark> LandmarkMap::fetchWaypointsByType( const int type ) const
{
	std::vector<Landmark> waypoints = fetchLandmarksByType( Landmark::LandmarkType::Waypoint );
	std::vector<Landmark> filteredWaypoints;
	
	for ( unsigned int i = 0; i < waypoints.size(); i++ )
	{
		if ( waypoints[i].id_ == type )
		{
			filteredWaypoints.push_back( waypoints[i] );
		}
	}
	return filteredWaypoints;
}

std::vector<visualization_msgs::Marker> LandmarkMap::createMarkerArray( const std::string & frame ) const
{
	std::vector<visualization_msgs::Marker> markers;
	markers.resize( landmarks_.size() );
	
	std::vector<int> typeCount;
	typeCount.resize( Landmark::LandmarkType::NumTypes );
	
	for ( unsigned int i = 0; i < markers.size(); i++ )
	{
		markers[i] = landmarks_[i].createMarker( frame, typeCount[landmarks_[i].landmark_type_] );
		typeCount[landmarks_[i].landmark_type_]++;
	}
	
	return markers;
}

localization_defs::LandmarkMap LandmarkMap::createMsg() const
{
	localization_defs::LandmarkMap msg;
	for ( unsigned int i = 0; i < landmarks_.size(); i++ )
	{
		msg.map.push_back( landmarks_[i].createMsg() );
	}
	
	return msg;
}

