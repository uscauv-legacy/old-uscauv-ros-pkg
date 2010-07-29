/*******************************************************************************
 *
 *      seabee3_driver
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

#include <landmark_map/LandmarkMap.h>

LandmarkMap::LandmarkMap( cv::Point2d dim )
{
	mDim = dim;
}

LandmarkMap::LandmarkMap( std::vector<Landmark> landmarks, cv::Point2d dim )
{
	mLandmarks = landmarks;
	mDim = dim;
}

LandmarkMap::LandmarkMap( const localization_defs::LandmarkMapMsg & msg )
{
	mDim.x = msg.Dim.x;
	mDim.y = msg.Dim.y;
	
	mLandmarks.resize( msg.Map.LandmarkArray.size() );
	
	for(unsigned int i = 0; i < mLandmarks.size(); i ++)
	{
		mLandmarks[i] = Landmark::parseMessage( msg.Map.LandmarkArray[i] );
	}
}

LandmarkMap::LandmarkMap(Landmark l)
{
	addLandmark(l);
}

LandmarkMap::LandmarkMap(std::vector<Landmark> l)
{
	for(unsigned int i = 0; i < l.size(); i ++)
	{
		addLandmark(l[i]);
	}
}

void LandmarkMap::addLandmark(const Landmark l)
{
	mLandmarks.push_back(l);
}

std::vector<Landmark> LandmarkMap::fetchLandmarksByType(const int type) const
{
	std::vector<Landmark> landmarks;
	for(unsigned int i = 0; i < mLandmarks.size(); i ++)
	{
		if(mLandmarks[i].mLandmarkType == type)
		{
			landmarks.push_back(mLandmarks[i]);
		}
	}
	return landmarks;
}

std::vector<Landmark> LandmarkMap::fetchWaypointsByType(const int type) const
{
	std::vector<Landmark> waypoints = fetchLandmarksByType(Landmark::LandmarkType::Waypoint);
	std::vector<Landmark> filteredWaypoints;
	
	for(unsigned int i = 0; i < waypoints.size(); i ++)
	{
		if(waypoints[i].mId == type)
		{
			filteredWaypoints.push_back(waypoints[i]);
		}
	}
	return filteredWaypoints;
}

std::vector<visualization_msgs::Marker> LandmarkMap::createMarkerArray(const std::string & frame) const
{
	std::vector<visualization_msgs::Marker> markers;
	markers.resize( mLandmarks.size() );
	
	std::vector<int> typeCount;
	typeCount.resize(Landmark::LandmarkType::NumTypes);
	
	for(unsigned int i = 0; i < markers.size(); i ++)
	{
		markers[i] = mLandmarks[i].createMarker( frame, typeCount[mLandmarks[i].mLandmarkType] );
		typeCount[ mLandmarks[i].mLandmarkType ] ++;
	}
	
	return markers;
}

localization_defs::LandmarkMapMsg LandmarkMap::createMsg() const
{
	localization_defs::LandmarkMapMsg msg;
	for(unsigned int i = 0; i < mLandmarks.size(); i ++)
	{
		msg.Map.LandmarkArray.push_back( mLandmarks[i].createMsg() );
	}
	
	return msg;
}
	
