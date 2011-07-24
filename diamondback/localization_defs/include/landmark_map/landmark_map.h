/*******************************************************************************
 *
 *      LandmarkMapServer
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

#ifndef LANDMARK_MAP_H_
#define LANDMARK_MAP_H_

#include <vector>
#include <landmark_map/landmark.h>
#include <localization_defs/LandmarkMap.h>
#include <opencv/cv.h>

class LandmarkMap
{
public:
	LandmarkMap( cv::Point2d dim = cv::Point2d( 0.0, 0.0 ) );
	LandmarkMap( std::vector<Landmark> landmarks, cv::Point2d dim );
	LandmarkMap( const localization_defs::LandmarkMap & msg );
	LandmarkMap( Landmark l );
	LandmarkMap( std::vector<Landmark> l );

	void addLandmark( const Landmark l );
	std::vector<Landmark> fetchLandmarksByType( const int type ) const;
	std::vector<Landmark> fetchWaypointsByType( const int type ) const;
	std::vector<visualization_msgs::Marker> createMarkerArray( const std::string & frame ) const;
	localization_defs::LandmarkMap createMsg() const;

	std::vector<Landmark> landmarks_;
	cv::Point2d dim_;
};

#endif /* LANDMARK_MAP_H_ */
