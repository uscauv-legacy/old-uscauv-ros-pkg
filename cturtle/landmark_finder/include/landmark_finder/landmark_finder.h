/*******************************************************************************
 *
 *      landmark_finder
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
 *      * Neither the name of "landmark_finder-RelWithDebInfo@landmark_finder" nor the names of its
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

#ifndef LANDMARK_FINDER_H_
#define LANDMARK_FINDER_H_

#include <base_node/base_node.h>
#include <landmark_finder/FindLandmarks.h>
#include <color_blob_finder/FindColorBlobs.h>
#include <contour_matcher/MatchContours.h>

typedef BaseNode<> _BaseNode;
typedef landmark_finder::FindLandmarks _FindLandmarksService;
typedef color_blob_finder::FindColorBlobs _FindColorBlobsService;
typedef contour_matcher::MatchContours _MatchContoursService;

class LandmarkFinder: public _BaseNode
{
public:
	ros::ServiceServer find_landmarks_svr_;
	ros::ServiceClient find_color_blobs_cli_;
	ros::ServiceClient match_contours_cli_;

	LandmarkFinder( ros::NodeHandle & nh ) :
			_BaseNode( nh )
	{
		find_landmarks_svr_ = nh_local_.advertiseService( "find_landmarks",
		                                                  &LandmarkFinder::findLandmarksCB,
		                                                  this );

		find_color_blobs_cli_ = nh_local_.serviceClient<_FindColorBlobsService>( "find_color_blobs" );

		match_contours_cli_ = nh_local_.serviceClient<_MatchContoursService>( "match_contours" );
	}

	bool findLandmarksCB( _FindLandmarksService::Request & req,
	                       _FindLandmarksService::Response & resp )
	{
		_FindColorBlobsService::Request find_color_blobs_req;
		_FindColorBlobsService::Response find_color_blobs_resp;
		find_color_blobs_cli_.call( find_color_blobs_req, find_color_blobs_resp );

		_MatchContoursService::Request match_contours_req;
		_MatchContoursService::Response match_contours_resp;
		match_contours_cli_.call( match_contours_req, match_contours_resp );

		return true;
	}

	void spinOnce()
	{
		// do whatever
	}

};

#endif /* LANDMARK_FINDER_H_ */
