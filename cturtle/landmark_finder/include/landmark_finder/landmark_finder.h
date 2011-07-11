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
#include <color_blob_finder/FindColorBlobs.h>
#include <contour_matcher/MatchContours.h>
#include <localization_defs/LandmarkArray.h>
//#include <localization_defs/landmark_map.h>
#include <landmark_finder/SetEnabledLandmarks.h>
#include <color_blob_finder/contour.h>
#include <array>
#include <landmark_finder/LandmarkFinderConfig.h>

typedef unsigned int _DimType;

const static _DimType NUM_LANDMARKS = 1;

typedef landmark_finder::LandmarkFinderConfig _ReconfigureType;
typedef BaseNode<_ReconfigureType> _BaseNode;
typedef landmark_finder::SetEnabledLandmarks _SetEnabledLandmarksService;
typedef color_blob_finder::FindColorBlobs _FindColorBlobsService;
typedef contour_matcher::MatchContours _MatchContoursService;
typedef localization_defs::Landmark _LandmarkMsgType;
typedef localization_defs::LandmarkArray _LandmarkArrayMsgType;

typedef std::vector<_ContourMessage> _ContourMsgArrayType;
typedef std::array<_ContourMsgArrayType, NUM_LANDMARKS> _TemplateContours;

class LandmarkFinder: public _BaseNode
{
public:
	// LandmarkDescriptionArray -> ( this ) -> ColorBlobDescriptionArray -> ( ColorBlobFinder )
	//                                     /<------------ ColorBlobArray <-/
	//                             ( this ) -> ContourArrayPair ----------> ( ContourMatcher)
	//                                     /<------- MatchedContourArray <-/
	//                             ( this ) -> LandmarkArray -------------> ( LandmarkMapServer )
	//                                                      \-------------> ( Seabee3MissionControl )
	ros::ServiceServer set_enabled_landmarks_svr_;
	ros::ServiceClient find_color_blobs_cli_;
	ros::ServiceClient match_contours_cli_;
	ros::Publisher landmarks_pub_;

	_TemplateContours template_contours_;
	std::array<bool, NUM_LANDMARKS> enabled_types_;
	std::array<double, NUM_LANDMARKS> min_match_thresholds_;

	// these are updated each time setEnabledLandmarksCB is called
	_FindColorBlobsService::Request find_color_blobs_req_;
	_MatchContoursService::Request match_contours_req_;

	LandmarkFinder( ros::NodeHandle & nh ) :
			_BaseNode( nh )
	{
		set_enabled_landmarks_svr_ = nh_local_.advertiseService( "set_enabled_landmarks",
		                                                         &LandmarkFinder::setEnabledLandmarksCB,
		                                                         this );

		find_color_blobs_cli_ = nh_local_.serviceClient<_FindColorBlobsService>( "find_color_blobs" );

		match_contours_cli_ = nh_local_.serviceClient<_MatchContoursService>( "match_contours" );

		landmarks_pub_ = nh_local_.advertise<_LandmarkArrayMsgType>( "landmarks",
		                                                             1 );

		// load template contours here
	}

	virtual void reconfigureCB( _ReconfigureType &config, uint32_t level )
	{
		min_match_thresholds_[0] = config.buoy_min_match_threshold;
	}

	bool setEnabledLandmarksCB( _SetEnabledLandmarksService::Request & req,
	                            _SetEnabledLandmarksService::Response & resp )
	{
		find_color_blobs_req_ = {};
		match_contours_req_ = {};

		for( _DimType i = 0; i < enabled_types_.size(); ++i )
		{
			enabled_types_[i] = false;
		}

		for ( _DimType i = 0; i < req.descriptions.size(); ++i )
		{
			// we don't have to worry about duplicate colors; the color blob finder will handle it for us
			find_color_blobs_req_.colors.push_back( req.descriptions[i].color );

			// we need to worry about duplicate landmark types; we do this with a bitmask
			enabled_types_[req.descriptions[i].type] = true;
		}

		for( _DimType i = 0; i < enabled_types_.size(); ++i )
		{
			if( enabled_types_[i] ) appendTemplateContours( match_contours_req_.template_contours,
			            			                        i );
		}

		return true;
	}

	void spinOnce()
	{
		_FindColorBlobsService::Response find_color_blobs_resp;
		if( !find_color_blobs_cli_.call( find_color_blobs_req_, find_color_blobs_resp ) || find_color_blobs_resp.blobs.size() == 0 )
		{
			ROS_WARN( "No color blobs found..." );
			return;
		}

		match_contours_req_.candidate_contours.clear();

		for ( _DimType i = 0; i < find_color_blobs_resp.blobs.size(); ++i )
		{
			match_contours_req_.candidate_contours.push_back( find_color_blobs_resp.blobs[i].contour );
		}

		_MatchContoursService::Response match_contours_resp;
		if ( !match_contours_cli_.call( match_contours_req_,
		                                match_contours_resp ) ) return;

		_LandmarkArrayMsgType::Ptr landmark_array_msg( new _LandmarkArrayMsgType );
		for ( _DimType i = 0; i < match_contours_resp.matched_contours.size(); ++i )
		{
			int num_matches = 0;
			for ( _DimType j = 0; j < match_contours_resp.matched_contours[i].match_qualities.size(); ++j )
			{
				if ( match_contours_resp.matched_contours[i].match_qualities[j] < min_match_thresholds_[i] ) num_matches++;
			}

			if ( num_matches > 0 )
			{
				// reproject to 3D


				// add new landmark to resp
				_LandmarkMsgType landmark_msg;
				landmark_array_msg->landmarks.push_back( landmark_msg );
			}
		}

		landmarks_pub_.publish( landmark_array_msg );
	}

	// given a landmark id, append all known template contours for this landmark to the list of template contours
	void appendTemplateContours( std::vector<_ContourMessage> & template_contours,
	                             int landmark_id )
	{
		for ( _DimType i = 0; i < template_contours_[landmark_id].size(); ++i )
		{
			template_contours.push_back( template_contours[i] );
		}
	}
};

#endif /* LANDMARK_FINDER_H_ */
