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
#include <pipeline_finder/Pipeline.h>
#include <pipeline_finder/FindPipelines.h>
#include <localization_defs/LandmarkArray.h>
#include <landmark_map/landmark_map.h>
#include <landmark_finder/SetEnabledLandmarks.h>
#include <color_blob_finder/contour.h>
#include <array>
#include <landmark_finder/LandmarkFinderConfig.h>
#include <image_loader/image_loader.h>
#include <image_geometry/pinhole_camera_model.h>
#include <camera_interpolation/distance_interpolation.h>

typedef unsigned int _DimType;

const static _DimType NUM_LANDMARKS = 1;

typedef landmark_finder::LandmarkFinderConfig _ReconfigureType;
typedef BaseNode<_ReconfigureType> _BaseNode;
typedef landmark_finder::SetEnabledLandmarks _SetEnabledLandmarksService;
typedef color_blob_finder::FindColorBlobs _FindColorBlobsService;
typedef contour_matcher::MatchContours _MatchContoursService;
typedef pipeline_finder::FindPipelines _FindPipelinesService;
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
	ros::Subscriber camera_info1_sub_;
	ros::ServiceServer set_enabled_landmarks_svr_;
	ros::ServiceClient find_color_blobs_cli_;
	ros::ServiceClient match_contours_cli_;
	ros::ServiceClient find_pipelines_cli_;
	ros::Publisher landmarks_pub_;
	
	tf::Transform current_landmark_tf_;

	_TemplateContours template_contours_;
	std::array<bool, NUM_LANDMARKS> enabled_types_;
	std::array<double, NUM_LANDMARKS> min_match_thresholds_;

	// these are updated each time setEnabledLandmarksCB is called
	_FindColorBlobsService::Request find_color_blobs_req_;
	_MatchContoursService::Request match_contours_req_;
	_FindPipelinesService::Request find_pipelines_req_;

	sensor_msgs::CameraInfo camera_info1_msg_;
	image_geometry::PinholeCameraModel camera_model_;

	ImageLoader image_loader_;

	DistanceInterpolation * distance_interpolation_;

	int buoy_pixel_size_;
	double buoy_meter_size_;

	LandmarkFinder( ros::NodeHandle & nh ) :
			_BaseNode( nh ), image_loader_( nh_local_, CV_LOAD_IMAGE_GRAYSCALE )
	{
		set_enabled_landmarks_svr_ = nh_local_.advertiseService( "set_enabled_landmarks",
		                                                         &LandmarkFinder::setEnabledLandmarksCB,
		                                                         this );

		camera_info1_sub_ = nh_local_.subscribe( "camera_info1", 1, &LandmarkFinder::cameraInfo1CB, this );

		find_color_blobs_cli_ = nh_local_.serviceClient<_FindColorBlobsService>( "find_color_blobs" );

		match_contours_cli_ = nh_local_.serviceClient<_MatchContoursService>( "match_contours" );

		find_pipelines_cli_ = nh_local_.serviceClient<_FindPipelinesService>( "find_pipelines" );

		landmarks_pub_ = nh_local_.advertise<_LandmarkArrayMsgType>( "landmarks",
		                                                             1 );

		nh_local_.param( "buoy_pixel_size", buoy_pixel_size_, 94 );
		nh_local_.param( "buoy_meter_size", buoy_meter_size_, 0.23 );

		distance_interpolation_ = new DistanceInterpolation( DistanceInterpolation::getPixelMeters( buoy_pixel_size_, 0.5588, buoy_meter_size_ ), 1.0 );

		cv::namedWindow( "output", 0 );

		/*find_color_blobs_req_.colors = { 1 };
		image_array_pub = nh_local_.advertise( "template_images" );

		template_contours[0].push_back( )*/
	}

	virtual void reconfigureCB( _ReconfigureType &config,
	                            uint32_t level )
	{
		min_match_thresholds_[0] = config.buoy_min_match_threshold;
	}

	bool setEnabledLandmarksCB( _SetEnabledLandmarksService::Request & req,
	                            _SetEnabledLandmarksService::Response & resp )
	{
		find_color_blobs_req_ =
		{};
		match_contours_req_ =
		{};
		find_pipelines_req_ =
		{};

		for( _DimType i = 0; i < enabled_types_.size(); ++i )
		{
			enabled_types_[i] = false;
		}

		for ( _DimType i = 0; i < req.descriptions.size(); ++i )
		{
			printf("Processing description: %d %d\n", req.descriptions[i].type, req.descriptions[i].color );
			// we don't have to worry about duplicate colors; the color blob finder will handle it for us
			find_color_blobs_req_.colors.push_back( req.descriptions[i].color );

			// we need to worry about duplicate landmark types; we do this with a bitmask
			enabled_types_[req.descriptions[i].type] = true;
		}

		for( _DimType i = 0; i < enabled_types_.size(); ++i )
		{
			if( enabled_types_[i] ) appendTemplateContours( match_contours_req_.template_contours, i );
		}

		return true;
	}

	void cameraInfo1CB( const sensor_msgs::CameraInfo::ConstPtr & camera_info_msg )
	{
		//camera_info1_msg_ = *camera_info_msg;
		printf("Got camera info");
		camera_model_.fromCameraInfo( camera_info_msg );
	}

	void spinOnce()
	{
		// load template images and extract contours from them
		if( !image_loader_.images_loaded_ )
		{
			image_loader_.loadImages();

			for( _DimType i = 0; i < image_loader_.image_cache_.size(); ++i )
			{
				std::vector<_Contour> contours;
				cv::Mat image_mat( image_loader_.image_cache_[i] );
				findContours( image_mat, contours );

				printf(" Found %zu contours in template image (%dx%d)\n", contours.size(), image_loader_.image_cache_[i]->depth, image_loader_.image_cache_[i]->nChannels );

				/*IplImage * output_image = cvCreateImage( cvGetSize( image_loader_.image_cache_[i] ), IPL_DEPTH_8U, 3 );
				cv::Mat output_image_mat( output_image );

				cv::drawContours( output_image_mat, contours, -1, cvScalar( 0, 255, 0 ), 1 );

				cv::imshow( "output", output_image_mat );
				cvWaitKey( 25 );*/

				for( _DimType j = 0; j < contours.size(); ++j )
				{
					_ContourMessage contour_msg;
					contour_msg << contours[j];
					template_contours_[i].push_back( contour_msg );
				}
			}
		}

		tf_utils::publishTfFrame( current_landmark_tf_, "/landmark_map", "/current_landmark" );

		if( find_color_blobs_req_.colors.size() == 0 )
		{
			ROS_WARN( "No landmark types enabled." );
			return;
		}

		_FindColorBlobsService::Response find_color_blobs_resp;
		if( find_color_blobs_cli_.call( find_color_blobs_req_, find_color_blobs_resp ) )
		{
			if( find_color_blobs_resp.blobs.size() == 0 )
			{
				ROS_WARN( "No color blobs found." );
				return;
			}
		}
		else ROS_WARN( "Could not connect to color blob finder" );

		_LandmarkArrayMsgType::Ptr landmark_array_msg( new _LandmarkArrayMsgType );

		match_contours_req_.candidate_contours.clear();
		find_pipelines_req_.candidate_contours.clear();

		cv::Rect largest_rect( 0, 0, 0, 0 );
		_DimType index;
		_Contour largest_contour;

		for ( _DimType i = 0; i < find_color_blobs_resp.blobs.size(); ++i )
		{
//			match_contours_req_.candidate_contours.push_back( find_color_blobs_resp.blobs[i].contour );
//			if( find_color_blobs_resp.blobs[i].contour.header.frame_id == "camera_down" ) find_pipelines_req_.candidate_contours.push_back( find_color_blobs_resp.blobs[i].contour );

			// we'll pick the info from either camera_source1 or camera_source2 here
			//camera_model_.fromCameraInfo( camera_info1_msg_ );
			// reproject to 3D
			// Noah's code for 3D project, probably want to move where variables are defined
			// and need to define input diameter and contour.
			//Replace with some way of iterating through landmarks found.
			_Contour contour; //landmark contour
			contour << find_color_blobs_resp.blobs[i].contour;

			cv::Rect rect = cv::boundingRect( cv::Mat( contour ) );
			
			printf("Looking at color blob of width %d\n", rect.width);

			if( rect.width > largest_rect.width )
			{
				largest_rect = rect;
				largest_contour = contour;
				index = i;
			}

		}

		if( largest_rect.width > 0 )
		{
			int pixels = largest_rect.width;

			double distance = distance_interpolation_->distanceToFeature( pixels, 0.23, 1.33 );

			cv::Point2d box_pixel_center( largest_rect.x * 2 + largest_rect.width / 2, largest_rect.y * 2 + largest_rect.height / 2);
			cv::Point3d ray =  camera_model_.projectPixelTo3dRay( box_pixel_center );
			cv::Point3d point = ray * distance;

			printf("Publishing local frame to landmark");
			tf::Transform local_landmark_tf( tf_utils::ZERO_QUAT, tf::Vector3( point.x, point.y, point.z ) );
			tf_utils::publishTfFrame( local_landmark_tf, "/camera2", "/current_landmark_local" );

			printf("Looking up transform to local landmark");
			tf::Transform global_landmark_tf;
			tf_utils::fetchTfFrame( global_landmark_tf, "/landmark_map", "/current_landmark_local", ros::Time( 0 ), 0.01 );
			current_landmark_tf_ = global_landmark_tf;

			// add new landmark to resp
			cv::Point3d landmark_pos;
			landmark_pos.x = global_landmark_tf.getOrigin().getX();
			landmark_pos.y = global_landmark_tf.getOrigin().getY();
			landmark_pos.z = global_landmark_tf.getOrigin().getZ();

			LandmarkTypes::Buoy current_landmark( landmark_pos, 0.0, find_color_blobs_resp.blobs[index].color_id );
			landmark_array_msg->landmarks.push_back( current_landmark.createMsg() );
		}

/*		_MatchContoursService::Response match_contours_resp;
		if ( match_contours_cli_.call( match_contours_req_, match_contours_resp ) )
		{
			if( match_contours_resp.matched_contours.size() == 0 ) ROS_WARN( "No contours returned." );

			for ( _DimType i = 0; i < match_contours_resp.matched_contours.size(); ++i )
			{
				int num_matches = 0;
				for ( _DimType j = 0; j < match_contours_resp.matched_contours[i].match_qualities.size(); ++j )
				{
					printf( "match quality: %f\n", match_contours_resp.matched_contours[i].match_qualities[j] );
					if ( match_contours_resp.matched_contours[i].match_qualities[j] < min_match_thresholds_[i] ) num_matches++;
				}

				if ( num_matches > 0 )
				{
					// we'll pick the info from either camera_source1 or camera_source2 here
					//camera_model_.fromCameraInfo( camera_info1_msg_ );
					// reproject to 3D
					// Noah's code for 3D project, probably want to move where variables are defined
					// and need to define input diameter and contour.
					//Replace with some way of iterating through landmarks found.
					_Contour contour; //landmark contour
					contour << match_contours_req_.candidate_contours[i];

					cv::Rect rect = cv::boundingRect( cv::Mat( contour ) );
					int pixels = rect.width;

					double distance = distance_interpolation_->distanceToFeature( pixels, 0.23, 1.33 );

					cv::Point2d box_pixel_center( rect.x + rect.width / 2, rect.y + rect.height / 2);
					cv::Point3d ray =  camera_model_.projectPixelTo3dRay( box_pixel_center );
					cv::Point3d point = ray * distance;

					tf::Transform local_landmark_tf( tf_utils::ZERO_QUAT, tf::Vector3( point.x, point.y, point.z ) );
					tf_utils::publishTfFrame( local_landmark_tf, "/camera1", "/current_landmark" );

					tf::Transform global_landmark_tf;
					tf_utils::fetchTfFrame( global_landmark_tf, "/landmark_map", "/current_landmark" );

					// add new landmark to resp
					cv::Point3d landmark_pos;
					landmark_pos.x = global_landmark_tf.getOrigin().getX();
					landmark_pos.y = global_landmark_tf.getOrigin().getY();
					landmark_pos.z = global_landmark_tf.getOrigin().getZ();

					LandmarkTypes::Buoy current_landmark( landmark_pos, 0.0, find_color_blobs_resp.blobs[i].color_id );
					landmark_array_msg->landmarks.push_back( current_landmark.createMsg() );
				}
			}
		}
		else ROS_WARN( "Could not connect to contour matcher" );

		_FindPipelinesService::Response find_pipelines_resp;
		if( find_pipelines_cli_.call( find_pipelines_req_, find_pipelines_resp ) )
		{
			if( find_pipelines_resp.found_pipelines.size() == 0 ) ROS_WARN( "No pipelines found." );
			for( _DimType i = 0; i < find_pipelines_resp.found_pipelines.size(); ++i )
			{
				// Do awesome stuff here...
			}
		}
		else ROS_WARN( "Could not connect to pipe finder" );*/

		if( landmark_array_msg->landmarks.size() > 0 ) landmarks_pub_.publish( landmark_array_msg );
		else ROS_WARN( "No landmarks found." );
	}

	// given a landmark id, append all known template contours for this landmark to the list of template contours
	void appendTemplateContours( std::vector<_ContourMessage> & template_contours, int landmark_id )
	{
		printf( "Appending %zu contours for landmark with id %d\n", template_contours_[landmark_id].size(), landmark_id );
		for ( _DimType i = 0; i < template_contours_[landmark_id].size(); ++i )
		{
			template_contours.push_back( template_contours_[landmark_id][i] );
		}
	}
};

#endif /* LANDMARK_FINDER_H_ */
