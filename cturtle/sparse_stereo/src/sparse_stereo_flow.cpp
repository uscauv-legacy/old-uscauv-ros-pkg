/*******************************************************************************
 *
 *      sparse_stereo
 *
 *      Copyright (c) 2010
 *
 *      Edward T. Kaszubski (ekaszubski@gmail.com)
 *      Rand Voorhies (rand.voorhies@gmail.com)
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
#include <pcl/common/transformation_from_correspondences.h>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <opencv/cv.h>
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sparse_stereo/SparseStereoConfig.h>
#include <string>
#include <limits>
#include <sensor_msgs/PointCloud.h>
#include <image_geometry/stereo_camera_model.h>
#include <base_stereo_image_proc/base_stereo_image_proc.h>
#include <Eigen/Core>
#include <Eigen3/Core>
#include <visualization_msgs/MarkerArray.h>

typedef BaseStereoImageProc<sparse_stereo::SparseStereoConfig> _BaseStereoImageProc;

class SparseStereo: public _BaseStereoImageProc
{
public:
	typedef unsigned char _FeatureDescriptorDataType;
	typedef double _MatchedFeatureDistanceDataType;

	template<class _DescriptorDataType>
	struct Feature
	{
		CvPoint2D32f pos_subpix_;
		cv::Point pos_;
		// bgr
		cv::Vec3b color_;

		Feature( CvPoint2D32f pos_subpix = cvPoint2D32f( 0.0, 0.0 ), cv::Vec3b color = cv::Vec3b() )
		{
			pos_subpix_ = pos_subpix;
			pos_ = cv::Point( cvRound( pos_subpix_.x ), cvRound( pos_subpix_.y ) );
			color_ = color;
		}
	};

	typedef Feature<_FeatureDescriptorDataType> _Feature;

	template<class _DescriptorDataType, class _DistanceDataType>
	struct MatchedFeature
	{
		Feature<_DescriptorDataType> left_feature_;
		Feature<_DescriptorDataType> right_feature_;
		_DistanceDataType distance_;

		MatchedFeature( Feature<_DescriptorDataType> left_feature, Feature<_DescriptorDataType> right_feature )
		{
			left_feature_ = left_feature;
			right_feature_ = right_feature;
			distance_ = sqrt( pow( left_feature_.pos_.x - right_feature_.pos_.x, 2 ) + pow( left_feature_.pos_.y - right_feature_.pos_.y, 2 ) );


			// features that the left camera can see are always to the right of features that the right camera can see (ignoring reflections)
			distance_ *= ( left_feature_.pos_.x >= right_feature_.pos_.x ? 1.0 : -1.0 );
		}
	};

	typedef MatchedFeature<_FeatureDescriptorDataType, _MatchedFeatureDistanceDataType> _MatchedFeature;
	typedef std::pair<std::vector<_MatchedFeature>, std::vector<_MatchedFeature> > _TemporalMatchedFeatureSet;
	typedef cv::Point3d _Point3;

private:
	_MatchedFeatureDistanceDataType feature_space_min_distance_threshold_;

	bool image_history_initialized_;
	bool optical_flow_initialized_;
	int last_marker_size_;

	// variables modified by dynamic_reconfigure
	int search_window_height_;
	int num_pyramid_levels_;
	int num_corners_to_track_;
	double search_window_width_percent_;
	double max_feature_error_threshold_;
	double min_feature_distance_;
	double min_matched_feature_distance_;
	double max_matched_feature_distance_;
	int block_size_;
	bool enable_harris_corners_;
	bool enable_corner_sub_pix_;
	double find_good_features_quality_level_;
	double find_good_features_k_;
	int flow_max_iterations_;
	double flow_min_epsilon_;
	int sub_pix_max_iterations_;
	double sub_pix_min_epsilon_;
	//

	ros::Publisher point_cloud_pub_, marker_array_pub_;

	std::vector<_Feature> left_features_, right_features_;
	_TemporalMatchedFeatureSet temporal_matched_features_;
	std::vector<_MatchedFeature> last_matched_features_;

	//pcl::TransformationFromCorrespondences world_transform_solver_;

public:
	SparseStereo( ros::NodeHandle & nh ) :
		_BaseStereoImageProc( nh ), image_history_initialized_( false ), optical_flow_initialized_( false ), last_marker_size_( 0 )
	{
		point_cloud_pub_ = nh_priv_.advertise<sensor_msgs::PointCloud> ( "points", 1 );
		marker_array_pub_ = nh_priv_.advertise<visualization_msgs::MarkerArray> ( "marker_array", 0 );

		initCfgParams();
	}

	_TemporalMatchedFeatureSet calculateStereoOpticalFlow()
	{
		std::vector<_MatchedFeature> matched_features, last_matched_features;
		_TemporalMatchedFeatureSet temporal_matched_features;

		static bool optical_flow_data_initialized = false;


		// optimizations

		static IplImage * new_left_image_bw = NULL;
		static IplImage * new_right_image_bw = NULL;
		static IplImage * old_left_image_bw = NULL;
		static IplImage * old_right_image_bw = NULL;

		static IplImage* eig_image = NULL;
		static IplImage* tmp_image = NULL;

		static std::vector<CvPoint2D32f> old_left_features;
		static std::vector<CvPoint2D32f> old_right_features;

		static std::vector<CvPoint2D32f> new_left_features;
		static std::vector<CvPoint2D32f> new_right_features;

		static IplImage* left_pyr = NULL;
		static IplImage* right_pyr = NULL;

		static int last_num_corners_to_track = -1;

		if ( last_num_corners_to_track != num_corners_to_track_ || !optical_flow_initialized_ )
		{
			last_num_corners_to_track = num_corners_to_track_;

			old_left_features.resize( last_num_corners_to_track );
			old_right_features.resize( last_num_corners_to_track );

			new_left_features.resize( last_num_corners_to_track );
			new_right_features.resize( last_num_corners_to_track );
		}

		if ( !optical_flow_initialized_ )
		{
			new_left_image_bw = cvCreateImage( cv::Size( left_image_mat_.size().width, left_image_mat_.size().height ), IPL_DEPTH_8U, 1 );
			new_right_image_bw = cvCreateImage( cv::Size( right_image_mat_.size().width, right_image_mat_.size().height ), IPL_DEPTH_8U, 1 );

			old_left_image_bw = cvCreateImage( cv::Size( left_image_mat_.size().width, left_image_mat_.size().height ), IPL_DEPTH_8U, 1 );
			old_right_image_bw = cvCreateImage( cv::Size( right_image_mat_.size().width, right_image_mat_.size().height ), IPL_DEPTH_8U, 1 );

			const CvSize img_sz = cvGetSize( new_left_image_bw );
			eig_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
			tmp_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );

			const CvSize pyr_sz = cvSize( new_left_image_bw->width + 8, new_right_image_bw->height / 3 );
			left_pyr = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
			right_pyr = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );

			optical_flow_initialized_ = true;
		}

		// end optimizations

		cvCvtColor( left_image_, new_left_image_bw, CV_BGR2GRAY );
		cvCvtColor( right_image_, new_right_image_bw, CV_BGR2GRAY );

		if ( !optical_flow_data_initialized )
		{
			cvCopy( new_left_image_bw, old_left_image_bw );
			cvCopy( new_right_image_bw, old_right_image_bw );
			optical_flow_data_initialized = true;
			return temporal_matched_features;
		}

		cvGoodFeaturesToTrack( new_left_image_bw, eig_image, tmp_image, &old_left_features[0], &last_num_corners_to_track, find_good_features_quality_level_, min_feature_distance_, 0, block_size_,
				enable_harris_corners_, find_good_features_k_ );

		const cv::Size window_size( search_window_width_percent_ * new_left_image_bw->width / 2, search_window_height_ / 2 );

		if ( enable_corner_sub_pix_ ) cvFindCornerSubPix( new_left_image_bw, &old_left_features[0], last_num_corners_to_track, cvSize( 10, 10 ), cvSize( -1, -1 ), cvTermCriteria( CV_TERMCRIT_ITER
				| CV_TERMCRIT_EPS, sub_pix_max_iterations_, sub_pix_min_epsilon_ ) );

		char features_found_spat_old[last_num_corners_to_track];
		float feature_errors_spat_old[last_num_corners_to_track];
		cvCalcOpticalFlowPyrLK( old_left_image_bw, old_right_image_bw, left_pyr, right_pyr, &old_left_features[0], &old_right_features[0], last_num_corners_to_track, window_size, num_pyramid_levels_,
				features_found_spat_old, feature_errors_spat_old, cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, flow_max_iterations_, flow_min_epsilon_ ), 0 );

		char features_found_temporal[last_num_corners_to_track];
		float feature_errors_temporal[last_num_corners_to_track];
		cvCalcOpticalFlowPyrLK( old_left_image_bw, new_left_image_bw, left_pyr, right_pyr, &old_left_features[0], &new_left_features[0], last_num_corners_to_track, window_size, num_pyramid_levels_,
				features_found_temporal, feature_errors_temporal, cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, flow_max_iterations_, flow_min_epsilon_ ), 0 );

		char features_found_spat_new[last_num_corners_to_track];
		float feature_errors_spat_new[last_num_corners_to_track];
		cvCalcOpticalFlowPyrLK( new_left_image_bw, new_right_image_bw, left_pyr, right_pyr, &new_left_features[0], &new_right_features[0], last_num_corners_to_track, window_size, num_pyramid_levels_,
				features_found_spat_new, feature_errors_spat_new, cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, flow_max_iterations_, flow_min_epsilon_ ), 0 );

		cvCopy( new_left_image_bw, old_left_image_bw );
		cvCopy( new_right_image_bw, old_right_image_bw );

		for ( int i = 0; i < last_num_corners_to_track; i++ )
		{
			if ( features_found_spat_old[i] == 0 || features_found_temporal[i] == 0 || features_found_spat_new[i] == 0 ) continue;

			_Feature new_left_feature( new_left_features[i] ), new_right_feature( new_right_features[i] );
			_Feature old_left_feature( old_left_features[i] ), old_right_feature( old_right_features[i] );

			_MatchedFeature old_matched_feature( old_left_feature, old_right_feature );
			_MatchedFeature new_matched_feature( new_left_feature, new_right_feature );

			if ( new_left_feature.pos_ == old_left_feature.pos_ || new_right_feature.pos_ == old_right_feature.pos_ ) continue;

			if ( new_left_feature.pos_.x > new_left_image_bw->width || new_left_feature.pos_.x < 0 || new_right_feature.pos_.x > new_right_image_bw->width || new_right_feature.pos_.x < 0
					|| new_left_feature.pos_.y > new_left_image_bw->height || new_left_feature.pos_.y < 0 || new_right_feature.pos_.y > new_right_image_bw->height || new_right_feature.pos_.y < 0 ) continue;

			new_left_feature.color_ = left_image_mat_.at<cv::Vec3b> ( new_left_feature.pos_ );
			new_right_feature.color_ = right_image_mat_.at<cv::Vec3b> ( new_right_feature.pos_ );

			const bool feature_error_check = feature_errors_spat_new[i] < max_feature_error_threshold_;
			const bool feature_distance_check = new_matched_feature.distance_ >= min_matched_feature_distance_ && new_matched_feature.distance_ <= max_matched_feature_distance_;

			if ( feature_error_check && feature_distance_check )
			{
				//printf( "%d %d\n", old_left_feature.pos_.x, new_left_feature.pos_.x );
				//printf( "pushing back feature %d %d\n", old_matched_feature.left_feature_.pos_.x, new_matched_feature.left_feature_.pos_.x );
				temporal_matched_features.first.push_back( old_matched_feature );
				temporal_matched_features.second.push_back( new_matched_feature );

				cv::line( combined_image_mat_, old_left_feature.pos_, new_left_feature.pos_, cv::Scalar( 0, 255, 0 ), 1 );
			}

			// don't draw anything if we're not going to publish the image
			if ( !publish_image_ || disparity_pub_.getNumSubscribers() == 0 ) continue;

			// default to red
			cv::Scalar line_color = cv::Scalar( 255, 0, 0 );

			if ( !feature_error_check )
			{
				line_color = cv::Scalar( 0, 127, 255 );
			}
			// failing the distance check overrides failing the error check
			if ( !feature_distance_check )
			{
				line_color = cv::Scalar( 0, 0, 255 );
			}
			cv::circle( combined_image_mat_, new_left_feature.pos_, 3, cv::Scalar( 255, 0, 0 ), 1 );
			cv::circle( combined_image_mat_, cv::Point( new_right_feature.pos_.x, new_right_feature.pos_.y + left_image_mat_.size().height ), 3, cv::Scalar( 255, 0, 0 ), 1 );
			cv::line( combined_image_mat_, new_left_feature.pos_, cv::Point( new_right_feature.pos_.x, new_right_feature.pos_.y + left_image_mat_.size().height ), line_color, 1 );
		}

		return temporal_matched_features;
	}

	void clearMarkers()
	{
		visualization_msgs::MarkerArray markers;
		markers.markers.resize( last_marker_size_ );
		for ( int i = 0; i < last_marker_size_; i++ )
		{
			visualization_msgs::Marker marker;
			marker.header.frame_id = "/sparse_stereo";
			marker.header.stamp = ros::Time();
			marker.ns = "velocities";
			marker.id = i;
			marker.action = visualization_msgs::Marker::DELETE;
			markers.markers[i] = marker;
		}

		marker_array_pub_.publish( markers );
	}

	void publishVelocityArrows( std::vector<_Point3> & old_points, std::vector<_Point3> & new_points )
	{
		clearMarkers();

		visualization_msgs::MarkerArray markers;
		markers.markers.resize( old_points.size() );
		last_marker_size_ = markers.markers.size();


		// generate arrows
		for ( size_t i = 0; i < markers.markers.size(); i++ )
		{
			visualization_msgs::Marker marker;

			marker.header.frame_id = "/sparse_stereo";
			marker.header.stamp = ros::Time();
			marker.ns = "velocities";
			marker.id = i;
			marker.type = visualization_msgs::Marker::ARROW;
			marker.action = visualization_msgs::Marker::ADD;

			geometry_msgs::Point old_point, new_point;
			old_point.x = old_points[i].x;
			old_point.y = old_points[i].y;
			old_point.z = old_points[i].z;

			new_point.x = new_points[i].x;
			new_point.y = new_points[i].y;
			new_point.z = new_points[i].z;

			marker.points.push_back( old_point );
			marker.points.push_back( new_point );

			marker.scale.x = 0.05;
			marker.scale.y = 0.1;
			marker.scale.z = 0.1;

			marker.color.a = 1.0;
			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;

			markers.markers[i] = marker;
		}

		marker_array_pub_.publish( markers );
	}

	std::vector<_Point3> projectFeatures( std::vector<_MatchedFeature> & features )
	{
		std::vector<_Point3> points( features.size() );

		for ( size_t i = 0; i < features.size(); i++ )
		{
			cv::Point3d point;
			stereo_model_.projectDisparityTo3d( features[i].left_feature_.pos_, features[i].left_feature_.pos_.x - features[i].right_feature_.pos_.x, point );
			points[i].x = point.x;
			points[i].y = point.y;
			points[i].z = point.z;

		}

		return points;
	}

	tf::Transform calculateWorldTransform( std::vector<_Point3> & old_points, std::vector<_Point3> & new_points )
	{
		world_transform_solver_.reset();

		for ( size_t i = 0; i < old_points.size(); ++i )
		{
			const Eigen::Vector3f old_point, new_point;

			old_point.x() = old_points[i].x;
			old_point.y() = old_points[i].y;
			old_point.z() = old_points[i].z;

			new_point.x() = new_points[i].x;
			new_point.y() = new_points[i].y;
			new_point.z() = new_points[i].z;

			world_transform_solver_.add( old_point, new_point );
		}

		Eigen3::Affine3f world_transform = world_transform_solver_.getTransformation();
		tf::Transform world_transform_tf;

		return world_transform_tf;
	}

	sensor_msgs::PointCloud projectPoints( std::vector<_MatchedFeature> features )
	{
		sensor_msgs::PointCloud points;

		points.header.frame_id = "/sparse_stereo";
		points.channels.resize( 1 );
		points.channels[0].values.reserve( features.size() );
		points.points.resize( features.size() );


		//printf("publishing cloud of %d points\n", features.size() );

		// project points
		for ( size_t i = 0; i < features.size(); i++ )
		{
			cv::Point3d point;
			stereo_model_.projectDisparityTo3d( features[i].left_feature_.pos_, features[i].left_feature_.pos_.x - features[i].right_feature_.pos_.x, point );
			points.points[i].x = point.x;
			points.points[i].y = point.y;
			points.points[i].z = point.z;

			const cv::Vec3b& bgr = features[i].left_feature_.color_;
			int32_t rgb_packed = ( bgr[2] << 16 ) | ( bgr[1] << 8 ) | bgr[0];

			points.channels[0].values.push_back( *(float*) ( &rgb_packed ) );
			points.channels[0].name = "rgb";
		}

		points.header.stamp = ros::Time::now();

		return points;
	}


	/*void processLeftImage( IplImage * left_image )
	 {
	 cv::Mat left_image_mat( left_image );

	 std::vector<_MatchedFeature> matched_features;
	 IplImage * left_image_bw = cvCreateImage( cvSize( left_image->width,
	 left_image->height ),
	 IPL_DEPTH_8U,
	 1 );
	 cvCvtColor( left_image,
	 left_image_bw,
	 CV_BGR2GRAY );
	 }

	 void processRightImage( IplImage * right_image )
	 {
	 cv::Mat left_image_mat( left_image );
	 right_image_bw_ = cvCreateImage( cvSize( right_image->width,
	 right_image->height ),
	 IPL_DEPTH_8U,
	 1 );
	 cvCvtColor( right_image,
	 right_image_bw,
	 CV_BGR2GRAY );
	 }*/

	cv::Mat processImages( IplImage * left_image, IplImage * right_image )
	{
		copyImagesToCombined();

		_TemporalMatchedFeatureSet temporal_matched_features = calculateStereoOpticalFlow();

		if ( temporal_matched_features.first.size() == 0 || temporal_matched_features.second.size() == 0 ) return combined_image_mat_;

		std::vector<_Point3> old_points = projectFeatures( temporal_matched_features.first );
		std::vector<_Point3> new_points = projectFeatures( temporal_matched_features.second );

		std::vector<_Point3> old_points_clean;
		std::vector<_Point3> new_points_clean;

		double max_change_in_feature_position = 0.5;
		for ( size_t i = 0; i < old_points.size(); ++i )
		{
			double distance = sqrt( pow( old_points[i].x - new_points[i].x, 2 ) + pow( old_points[i].y - new_points[i].y, 2 ) + pow( old_points[i].z - new_points[i].z, 2 ) );

			if ( distance < max_change_in_feature_position )
			{
				old_points_clean.push_back( old_points[i] );
				new_points_clean.push_back( new_points[i] );

			}

		}
		publishVelocityArrows( old_points_clean, new_points_clean );


		//Eigen3::Affine3f world_transform = calculateWorldTransform( old_points_clean, new_points_clean );


		point_cloud_pub_.publish( temporal_matched_features.second );

		IplImage * combined_ipl = & ( (IplImage) combined_image_mat_ );

		disparity_pub_.publish( bridge_.cvToImgMsg( combined_ipl ) );

		return combined_image_mat_;
	}

	void reconfigureCB( _ReconfigureType &config, uint32_t level )
	{
		search_window_height_ = config.search_window_height;
		search_window_width_percent_ = config.search_window_width_percent;
		num_pyramid_levels_ = config.num_pyramid_levels;
		num_corners_to_track_ = config.num_corners_to_track;
		max_feature_error_threshold_ = config.max_feature_error_threshold;
		min_feature_distance_ = config.min_feature_distance;
		min_matched_feature_distance_ = config.min_matched_feature_distance;
		max_matched_feature_distance_ = config.max_matched_feature_distance;
		block_size_ = 2 * config.block_size + 1;
		enable_harris_corners_ = config.enable_harris_corners;
		enable_corner_sub_pix_ = config.enable_corner_sub_pix;
		find_good_features_quality_level_ = config.find_good_features_quality_level;
		find_good_features_k_ = config.find_good_features_k;
		flow_max_iterations_ = config.flow_max_iterations;
		flow_min_epsilon_ = config.flow_min_epsilon;
		sub_pix_max_iterations_ = config.sub_pix_max_iterations;
		sub_pix_min_epsilon_ = config.sub_pix_min_epsilon;
	}

};

int main( int argc, char * argv[] )
{
	ros::init( argc, argv, "sparse_stereo" );
	ros::NodeHandle nh;

	SparseStereo sparse_stereo( nh );
	sparse_stereo.spin();
}
