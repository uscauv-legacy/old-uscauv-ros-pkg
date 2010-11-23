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

private:
	_MatchedFeatureDistanceDataType feature_space_min_distance_threshold_;

	bool image_history_initialized_;
	bool optical_flow_initialized_;

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

	ros::Publisher point_cloud_pub_;

	std::vector<_Feature> left_features_, right_features_;
	std::vector<_MatchedFeature> matched_features_;
	std::vector<_MatchedFeature> last_matched_features_;

	std::vector<cv::Point2f> left_matched_points_, right_matched_points_, left_output_points_, right_output_points_;
	std::vector<uchar> left_status_, right_status_;
	std::vector<float> left_error_, right_error_;

public:
	SparseStereo( ros::NodeHandle & nh ) :
		_BaseStereoImageProc( nh ), image_history_initialized_( false ), optical_flow_initialized_( false )
	{
		point_cloud_pub_ = nh_priv_.advertise<sensor_msgs::PointCloud> ( "points", 1 );

		initCfgParams();
	}

	std::vector<_MatchedFeature> calculateStereoOpticalFlow()
	{
		std::vector<_MatchedFeature> matched_features;


		// optimizations

		static IplImage * left_image_bw = NULL;
		static IplImage * right_image_bw = NULL;

		static IplImage* eig_image = NULL;
		static IplImage* tmp_image = NULL;

		static CvPoint2D32f* left_features = NULL;
		static CvPoint2D32f* right_features = NULL;

		static IplImage* left_pyr = NULL;
		static IplImage* right_pyr = NULL;

		static int last_num_corners_to_track = -1;

		if ( last_num_corners_to_track != num_corners_to_track_ || !optical_flow_initialized_ )
		{
			last_num_corners_to_track = num_corners_to_track_;

			if ( left_features ) delete[] left_features;
			left_features = new CvPoint2D32f[last_num_corners_to_track];
			if ( right_features ) delete[] right_features;
			right_features = new CvPoint2D32f[last_num_corners_to_track];
		}

		if ( !optical_flow_initialized_ )
		{
			left_image_bw = cvCreateImage( cv::Size( left_image_mat_.size().width, left_image_mat_.size().height ), IPL_DEPTH_8U, 1 );
			right_image_bw = cvCreateImage( cv::Size( right_image_mat_.size().width, right_image_mat_.size().height ), IPL_DEPTH_8U, 1 );

			const CvSize img_sz = cvGetSize( left_image_bw );
			eig_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
			tmp_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );

			const CvSize pyr_sz = cvSize( left_image_bw->width + 8, right_image_bw->height / 3 );
			left_pyr = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
			right_pyr = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );

			optical_flow_initialized_ = true;
		}

		// end optimizations

		cvCvtColor( left_image_, left_image_bw, CV_BGR2GRAY );
		cvCvtColor( right_image_, right_image_bw, CV_BGR2GRAY );

		cvGoodFeaturesToTrack( left_image_bw, eig_image, tmp_image, left_features, &last_num_corners_to_track, find_good_features_quality_level_, min_feature_distance_, 0, block_size_,
				enable_harris_corners_, find_good_features_k_ );

		const cv::Size window_size( search_window_width_percent_ * left_image_bw->width / 2, search_window_height_ / 2 );

		if ( enable_corner_sub_pix_ )
			cvFindCornerSubPix( left_image_bw, left_features, last_num_corners_to_track, cvSize( 10, 10 ), cvSize( -1, -1 ), cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, sub_pix_max_iterations_, sub_pix_min_epsilon_ ) );

		char features_found[last_num_corners_to_track];
		float feature_errors[last_num_corners_to_track];
		cvCalcOpticalFlowPyrLK( left_image_bw, right_image_bw, left_pyr, right_pyr, left_features, right_features, last_num_corners_to_track, window_size, num_pyramid_levels_, features_found,
				feature_errors, cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, flow_max_iterations_, flow_min_epsilon_ ), 0 );

		for ( int i = 0; i < last_num_corners_to_track; i++ )
		{
			if ( features_found[i] == 0 ) continue;

			_Feature left_feature( left_features[i] ), right_feature( right_features[i] );

			if ( left_feature.pos_.x > left_image_bw->width || left_feature.pos_.x < 0 || right_feature.pos_.x > right_image_bw->width || right_feature.pos_.x < 0 || left_feature.pos_.y
					> left_image_bw->height || left_feature.pos_.y < 0 || right_feature.pos_.y > right_image_bw->height || right_feature.pos_.y < 0 ) continue;

			left_feature.color_ = left_image_mat_.at<cv::Vec3b> ( left_feature.pos_ );
			right_feature.color_ = right_image_mat_.at<cv::Vec3b> ( right_feature.pos_ );

			_MatchedFeature matched_feature( left_feature, right_feature );

			const bool feature_error_check = feature_errors[i] < max_feature_error_threshold_;
			const bool feature_distance_check = matched_feature.distance_ >= min_matched_feature_distance_ && matched_feature.distance_ <= max_matched_feature_distance_;

			if ( feature_error_check && feature_distance_check ) matched_features.push_back( matched_feature );


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
			cv::circle( combined_image_mat_, left_feature.pos_, 3, cv::Scalar( 255, 0, 0 ), 1 );
			cv::circle( combined_image_mat_, cv::Point( right_feature.pos_.x, right_feature.pos_.y + left_image_mat_.size().height ), 3, cv::Scalar( 255, 0, 0 ), 1 );
			cv::line( combined_image_mat_, left_feature.pos_, cv::Point( right_feature.pos_.x, right_feature.pos_.y + left_image_mat_.size().height ), line_color, 1 );
		}


		// images are no longer re-allocated each time so we don't need to release them here
		//cvReleaseImage( &left_image_bw );
		//cvReleaseImage( &right_image_bw );
		//cvReleaseImage( &left_pyr );
		//cvReleaseImage( &right_pyr );
		//cvReleaseImage( &eig_image );
		//cvReleaseImage( &tmp_image );

		return matched_features;
	}

	sensor_msgs::PointCloud projectFeatures( std::vector<_MatchedFeature> features )
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

		matched_features_.clear();

		matched_features_ = calculateStereoOpticalFlow();

		ROS_DEBUG( "matched_features: %zu\n", matched_features_.size() );


		/*for ( size_t i = 0; i < matched_features_.size(); i++ )
		 {
		 // ROS_DEBUG( "matched feature distance: %f\n", matched_features_[i].distance_ );
		 if ( left_status_[i] > 0 && right_status_[i] > 0 )
		 {
		 std::cout << "Point!" << std::endl;
		 cv::line( combined_image_mat_, left_matched_points_[i], left_output_points_[i], cv::Scalar( 255, 0, 0 ) );

		 const cv::Point right_matched_point( right_matched_points_[i].x, right_matched_points_[i].y + left_image_->height );
		 const cv::Point right_output_point( right_output_points_[i].x, right_output_points_[i].y + left_image_->height );
		 cv::line( combined_image_mat_, right_matched_point, right_output_point, cv::Scalar( 255, 0, 0 ) );
		 }
		 //cv::line( combined_image_mat_, matched_features_[i].left_feature_.pos_, cv::Point( matched_features_[i].right_feature_.pos_.x, matched_features_[i].right_feature_.pos_.y
		 //		+ left_image_->height ), cv::Scalar( 0, 0, 255 ), 1 );
		 }*/

		point_cloud_pub_.publish( projectFeatures( matched_features_ ) );

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
