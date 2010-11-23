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
#include <image_transport/image_transport.h>
#include <boost/thread.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <opencv/cv.h>
#include <cv_bridge/CvBridge.h>
#include <sensor_msgs/Image.h>
#include <sparse_stereo/SparseStereoConfig.h>
#include <string>
#include <limits>
#include <sensor_msgs/PointCloud.h>
#include <image_geometry/stereo_camera_model.h>
#include <base_node/base_node.h>

class SparseStereo: public BaseNode<sparse_stereo::SparseStereoConfig>
{
public:
	typedef unsigned char _FeatureDescriptorDataType;
	typedef double _MatchedFeatureDistanceDataType;

	template<class _DescriptorDataType>
	struct Feature
	{
		std::vector<_DescriptorDataType> descriptor_;
		cv::Point pos_;
		// bgr
		cv::Vec3b color_;
	};

	typedef Feature<_FeatureDescriptorDataType> _Feature;

	template<class _DescriptorDataType, class _DistanceDataType>
	struct MatchedFeature
	{
		Feature<_DescriptorDataType> left_feature_;
		Feature<_DescriptorDataType> right_feature_;
		_DistanceDataType distance_;

		MatchedFeature( Feature<_DescriptorDataType> left_feature,
		                Feature<_DescriptorDataType> right_feature,
		                _DistanceDataType distance )
		{
			left_feature_ = left_feature;
			right_feature_ = right_feature;
			distance_ = distance;
		}
	};

	typedef MatchedFeature<_FeatureDescriptorDataType, _MatchedFeatureDistanceDataType> _MatchedFeature;

private:
	_MatchedFeatureDistanceDataType feature_space_min_distance_threshold_;

	boost::mutex l_img_mutex_, r_img_mutex_, l_info_mutex_, r_info_mutex_, flag_mutex_;

	bool new_left_img_;
	bool processed_left_img_;
	bool new_right_img_;
	bool processed_right_img_;
	bool new_left_info_;
	bool new_right_info_;
	bool image_history_initialized_;

	// variables modified by dynamic_reconfigure
	double yspace_diff_threshold_;
	int blur_kernel_radius_, num_pyramid_levels_, feature_patch_size_;
	//

	sensor_msgs::ImagePtr left_image_msg_, right_img_msg_;
	sensor_msgs::CameraInfo left_info_, right_info_;
	image_transport::Publisher disparity_pub_;
	sensor_msgs::CvBridge bridge_;

	IplImage *right_image_, *left_image_;
	cv::Mat image_mask_, combined_image_mat_, last_left_image_mat_, last_right_image_mat_;

	ros::Publisher point_cloud_pub_;

	image_geometry::StereoCameraModel stereo_model_;

	std::string stereo_ns, image, left_ns, right_ns;

	image_transport::Subscriber image_l_sub_, image_r_sub_;
	ros::Subscriber info_l_sub_, info_r_sub_;

	image_transport::ImageTransport it_;

	bool combined_image_initialized_, stereo_model_initialized_;

	std::vector<_Feature> left_features_, right_features_;
	std::vector<_MatchedFeature> matched_features_;
	std::vector<_MatchedFeature> last_matched_features_;

	std::vector<cv::Point2f> left_matched_points_, right_matched_points_, left_output_points_, right_output_points_;
	std::vector<uchar> left_status_, right_status_;
	std::vector<float> left_error_, right_error_;

public:
	SparseStereo( ros::NodeHandle & nh ) :
		BaseNode<_ReconfigureType> ( nh ), it_( nh_priv_ ), new_left_img_( false ), new_left_info_( false ), new_right_img_( false ),
		        new_right_info_( false ), processed_left_img_( false ), processed_right_img_( false ), combined_image_initialized_( false ),
		        stereo_model_initialized_( false ), image_history_initialized_( false )
	{
		point_cloud_pub_ = nh_priv_.advertise<sensor_msgs::PointCloud> ( "points",
		                                                                 1 );

		nh_priv_.param( "stereo",
		                stereo_ns,
		                std::string( "/stereo" ) );
		nh_priv_.param( "image",
		                image,
		                std::string( "image_rect_color" ) );
		nh_priv_.param( "left",
		                left_ns,
		                std::string( "left" ) );
		nh_priv_.param( "right",
		                right_ns,
		                std::string( "right" ) );

		image_l_sub_ = it_.subscribe( stereo_ns + "/" + left_ns + "/" + image,
		                              1,
		                              &SparseStereo::leftImageCB,
		                              this );
		image_r_sub_ = it_.subscribe( stereo_ns + "/" + right_ns + "/" + image,
		                              1,
		                              &SparseStereo::rightImageCB,
		                              this );

		info_l_sub_ = nh_priv_.subscribe( stereo_ns + "/" + left_ns + "/camera_info",
		                                  1,
		                                  &SparseStereo::leftInfoCB,
		                                  this );
		info_r_sub_ = nh_priv_.subscribe( stereo_ns + "/" + right_ns + "/camera_info",
		                                  1,
		                                  &SparseStereo::rightInfoCB,
		                                  this );

		disparity_pub_ = it_.advertise( stereo_ns + "/disparity",
		                                1 );

		initCfgParams();
	}

	template<class _DescriptorDataType>
	_Feature calculateFeature( const std::vector<cv::Mat> & pyramid,
	                           cv::Point pos )
	{
		_Feature feature;
		feature.pos_ = pos;


		//cv::Point3d feature_color;

		cv::Mat image_mat = pyramid[0];

		const cv::Vec3b & feature_pix = image_mat.at<cv::Vec3b> ( pos );

		feature.color_[0] = feature_pix[0];
		feature.color_[1] = feature_pix[1];
		feature.color_[2] = feature_pix[2];


		// for every level in the pyramid
		for ( size_t i = 0; i < pyramid.size(); i++ )
		{
			// get the sub-patch of the original image (and scale the keypoint pos to match the shrinking sub-patch)
			cv::getRectSubPix( pyramid[i],
			                   cv::Size( feature_patch_size_,
			                             feature_patch_size_ ),
			                   cv::Point2f( (float) pos.x / float ( i + 1 ),
			                                (float) pos.y / float ( i + 1 ) ),
			                   image_mat );

			cv::Vec3b center = image_mat.at<cv::Vec3b> ( cv::Point( image_mat.size().width / 2,
			                                                        image_mat.size().height / 2 ) );

			for ( int x = 0; x < image_mat.size().width; x++ )
			{
				for ( int y = 0; y < image_mat.size().height; y++ )
				{
					const cv::Vec3b & pix = image_mat.at<cv::Vec3b> ( cv::Point( x,
					                                                             y ) );
					feature.descriptor_.push_back( center[0] - pix[0] );
					feature.descriptor_.push_back( center[1] - pix[1] );
					feature.descriptor_.push_back( center[2] - pix[2] );


					// the color of the feature is the average color of the feature's top-level neighborhood
					/*if ( i == 0 )
					 {
					 feature_color.x += pix[0];
					 feature_color.y += pix[1];
					 feature_color.z += pix[2];
					 }*/
				}
			}

		}


		/*feature.color_[0] = feature_color.x / ( feature_patch_size_ * feature_patch_size_ );
		 feature.color_[1] = feature_color.y / ( feature_patch_size_ * feature_patch_size_ );
		 feature.color_[2] = feature_color.z / ( feature_patch_size_ * feature_patch_size_ );*/

		//printf( "%d %d %d\n", feature.color_[0], feature.color_[1], feature.color_[2] );

		return feature;
	}

	std::vector<_Feature> calculateFeatures( cv::Mat image_mat,
	                                         int corner_count = 300 )
	{
		IplImage * image_bw = cvCreateImage( cvSize( image_mat.size().width,
		                                             image_mat.size().height ),
		                                     IPL_DEPTH_8U,
		                                     1 );
		IplImage * image = &IplImage( image_mat );
		cvCvtColor( image,
		            image_bw,
		            CV_BGR2GRAY );

		IplImage * eig_image = cvCreateImage( cvSize( image_bw->width,
		                                              image_bw->height ),
		                                      IPL_DEPTH_32F,
		                                      1 );
		IplImage * tmp_image = cvCreateImage( cvSize( image_bw->width,
		                                              image_bw->height ),
		                                      IPL_DEPTH_32F,
		                                      1 );
		CvPoint2D32f corners[corner_count];

		cvGoodFeaturesToTrack( image_bw,
		                       eig_image,
		                       tmp_image,
		                       corners,
		                       &corner_count,
		                       0.1,
		                       10.0 );

		cvReleaseImage( &image_bw );
		cvReleaseImage( &eig_image );
		cvReleaseImage( &tmp_image );

		cv::Mat image_mat_blur( image_mat.size(),
		                        image_mat.type() );

		cv::GaussianBlur( image_mat,
		                  image_mat_blur,
		                  cv::Size( 5,
		                            5 ),
		                  0.0 );

		std::vector<cv::Mat> pyramid( num_pyramid_levels_ + 1 );

		cv::buildPyramid( image_mat,
		                  pyramid,
		                  num_pyramid_levels_ );

		std::vector<_Feature> features;
		features.reserve( corner_count );
		for ( int i = 0; i < corner_count; i++ )
		{
			_Feature feature = calculateFeature<_FeatureDescriptorDataType> ( pyramid,
			                                                                  corners[i] );
			if ( feature.descriptor_.size() > 0 ) features.push_back( feature );
		}

		return features;
	}

	_MatchedFeatureDistanceDataType calculateFeatureDistance( _Feature left_feature,
	                                                          _Feature right_feature )
	{
		_MatchedFeatureDistanceDataType result = 0;
		for ( size_t i = 0; i < left_feature.descriptor_.size(); i++ )
		{
			result += pow( left_feature.descriptor_[i] - right_feature.descriptor_[i],
			               2 );
		}
		return sqrt( result ) / ( (double) pow( feature_patch_size_,
		                                        2 ) * (double) num_pyramid_levels_ );
	}

	std::vector<_MatchedFeature> matchFeatures( std::vector<_Feature> left_features,
	                                            std::vector<_Feature> right_features,
	                                            _MatchedFeatureDistanceDataType threshold )
	{
		std::vector<_MatchedFeature> result;
		_MatchedFeatureDistanceDataType max_match_distance = std::numeric_limits<_MatchedFeatureDistanceDataType>::max();

		for ( size_t l_feat_ind = 0; l_feat_ind < left_features.size(); l_feat_ind++ )
		{
			_MatchedFeatureDistanceDataType best_match_distance = max_match_distance;
			size_t best_match_index = 0;
			for ( size_t r_feat_ind = 0; r_feat_ind < right_features.size(); r_feat_ind++ )
			{
				double yspace_diff = fabs( left_features[l_feat_ind].pos_.y - right_features[r_feat_ind].pos_.y );
				if ( yspace_diff > yspace_diff_threshold_ ) continue;

				if ( left_features[l_feat_ind].pos_.x < right_features[r_feat_ind].pos_.x ) continue;

				_MatchedFeatureDistanceDataType feature_distance = calculateFeatureDistance( left_features[l_feat_ind],
				                                                                             right_features[r_feat_ind] );

				if ( feature_distance < best_match_distance )
				{
					best_match_distance = feature_distance;
					best_match_index = r_feat_ind;
				}
			}

			if ( best_match_distance < threshold )
			{
				_MatchedFeature feature( left_features[l_feat_ind],
				                         right_features[best_match_index],
				                         best_match_distance );
				result.push_back( feature );
			}
		}

		return result;
	}

	std::vector<_Feature> processImage( IplImage * image,
	                                    int y_draw_offset = 0 )

	{
		cv::Mat image_mat = cv::Mat( image );

		for ( int y = 0; y < image->height; y++ )
		{
			for ( int x = 0; x < image->width; x++ )
			{
				cv::Vec3b data = image_mat.at<cv::Vec3b> ( cv::Point( x,
				                                                      y ) );
				combined_image_mat_.at<cv::Vec3b> ( cv::Point( x,
				                                               y + y_draw_offset ) ) = data;
			}
		}

		std::vector<_Feature> result = calculateFeatures( image_mat );


		// draw keypoints
		for ( size_t i = 0; i < result.size(); i++ )
		{
			cv::Point temp = cv::Point( result[i].pos_.x,
			                            y_draw_offset + result[i].pos_.y );
			cv::circle( combined_image_mat_,
			            temp,
			            3,
			            cv::Scalar( 255,
			                        0,
			                        0 ) );
		}

		return result;
	}

	sensor_msgs::PointCloud projectFeatures( std::vector<_MatchedFeature> features )
	{
		sensor_msgs::PointCloud points;

		points.header.frame_id = "/sparse_stereo";
		points.channels.resize( 1 );
		points.channels[0].values.reserve( features.size() );
		points.points.resize( features.size() );


		// project points
		for ( size_t i = 0; i < features.size(); i++ )
		{
			cv::Point3d point;
			stereo_model_.projectDisparityTo3d( features[i].left_feature_.pos_,
			                                    features[i].left_feature_.pos_.x - features[i].right_feature_.pos_.x,
			                                    point );
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

	void process_images()
	{
		//ROS_DEBUG( "Trying to process available resources..." );
		if ( !combined_image_initialized_ && new_right_img_ && new_left_img_ )
		{
			boost::lock_guard<boost::mutex> limgguard( l_img_mutex_ );
			boost::lock_guard<boost::mutex> rimgguard( r_img_mutex_ );

			left_image_ = bridge_.imgMsgToCv( left_image_msg_ );
			right_image_ = bridge_.imgMsgToCv( right_img_msg_ );
			cv::Mat left_tmp = cv::Mat( left_image_ );

			int width, height;

			width = right_image_->width;
			height = std::max( right_image_->height,
			                   left_image_->height );

			image_mask_ = cv::Mat::ones( height,
			                             width,
			                             CV_8U);
			combined_image_mat_ = cv::Mat( cv::Size( width,
			                                         2 * height ),
			                               left_tmp.type(),
			                               cv::Scalar( 0 ) );

			ROS_DEBUG( "Initialized." );
			combined_image_initialized_ = true;
		}
		else if ( !combined_image_initialized_ ) return;

		if ( new_left_img_ && !processed_left_img_ )
		{
			//ROS_DEBUG( "processing left image..." );
			boost::lock_guard<boost::mutex> limgguard( l_img_mutex_ );
			left_image_ = bridge_.imgMsgToCv( left_image_msg_ );

			left_features_.clear();
			left_features_ = processImage( left_image_ );
			ROS_DEBUG( "l_features: %zu\n",
			           left_features_.size() );

			processed_left_img_ = true;
		}

		if ( new_right_img_ && !processed_right_img_ )
		{
			//ROS_DEBUG( "processing right image..." );
			boost::lock_guard<boost::mutex> rimgguard( r_img_mutex_ );
			right_image_ = bridge_.imgMsgToCv( right_img_msg_ );

			right_features_.clear();
			right_features_ = processImage( right_image_,
			                                right_image_->height );
			ROS_DEBUG( "r_features: %zu\n",
			           right_features_.size() );

			processed_right_img_ = true;
		}

		if ( new_left_img_ && new_right_img_ && new_left_info_ && new_right_info_ )
		{
			//ROS_DEBUG( "Checking locks..." );
			boost::lock_guard<boost::mutex> limgguard( l_img_mutex_ );
			boost::lock_guard<boost::mutex> rimgguard( r_img_mutex_ );
			boost::lock_guard<boost::mutex> linfoguard( l_info_mutex_ );
			boost::lock_guard<boost::mutex> rinfoguard( r_info_mutex_ );

			if ( !stereo_model_initialized_ ) stereo_model_.fromCameraInfo( left_info_,
			                                                                right_info_ );

			matched_features_.clear();
			matched_features_ = matchFeatures( left_features_,
			                                   right_features_,
			                                   feature_space_min_distance_threshold_ );
			ROS_DEBUG( "matched_features: %zu\n",
			           matched_features_.size() );

			matchTemporalFeatures();


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


			//Reset new flags
			new_left_img_ = false;
			new_right_img_ = false;
			new_left_info_ = false;
			new_right_info_ = false;
			processed_left_img_ = false;
			processed_right_img_ = false;

			ROS_DEBUG( "done" );
		}

	}

	void matchTemporalFeatures()
	{
		// optical flow in both images
		// get disparity

		IplImage * left_image_bw = cvCreateImage( cvSize( left_image_->width,
		                                                  left_image_->height ),
		                                          IPL_DEPTH_8U,
		                                          1 );
		cvCvtColor( left_image_,
		            left_image_bw,
		            CV_BGR2GRAY );
		IplImage * right_image_bw = cvCreateImage( cvSize( left_image_->width,
		                                                   left_image_->height ),
		                                           IPL_DEPTH_8U,
		                                           1 );
		cvCvtColor( right_image_,
		            right_image_bw,
		            CV_BGR2GRAY );

		const cv::Mat left_image_mat( left_image_bw );
		const cv::Mat right_image_mat( right_image_bw );

		if ( image_history_initialized_ )
		{

			left_matched_points_.resize( matched_features_.size() );
			right_matched_points_.resize( matched_features_.size() );

			left_output_points_.resize( matched_features_.size() );
			right_output_points_.resize( matched_features_.size() );

			left_status_.resize( matched_features_.size() );
			right_status_.resize( matched_features_.size() );

			left_error_.resize( matched_features_.size() );
			right_error_.resize( matched_features_.size() );

			for ( size_t i = 0; i < matched_features_.size(); i++ )
			{
				const cv::Point left_point = matched_features_[i].left_feature_.pos_;
				const cv::Point right_point = matched_features_[i].right_feature_.pos_;
				left_matched_points_[i].x = left_point.x;
				left_matched_points_[i].y = left_point.y;

				right_matched_points_[i].x = right_point.x;
				right_matched_points_[i].y = right_point.y;
			}

			/*
			 calcOpticalFlowPyrLK(const Mat& prevImg,
			 const Mat& nextImg,
			 const vector<Point2f>& prevPts,
			 vector<Point2f>& nextPts,
			 vector<uchar>& status,
			 vector<float>& err,
			 Size winSize=Size(15, 15),
			 int maxLevel=3,
			 TermCriteria criteria=TermCriteria( TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01),
			 double derivLambda=0.5, int flags=0)¶
			 */

			cv::Size searchSize( 20,
			                     20 );
			int maxLevel = 1;
			cv::TermCriteria criteria = cv::TermCriteria( cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
			                                              20,
			                                              0.01 );

			cv::calcOpticalFlowPyrLK( last_left_image_mat_,
			                          left_image_mat,
			                          left_matched_points_,
			                          left_output_points_,
			                          left_status_,
			                          left_error_,
			                          searchSize,
			                          maxLevel,
			                          criteria );

			cv::calcOpticalFlowPyrLK( last_right_image_mat_,
			                          right_image_mat,
			                          right_matched_points_,
			                          right_output_points_,
			                          right_status_,
			                          right_error_,
			                          searchSize,
			                          maxLevel,
			                          criteria );

			for ( size_t i = 0; i < left_matched_points_.size(); ++i )
			{
				//if(left_status_[i] > 0)
				{
					std::cout << "Left Point! " << std::endl;
					cv::line( combined_image_mat_,
					          left_matched_points_[i],
					          left_output_points_[i],
					          cv::Scalar( 255,
					                      0,
					                      0 ) );
				}
			}

			for ( size_t i = 0; i < right_matched_points_.size(); ++i )
			{
				//if ( right_status_[i] > 0 )
				{
					std::cout << "Right Point! " << std::endl;

					const cv::Point right_matched_point( right_matched_points_[i].x,
					                                     right_matched_points_[i].y + left_image_->height );
					const cv::Point right_output_point( right_output_points_[i].x,
					                                    right_output_points_[i].y + left_image_->height );

					cv::line( combined_image_mat_,
					          right_matched_point,
					          right_output_point,
					          cv::Scalar( 255,
					                      0,
					                      0 ) );
				}
			}

		}

		last_left_image_mat_ = cv::Mat( left_image_bw );
		last_right_image_mat_ = cv::Mat( right_image_bw );
		image_history_initialized_ = true;


		//cvReleaseImage( &left_image_bw );
		//cvReleaseImage( &right_image_bw );

	}

	void leftImageCB( const sensor_msgs::ImageConstPtr& msg )
	{
		ROS_DEBUG( "got left img" );
		l_img_mutex_.lock();
		left_image_msg_ = boost::const_pointer_cast<sensor_msgs::Image>( msg );
		//img_left = msg;

		l_img_mutex_.unlock();

		boost::lock_guard<boost::mutex> guard( flag_mutex_ );
		new_left_img_ = true;

		process_images();
	}

	void rightImageCB( const sensor_msgs::ImageConstPtr& msg )
	{
		ROS_DEBUG( "got right img" );
		r_img_mutex_.lock();
		right_img_msg_ = boost::const_pointer_cast<sensor_msgs::Image>( msg );
		//img_right = msg;

		r_img_mutex_.unlock();

		boost::lock_guard<boost::mutex> guard( flag_mutex_ );
		new_right_img_ = true;

		process_images();
	}

	void leftInfoCB( const sensor_msgs::CameraInfoConstPtr& msg )
	{
		ROS_DEBUG( "got left info" );
		l_info_mutex_.lock();
		left_info_ = *msg;
		//info_left = boost::const_pointer_cast<sensor_msgs::CameraInfo>(msg);

		l_info_mutex_.unlock();

		boost::lock_guard<boost::mutex> guard( flag_mutex_ );
		new_left_info_ = true;

		process_images();
	}

	void rightInfoCB( const sensor_msgs::CameraInfoConstPtr& msg )
	{
		ROS_DEBUG( "got right info" );
		r_info_mutex_.lock();
		right_info_ = *msg;

		r_info_mutex_.unlock();

		boost::lock_guard<boost::mutex> guard( flag_mutex_ );
		new_right_info_ = true;

		process_images();
	}

	void reconfigureCB( _ReconfigureType &config,
	                    uint32_t level )
	{
		yspace_diff_threshold_ = (double) config.yspace_diff_threshold;
		blur_kernel_radius_ = config.blur_kernel_radius * 2 + 1;
		num_pyramid_levels_ = config.num_pyramid_levels;
		feature_patch_size_ = config.feature_patch_size * 2 + 1;
		feature_space_min_distance_threshold_ = (_MatchedFeatureDistanceDataType) config.feature_space_min_distance_threshold;
	}

};

int main( int argc,
          char * argv[] )
{
	ros::init( argc,
	           argv,
	           "sparse_stereo" );
	ros::NodeHandle nh;

	SparseStereo sparse_stereo( nh );
	sparse_stereo.spin();
}
