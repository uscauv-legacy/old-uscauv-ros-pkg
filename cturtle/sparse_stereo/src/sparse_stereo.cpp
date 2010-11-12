#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <boost/thread.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <opencv/cv.h>
#include <cv_bridge/CvBridge.h>
#include <sensor_msgs/Image.h>
#include <dynamic_reconfigure/server.h>
#include <sparse_stereo/SparseStereoConfig.h>
#include <string>
#include <limits>
#include <sensor_msgs/PointCloud.h>
#include <image_geometry/stereo_camera_model.h>

typedef unsigned char _FeatureDescriptorDataType;
typedef double _MatchedFeatureDistanceDataType;

_MatchedFeatureDistanceDataType feature_space_min_distance_threshold_;

boost::mutex l_img_mutex_, r_img_mutex_, l_info_mutex_, r_info_mutex_, flag_mutex_;

bool new_left_img_ = false;
bool processed_left_img = false;
bool new_right_img_ = false;
bool processed_right_img = false;
bool new_left_info_ = false;
bool new_right_info_ = false;

double yspace_diff_threshold;
int blur_kernel_radius_;

sensor_msgs::ImagePtr left_img_, right_img_;
sensor_msgs::CameraInfo left_info_, right_info_;
image_transport::Publisher *disparity_pub;
sensor_msgs::CvBridge bridge;

IplImage *rightImage, *leftImage;
cv::Mat imageMask, combined;

ros::Publisher point_cloud_pub_;

image_geometry::StereoCameraModel stereo_model_;

bool initialized = false, stereo_model_initialized_ = false;

template<class _DescriptorDataType>
struct Feature
{
	std::vector<_DescriptorDataType> descriptor_;
	cv::Point pos_;
	cv::Vec3b color_;
};

typedef Feature<_FeatureDescriptorDataType> _Feature;

template<class _DescriptorDataType, class _DistanceDataType>
struct MatchedFeature
{
	Feature<_DescriptorDataType> left_feature_;
	Feature<_DescriptorDataType> right_feature_;
	_DistanceDataType distance_;

	MatchedFeature( Feature<_DescriptorDataType> left_feature, Feature<_DescriptorDataType> right_feature, _DistanceDataType distance )
	{
		left_feature_ = left_feature;
		right_feature_ = right_feature;
		distance_ = distance;
	}
};

typedef MatchedFeature<_FeatureDescriptorDataType, _MatchedFeatureDistanceDataType> _MatchedFeature;

std::vector<_Feature> left_features_, right_features_;
std::vector<_MatchedFeature> matched_features_;

template<class _DescriptorDataType>
_Feature calculateFeature( cv::Mat image_mat, cv::Point pos )
{
	_Feature feature;
	int keySize = 9;

	feature.pos_ = pos;

	cv::Vec3b center = image_mat.at<cv::Vec3b> ( pos );

	if ( ( pos.x > keySize / 2 ) && ( pos.x < image_mat.size().width - keySize / 2 ) && ( pos.y > keySize / 2 ) && ( pos.y < image_mat.size().height - keySize / 2 ) )
	{
		for ( int x = -keySize / 2; x < keySize / 2; x++ )
		{
			for ( int y = -keySize / 2; y < keySize / 2; y++ )
			{
				cv::Vec3b pix = image_mat.at<cv::Vec3b> ( cv::Point( pos.x + x, pos.y + y ) );
				feature.descriptor_.push_back( center[0] - pix[0] );
				feature.descriptor_.push_back( center[1] - pix[1] );
				feature.descriptor_.push_back( center[2] - pix[2] );

				feature.color_[0] += pix[0];
				feature.color_[1] += pix[1];
				feature.color_[2] += pix[2];
			}
		}

		feature.color_[0] /= keySize * keySize;
		feature.color_[1] /= keySize * keySize;
		feature.color_[2] /= keySize * keySize;
	}

	return feature;
}

std::vector<_Feature> calculateFeatures( cv::Mat image_mat, int corner_count = 300 )
{
	IplImage * image_bw = cvCreateImage( cvSize( image_mat.size().width, image_mat.size().height ), IPL_DEPTH_8U, 1 );
	IplImage * image = &IplImage( image_mat );
	cvCvtColor( image, image_bw, CV_BGR2GRAY );

	IplImage * eig_image = cvCreateImage( cvSize( image_bw->width, image_bw->height ), IPL_DEPTH_32F, 1 );
	IplImage * tmp_image = cvCreateImage( cvSize( image_bw->width, image_bw->height ), IPL_DEPTH_32F, 1 );
	CvPoint2D32f corners[corner_count];

	cvGoodFeaturesToTrack( image_bw, eig_image, tmp_image, corners, &corner_count, 0.1, 10.0 );

	cvReleaseImage( &image_bw );
	cvReleaseImage( &eig_image );
	cvReleaseImage( &tmp_image );

	cv::Mat image_mat_blur( image_mat.size(), image_mat.type() );

	cv::GaussianBlur( image_mat, image_mat_blur, cv::Size( 5, 5 ), 0.0 );

	std::vector<_Feature> features;
	//features.reserve( corner_count );
	for ( int i = 0; i < corner_count; i++ )
	{
		_Feature feature = calculateFeature<_FeatureDescriptorDataType> ( image_mat_blur, corners[i] );
		if ( feature.descriptor_.size() > 0 ) features.push_back( feature );
	}

	return features;
}

_MatchedFeatureDistanceDataType calculateFeatureDistance( _Feature left_feature, _Feature right_feature )
{
	_MatchedFeatureDistanceDataType result = 0;
	for ( size_t i = 0; i < left_feature.descriptor_.size(); i++ )
	{
		result += pow( left_feature.descriptor_[i] - right_feature.descriptor_[i], 2 );
	}
	return sqrt( result );
}

std::vector<_MatchedFeature> matchFeatures( std::vector<_Feature> left_features, std::vector<_Feature> right_features, _MatchedFeatureDistanceDataType threshold =
		feature_space_min_distance_threshold_ )
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
			if ( yspace_diff > yspace_diff_threshold ) continue;

			if ( left_features[l_feat_ind].pos_.x < right_features[r_feat_ind].pos_.x ) continue;

			_MatchedFeatureDistanceDataType feature_distance = calculateFeatureDistance( left_features[l_feat_ind], right_features[r_feat_ind] );

			if ( feature_distance < best_match_distance )
			{
				best_match_distance = feature_distance;
				best_match_index = r_feat_ind;
			}
		}

		if ( best_match_distance < threshold )
		{
			_MatchedFeature feature( left_features[l_feat_ind], right_features[best_match_index], best_match_distance );
			result.push_back( feature );
		}
	}

	return result;
}

std::vector<_Feature> processImage( IplImage * image, int y_draw_offset = 0 )

{
	cv::Mat image_mat = cv::Mat( image );

	for ( int y = 0; y < image->height; y++ )
	{
		for ( int x = 0; x < image->width; x++ )
		{
			cv::Vec3b data = image_mat.at<cv::Vec3b> ( cv::Point( x, y ) );
			combined.at<cv::Vec3b> ( cv::Point( x, y + y_draw_offset ) ) = data;
		}
	}

	std::vector<_Feature> result = calculateFeatures( image_mat );


	// draw keypoints
	for ( size_t i = 0; i < result.size(); i++ )
	{
		cv::Point temp = cv::Point( result[i].pos_.x, y_draw_offset + result[i].pos_.y );
		cv::circle( combined, temp, 3, cv::Scalar( 255, 0, 0 ) );
	}

	return result;
}

sensor_msgs::PointCloud projectKeypoints( std::vector<_MatchedFeature> features )
{
	sensor_msgs::PointCloud points;

	points.header.stamp = ros::Time::now();
	points.header.frame_id = "/sparse_stereo";
	points.channels.resize( features.size() );
	points.points.resize( features.size() );

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

		points.channels[i].values.push_back( *(float*) ( &rgb_packed ) );
		points.channels[i].name = "rgb";
	}

	return points;
}

void process_images()
{
	//ROS_INFO( "Trying to process available resources..." );
	if ( !initialized && new_right_img_ && new_left_img_ )
	{
		boost::lock_guard<boost::mutex> limgguard( l_img_mutex_ );
		boost::lock_guard<boost::mutex> rimgguard( r_img_mutex_ );

		leftImage = bridge.imgMsgToCv( left_img_ );
		rightImage = bridge.imgMsgToCv( right_img_ );
		cv::Mat left_tmp = cv::Mat( leftImage );

		int width, height;

		width = rightImage->width;
		height = std::max( rightImage->height, leftImage->height );

		imageMask = cv::Mat::ones( height, width, CV_8U);
		combined = cv::Mat( cv::Size( width, 2 * height ), left_tmp.type(), cv::Scalar( 0 ) );

		ROS_INFO( "Initialized." );
		initialized = true;
	}
	else if ( !initialized ) return;

	if ( new_left_img_ && !processed_left_img )
	{
		//ROS_INFO( "processing left image..." );
		boost::lock_guard<boost::mutex> limgguard( l_img_mutex_ );
		leftImage = bridge.imgMsgToCv( left_img_ );

		left_features_.clear();
		left_features_ = processImage( leftImage );
		ROS_DEBUG( "l_features: %zu\n", left_features_.size() );

		processed_left_img = true;
	}

	if ( new_right_img_ && !processed_right_img )
	{
		//ROS_INFO( "processing right image..." );
		boost::lock_guard<boost::mutex> rimgguard( r_img_mutex_ );
		rightImage = bridge.imgMsgToCv( right_img_ );

		right_features_.clear();
		right_features_ = processImage( rightImage, rightImage->height );
		ROS_DEBUG( "r_features: %zu\n", right_features_.size() );

		processed_right_img = true;
	}

	if ( new_left_img_ && new_right_img_ && new_left_info_ && new_right_info_ )
	{
		//ROS_INFO( "Checking locks..." );
		boost::lock_guard<boost::mutex> limgguard( l_img_mutex_ );
		boost::lock_guard<boost::mutex> rimgguard( r_img_mutex_ );
		boost::lock_guard<boost::mutex> linfoguard( l_info_mutex_ );
		boost::lock_guard<boost::mutex> rinfoguard( r_info_mutex_ );

		if ( !stereo_model_initialized_ ) stereo_model_.fromCameraInfo( left_info_, right_info_ );

		matched_features_.clear();
		matched_features_ = matchFeatures( left_features_, right_features_ );
		ROS_DEBUG( "matched_features: %zu\n", matched_features_.size() );

		for ( size_t i = 0; i < matched_features_.size(); i++ )
		{
			ROS_DEBUG( "matched feature distance: %f\n", matched_features_[i].distance_ );
			cv::line( combined, matched_features_[i].left_feature_.pos_, cv::Point( matched_features_[i].right_feature_.pos_.x, matched_features_[i].right_feature_.pos_.y + leftImage->height ),
					cv::Scalar( 0, 0, 255 ), 1 );
		}

		point_cloud_pub_.publish( projectKeypoints( matched_features_ ) );

		IplImage * combined_ipl = & ( (IplImage) combined );

		disparity_pub->publish( bridge.cvToImgMsg( combined_ipl ) );


		//Reset new flags
		new_left_img_ = false;
		new_right_img_ = false;
		new_left_info_ = false;
		new_right_info_ = false;
		processed_left_img = false;
		processed_right_img = false;

		ROS_INFO( "done" );
	}
}

void img_cb_l( const sensor_msgs::ImageConstPtr& msg )
{
	ROS_INFO( "got left img" );
	l_img_mutex_.lock();
	left_img_ = boost::const_pointer_cast<sensor_msgs::Image>( msg );
	//img_left = msg;

	l_img_mutex_.unlock();

	boost::lock_guard<boost::mutex> guard( flag_mutex_ );
	new_left_img_ = true;

	process_images();
}

void img_cb_r( const sensor_msgs::ImageConstPtr& msg )
{
	ROS_INFO( "got right img" );
	r_img_mutex_.lock();
	right_img_ = boost::const_pointer_cast<sensor_msgs::Image>( msg );
	//img_right = msg;

	r_img_mutex_.unlock();

	boost::lock_guard<boost::mutex> guard( flag_mutex_ );
	new_right_img_ = true;

	process_images();
}

void info_cb_l( const sensor_msgs::CameraInfoConstPtr& msg )
{
	ROS_INFO( "got left info" );
	l_info_mutex_.lock();
	left_info_ = *msg;
	//info_left = boost::const_pointer_cast<sensor_msgs::CameraInfo>(msg);

	l_info_mutex_.unlock();

	boost::lock_guard<boost::mutex> guard( flag_mutex_ );
	new_left_info_ = true;

	process_images();
}

void info_cb_r( const sensor_msgs::CameraInfoConstPtr& msg )
{
	ROS_INFO( "got right info" );
	r_info_mutex_.lock();
	right_info_ = *msg;

	r_info_mutex_.unlock();

	boost::lock_guard<boost::mutex> guard( flag_mutex_ );
	new_right_info_ = true;

	process_images();
}

void reconfigureCallback( sparse_stereo::SparseStereoConfig &config, uint32_t level )
{
	//ROS_INFO( "Setting norm_threshold to %f", config.norm_threshold );
	//ROS_INFO( "Setting norm_threshold to %f", config.norm_threshold );
	//norm_threshold = config.norm_threshold;
	blur_kernel_radius_ = config.blur_kernel_radius * 2 + 1;
	yspace_diff_threshold = (double) config.yspace_diff_threshold;
	feature_space_min_distance_threshold_ = (_MatchedFeatureDistanceDataType) config.feature_space_min_distance_threshold;
	//gauss_mean = config.gauss_mean;
	//gauss_variance = config.gauss_variance;
}

int main( int argc, char * argv[] )
{
	ros::init( argc, argv, "sparse_stereo" );
	ros::NodeHandle n;
	ros::NodeHandle n_priv( "~" );
	ros::MultiThreadedSpinner spinner( 4 );

	std::string stereo_ns, image, left_ns, right_ns;

	image_transport::Subscriber img_sub_l, img_sub_r;
	ros::Subscriber info_sub_l, info_sub_r;
	image_transport::ImageTransport it( n_priv );

	point_cloud_pub_ = n_priv.advertise<sensor_msgs::PointCloud> ( "points", 1 );

	n_priv.param( "stereo", stereo_ns, std::string( "/stereo" ) );
	n_priv.param( "image", image, std::string( "image_rect_color" ) );
	n_priv.param( "left", left_ns, std::string( "left" ) );
	n_priv.param( "right", right_ns, std::string( "right" ) );

	img_sub_l = it.subscribe( stereo_ns + "/" + left_ns + "/" + image, 1, img_cb_l );
	img_sub_r = it.subscribe( stereo_ns + "/" + right_ns + "/" + image, 1, img_cb_r );

	info_sub_l = n_priv.subscribe( stereo_ns + "/" + left_ns + "/camera_info", 1, info_cb_l );
	info_sub_r = n_priv.subscribe( stereo_ns + "/" + right_ns + "/camera_info", 1, info_cb_r );

	disparity_pub = new image_transport::Publisher;
	*disparity_pub = it.advertise( stereo_ns + "/disparity", 1 );

	dynamic_reconfigure::Server<sparse_stereo::SparseStereoConfig> srv;
	dynamic_reconfigure::Server<sparse_stereo::SparseStereoConfig>::CallbackType f = boost::bind( &reconfigureCallback, _1, _2 );
	srv.setCallback( f );

	spinner.spin();
}
