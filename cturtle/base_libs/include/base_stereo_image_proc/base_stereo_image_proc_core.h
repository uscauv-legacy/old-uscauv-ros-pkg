/*******************************************************************************
 *
 *      base_stereo_image_proc_core
 * 
 *      Copyright (c) 2010
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

#ifndef BASE_STEREO_IMAGE_PROC_CORE_H_
#define BASE_STEREO_IMAGE_PROC_CORE_H_

// for ImageTransport, Publisher
#include <image_transport/image_transport.h>
// for Mat
#include <opencv/cv.h>
#include <cxcore.h>
// for image utilities like imread()
#include "highgui.h"
//for CvBridge
#include <cv_bridge/CvBridge.h>
// for a ton of boost-related shit
#include <boost/thread.hpp>
// for CameraInfo
#include <sensor_msgs/CameraInfo.h>
// for stereo camera model
#include <image_geometry/stereo_camera_model.h>
// for the DoSomething service
#include <std_srvs/Empty.h>
#include <base_node/base_node.h>

template<typename _ReconfigureType, typename _ServiceType>
class BaseStereoImageProcCore: public BaseNode<_ReconfigureType>
{
public:
	struct ImageId
	{
		const static int left = 0;
		const static int right = 1;
	};

protected:
	sensor_msgs::ImagePtr left_image_msg_, right_image_msg_;
	sensor_msgs::CameraInfo left_info_, right_info_;

	ros::Subscriber info_l_sub_, info_r_sub_;
	image_transport::Subscriber image_l_sub_, image_r_sub_;
	image_transport::Publisher disparity_pub_;

	image_transport::ImageTransport it_;
	sensor_msgs::CvBridge bridge_;

	IplImage *right_image_, *left_image_;
	cv::Mat left_image_mat_, right_image_mat_;
	cv::Mat combined_image_mat_;

	boost::mutex l_img_mutex_, r_img_mutex_, l_info_mutex_, r_info_mutex_, flag_mutex_;

	bool new_left_img_;
	bool processed_left_img_;
	bool new_right_img_;
	bool processed_right_img_;
	bool new_left_info_;
	bool new_right_info_;
	bool combined_image_initialized_;
	bool stereo_model_initialized_;

	image_geometry::StereoCameraModel stereo_model_;

	std::string stereo_ns, image, left_ns, right_ns;

	std::string image_transport_;
	bool publish_image_;
public:
	BaseStereoImageProcCore( ros::NodeHandle & nh,
	                         uint threads = 3 );
	virtual ~BaseStereoImageProcCore();

protected:
	void copyImageToCombined( int image_id );
	void copyImagesToCombined();
	virtual void leftImageCB();
	virtual void rightImageCB();
	void publishCvImage( cv::Mat & img );
	void publishCvImage( IplImage * ipl_img );

private:
	void leftImageCB_0( const sensor_msgs::ImageConstPtr& msg );
	void rightImageCB_0( const sensor_msgs::ImageConstPtr& msg );
	void leftInfoCB( const sensor_msgs::CameraInfoConstPtr& msg );
	void rightInfoCB( const sensor_msgs::CameraInfoConstPtr& msg );
	void initializeStereoModel();

};

template<typename _ReconfigureType, typename _ServiceType>
BaseStereoImageProcCore<_ReconfigureType, _ServiceType>::BaseStereoImageProcCore( ros::NodeHandle & nh,
                                                                                  uint threads ) :
	BaseNode<_ReconfigureType> ( nh,
	                             threads ), it_( this->nh_priv_ ), left_image_( NULL ), right_image_( NULL ), new_left_img_( false ),
	        processed_left_img_( false ), new_right_img_( false ), processed_right_img_( false ), new_left_info_( false ), new_right_info_( false ),
	        combined_image_initialized_( false ), stereo_model_initialized_( false )
{

	this->nh_priv_.param( "image_transport",
	                      image_transport_,
	                      std::string( "raw" ) );
	this->nh_priv_.param( "publish_image",
	                      publish_image_,
	                      true );
	this->nh_priv_.param( "stereo",
	                      stereo_ns,
	                      std::string( "/stereo" ) );
	this->nh_priv_.param( "image",
	                      image,
	                      std::string( "image_rect_color" ) );
	this->nh_priv_.param( "left",
	                      left_ns,
	                      std::string( "left" ) );
	this->nh_priv_.param( "right",
	                      right_ns,
	                      std::string( "right" ) );

	image_l_sub_ = it_.subscribe( stereo_ns + "/" + left_ns + "/" + image,
	                              1,
	                              &BaseStereoImageProcCore::leftImageCB_0,
	                              this,
	                              image_transport_ );
	image_r_sub_ = it_.subscribe( stereo_ns + "/" + right_ns + "/" + image,
	                              1,
	                              &BaseStereoImageProcCore::rightImageCB_0,
	                              this,
	                              image_transport_ );

	info_l_sub_ = this->nh_priv_.subscribe( stereo_ns + "/" + left_ns + "/camera_info",
	                                        1,
	                                        &BaseStereoImageProcCore::leftInfoCB,
	                                        this );
	info_r_sub_ = this->nh_priv_.subscribe( stereo_ns + "/" + right_ns + "/camera_info",
	                                        1,
	                                        &BaseStereoImageProcCore::rightInfoCB,
	                                        this );

	if ( publish_image_ ) disparity_pub_ = it_.advertise( stereo_ns + "/disparity",
	                                                      1 );
}

template<typename _ReconfigureType, typename _ServiceType>
BaseStereoImageProcCore<_ReconfigureType, _ServiceType>::~BaseStereoImageProcCore()
{
	//
}

template<typename _ReconfigureType, typename _ServiceType>
void BaseStereoImageProcCore<_ReconfigureType, _ServiceType>::copyImageToCombined( int image_id )
{
	if ( !combined_image_initialized_ )
	{
		int width = std::max( left_image_mat_.size().width,
		                      right_image_mat_.size().width ), height = left_image_mat_.size().height;

		combined_image_mat_ = cv::Mat( cv::Size( width,
		                                         2 * height ),
		                               left_image_mat_.type(),
		                               cv::Scalar( 0 ) );
		combined_image_initialized_ = true;
	}

	const cv::Mat & image_mat = image_id == ImageId::left ? left_image_mat_
	                                                      : right_image_mat_;

	int y_draw_offset = image_id == ImageId::left ? 0
	                                              : left_image_mat_.size().height;

	for ( int y = 0; y < image_mat.size().height; y++ )
	{
		for ( int x = 0; x < image_mat.size().width; x++ )
		{
			cv::Vec3b data = image_mat.at<cv::Vec3b> ( cv::Point( x,
			                                                      y ) );
			combined_image_mat_.at<cv::Vec3b> ( cv::Point( x,
			                                               y + y_draw_offset ) ) = data;
		}
	}
}

template<typename _ReconfigureType, typename _ServiceType>
void BaseStereoImageProcCore<_ReconfigureType, _ServiceType>::copyImagesToCombined()
{
	copyImageToCombined( ImageId::left );
	copyImageToCombined( ImageId::right );
}

template<typename _ReconfigureType, typename _ServiceType>
void BaseStereoImageProcCore<_ReconfigureType, _ServiceType>::publishCvImage( IplImage * ipl_img )
{
	ROS_DEBUG( "Published image" );
	disparity_pub_.publish( bridge_.cvToImgMsg( ipl_img ) );
}

template<typename _ReconfigureType, typename _ServiceType>
void BaseStereoImageProcCore<_ReconfigureType, _ServiceType>::publishCvImage( cv::Mat & img )
{
	IplImage * ipl_img = & ( (IplImage) img );
	publishCvImage( ipl_img );
}

template<typename _ReconfigureType, typename _ServiceType>
void BaseStereoImageProcCore<_ReconfigureType, _ServiceType>::leftImageCB_0( const sensor_msgs::ImageConstPtr& msg )
{
	ROS_DEBUG( "got left img" );
	l_img_mutex_.lock();
	left_image_msg_ = boost::const_pointer_cast<sensor_msgs::Image>( msg );

	l_img_mutex_.unlock();

	boost::lock_guard<boost::mutex> guard( flag_mutex_ );
	new_left_img_ = true;

	leftImageCB();
}

template<typename _ReconfigureType, typename _ServiceType>
void BaseStereoImageProcCore<_ReconfigureType, _ServiceType>::rightImageCB_0( const sensor_msgs::ImageConstPtr& msg )
{
	ROS_DEBUG( "got right img" );
	r_img_mutex_.lock();
	right_image_msg_ = boost::const_pointer_cast<sensor_msgs::Image>( msg );

	r_img_mutex_.unlock();

	boost::lock_guard<boost::mutex> guard( flag_mutex_ );
	new_right_img_ = true;

	rightImageCB();
}

template<typename _ReconfigureType, typename _ServiceType>
void BaseStereoImageProcCore<_ReconfigureType, _ServiceType>::leftInfoCB( const sensor_msgs::CameraInfoConstPtr& msg )
{
	if ( new_left_info_ ) return;

	ROS_DEBUG( "got left info" );
	l_info_mutex_.lock();
	left_info_ = *msg;

	l_info_mutex_.unlock();

	boost::lock_guard<boost::mutex> guard( flag_mutex_ );
	new_left_info_ = true;

	initializeStereoModel();
}

template<typename _ReconfigureType, typename _ServiceType>
void BaseStereoImageProcCore<_ReconfigureType, _ServiceType>::rightInfoCB( const sensor_msgs::CameraInfoConstPtr& msg )
{
	if ( new_right_info_ ) return;

	ROS_DEBUG( "got right info" );
	r_info_mutex_.lock();
	right_info_ = *msg;

	r_info_mutex_.unlock();

	boost::lock_guard<boost::mutex> guard( flag_mutex_ );
	new_right_info_ = true;

	initializeStereoModel();
}

template<typename _ReconfigureType, typename _ServiceType>
void BaseStereoImageProcCore<_ReconfigureType, _ServiceType>::initializeStereoModel()
{
	boost::lock_guard<boost::mutex> linfoguard( l_info_mutex_ );
	boost::lock_guard<boost::mutex> rinfoguard( r_info_mutex_ );

	if ( !stereo_model_initialized_ && new_left_info_ && new_right_info_ )
	{
		stereo_model_.fromCameraInfo( left_info_,
		                              right_info_ );
		stereo_model_initialized_ = true;
	}
}

// virtual
template<typename _ReconfigureType, typename _ServiceType>
void BaseStereoImageProcCore<_ReconfigureType, _ServiceType>::leftImageCB()
{
	//
}

// virtual
template<typename _ReconfigureType, typename _ServiceType>
void BaseStereoImageProcCore<_ReconfigureType, _ServiceType>::rightImageCB()
{
	//
}

#endif /* BASE_STEREO_IMAGE_PROC_CORE_H_ */
