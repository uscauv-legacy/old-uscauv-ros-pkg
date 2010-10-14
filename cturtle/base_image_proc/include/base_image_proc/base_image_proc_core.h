/*******************************************************************************
 *
 *      base_image_proc_core
 * 
 *      Copyright (c) 2010, edward
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

#ifndef BASE_IMAGE_PROC_CORE_H_
#define BASE_IMAGE_PROC_CORE_H_

#include <ros/ros.h>
// for ImageTransport, Publisher
#include <image_transport/image_transport.h>
// for Mat
#include <opencv/cv.h>
//for CvBridge
#include <cv_bridge/CvBridge.h>
// for a ton of boost-related shit
#include <boost/thread.hpp>
// for CameraInfo
#include <sensor_msgs/CameraInfo.h>
// for pinhole camera model
#include <image_geometry/pinhole_camera_model.h>
// for dynamic reconfigure server
#include <dynamic_reconfigure/server.h>
// for the DoSomething service
#include <std_srvs/Empty.h>
// for cfg code
#include <base_image_proc/../../cfg/cpp/base_image_proc/EmptyConfig.h>

template<typename _ReconfigureType>
struct ReconfigureSettings
{
	static bool enableReconfigure()
	{
		return true;
	}
};

template<>
struct ReconfigureSettings<base_image_proc::EmptyConfig>
{
	static bool enableReconfigure()
	{
		return false;
	}
};

template<typename _ReconfigureType, typename _ServiceType>
class BaseImageProcCore
{
	typedef typename _ServiceType::Request _ServiceRequest;
	typedef typename _ServiceType::Response _ServiceResponse;

protected:
	ros::NodeHandle nh_priv_;
	ros::MultiThreadedSpinner spinner_;

	sensor_msgs::ImagePtr img_;
	sensor_msgs::CameraInfo info_;

	image_transport::Publisher img_pub_;

	image_transport::Subscriber img_sub_;
	image_transport::ImageTransport it_;

	ros::Subscriber info_sub_;

	sensor_msgs::CvBridge bridge_;

	dynamic_reconfigure::Server<_ReconfigureType> reconfigure_srv_;
	typename dynamic_reconfigure::Server<_ReconfigureType>::CallbackType reconfigure_callback_;

	cv::Mat cv_img_;

	boost::mutex img_mutex_, info_mutex_, flag_mutex_;
	bool new_img_;
	std::string image_transport_;
	bool publish_image_, reconfigure_initialized_, ignore_reconfigure_;

	// workaround for reconfigure ussues
	_ReconfigureType initial_config_params_;
	uint32_t initial_config_level_;

public:
	BaseImageProcCore( ros::NodeHandle & nh, uint threads = 3 );
	~BaseImageProcCore();
	void spin();
//	static bool hasReconfigure();

protected:
	virtual void reconfigureCB( _ReconfigureType &config, uint32_t level );
	void initCfgParams();
	virtual void imageCB();
	void publishCvImage( cv::Mat & img );

private:
	void reconfigureCB_0( _ReconfigureType &config, uint32_t level );
	void imageCB_0( const sensor_msgs::ImageConstPtr& msg );
	void infoCB( const sensor_msgs::CameraInfoConstPtr& msg );

};

/*template<typename _ServiceType> class BaseImageProcCore<base_image_proc::EmptyConfig, _ServiceType>
{
	static bool hasReconfigure();
};

// static
template<typename _ReconfigureType, typename _ServiceType>
bool BaseImageProcCore<_ReconfigureType, _ServiceType>::hasReconfigure()
{
	return false;
}

// if we're using EmptyConfig, set reconfigure_initialized to be true
// static
template<typename _ServiceType>
bool BaseImageProcCore<base_image_proc::EmptyConfig, _ServiceType>::hasReconfigure()
{
	return true;
}*/

template<typename _ReconfigureType, typename _ServiceType>
BaseImageProcCore<_ReconfigureType, _ServiceType>::BaseImageProcCore( ros::NodeHandle & nh, uint threads ) :
	nh_priv_( "~" ), spinner_( threads ), it_( nh_priv_ ), new_img_( false ), ignore_reconfigure_( ReconfigureSettings<_ReconfigureType>::enableReconfigure() )
{
	nh_priv_.param( "image_transport", image_transport_, std::string( "raw" ) );
	nh_priv_.param( "publish_image", publish_image_, true );

	img_sub_ = it_.subscribe( nh.resolveName( "image" ), 1, &BaseImageProcCore::imageCB_0, this, image_transport_ );
	info_sub_ = nh_priv_.subscribe( nh.resolveName( "camera_info" ), 1, &BaseImageProcCore::infoCB, this );

	if ( publish_image_ ) img_pub_ = it_.advertise( "output_image", 1 );

	reconfigure_initialized_ = ignore_reconfigure_;
	reconfigure_callback_ = boost::bind( &BaseImageProcCore::reconfigureCB_0, this, _1, _2 );
	reconfigure_srv_.setCallback( reconfigure_callback_ );
}

template<typename _ReconfigureType, typename _ServiceType>
BaseImageProcCore<_ReconfigureType, _ServiceType>::~BaseImageProcCore()
{
	//
}

template<typename _ReconfigureType, typename _ServiceType>
void BaseImageProcCore<_ReconfigureType, _ServiceType>::spin()
{
	spinner_.spin();
}

//virtual
template<typename _ReconfigureType, typename _ServiceType>
void BaseImageProcCore<_ReconfigureType, _ServiceType>::reconfigureCB( _ReconfigureType &config, uint32_t level )
{
	ROS_DEBUG( "Reconfigure successful in base class" );
}

template<typename _ReconfigureType, typename _ServiceType>
void BaseImageProcCore<_ReconfigureType, _ServiceType>::publishCvImage( cv::Mat & img )
{
	ROS_DEBUG( "Published image" );
	IplImage * ipl_img = & ( (IplImage) img );
	img_pub_.publish( bridge_.cvToImgMsg( ipl_img ) );
}

template<typename _ReconfigureType, typename _ServiceType>
void BaseImageProcCore<_ReconfigureType, _ServiceType>::reconfigureCB_0( _ReconfigureType &config, uint32_t level )
{
	//config_dst_->reconfigureCB( config, level );
	/*if( !reconfigure_initialized_ )
		initial_config_params_ = config;

	reconfigure_initialized_ = true;*/

	initial_config_params_ = config;
	initial_config_level_ = level;
	reconfigureCB( config, level );
}

template<typename _ReconfigureType, typename _ServiceType>
void BaseImageProcCore<_ReconfigureType, _ServiceType>::imageCB_0( const sensor_msgs::ImageConstPtr& msg )
{
	ROS_DEBUG( "got image" );
	img_mutex_.lock();

	img_ = boost::const_pointer_cast<sensor_msgs::Image>( msg );

	img_mutex_.unlock();

	boost::lock_guard<boost::mutex> guard( flag_mutex_ );
	new_img_ = true;

	imageCB();
}

template<typename _ReconfigureType, typename _ServiceType>
void BaseImageProcCore<_ReconfigureType, _ServiceType>::infoCB( const sensor_msgs::CameraInfoConstPtr& msg )
{
	ROS_DEBUG( "got camera info" );
	info_mutex_.lock();

	info_ = *msg;

	info_mutex_.unlock();
}

template<typename _ReconfigureType, typename _ServiceType>
void BaseImageProcCore<_ReconfigureType, _ServiceType>::imageCB()
{
	//
}

// virtual
template<typename _ReconfigureType, typename _ServiceType>
void BaseImageProcCore<_ReconfigureType, _ServiceType>::initCfgParams()
{
	if( !ignore_reconfigure_ )
		reconfigureCB( initial_config_params_, initial_config_level_ );
}

#endif /* BASE_IMAGE_PROC_CORE_H_ */
