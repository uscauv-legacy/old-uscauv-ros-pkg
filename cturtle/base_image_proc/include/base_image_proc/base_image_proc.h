/*******************************************************************************
 *
 *      base_image_proc
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

#ifndef BASE_IMAGE_PROC_H_
#define BASE_IMAGE_PROC_H_

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
//#include <base_image_proc/BaseImageProcConfig.h>

template<typename _ReconfigureType, typename _ServiceType = std_srvs::Empty>
class BaseImageProc
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
	ros::ServiceServer service_srv_;
	_ServiceResponse last_response_;

	sensor_msgs::CvBridge bridge_;

	dynamic_reconfigure::Server<_ReconfigureType> reconfigure_srv_;
	typename dynamic_reconfigure::Server<_ReconfigureType>::CallbackType reconfigure_callback_;

	cv::Mat cv_img_;

	boost::mutex img_mutex_, info_mutex_, flag_mutex_;
	bool new_img_;
	std::string image_transport_;
	bool publish_image_, use_service_, reconfigure_initialized_;

public:
	BaseImageProc( ros::NodeHandle & nh, bool use_service = false, uint threads = 3 );
	~BaseImageProc();
	void spin();
protected:
	virtual void reconfigureCB( _ReconfigureType &config, uint32_t level );
	// process the last image; the intent is for this method to modify the image
	// called at the end of imageCB
	virtual cv::Mat processImage( IplImage * ipl_img );
	// process the last image given the request and provide a response; modification of the image is optional
	// called during serviceCB
	virtual cv::Mat processImage( IplImage * ipl_img, _ServiceRequest & req, _ServiceResponse & resp );
	void publishCvImage( cv::Mat & img );

private:
	void reconfigureCB_0( _ReconfigureType &config, uint32_t level );
	void imageCB( const sensor_msgs::ImageConstPtr& msg );
	void infoCB( const sensor_msgs::CameraInfoConstPtr& msg );
	bool serviceCB( _ServiceRequest & req, _ServiceResponse & resp );

};

template<typename _ReconfigureType, typename _ServiceType>
BaseImageProc<_ReconfigureType, _ServiceType>::BaseImageProc( ros::NodeHandle & nh, bool use_service, uint threads ) :
	nh_priv_( "~" ), spinner_( threads ), it_( nh_priv_ ), new_img_( false ), use_service_( use_service ), reconfigure_initialized_( false )
{
	nh_priv_.param( "image_transport", image_transport_, std::string( "raw" ) );
	nh_priv_.param( "publish_image", publish_image_, true );

	img_sub_ = it_.subscribe( nh.resolveName( "image" ), 1, &BaseImageProc::imageCB, this, image_transport_ );
	info_sub_ = nh_priv_.subscribe( nh.resolveName( "camera_info" ), 1, &BaseImageProc::infoCB, this );

	if ( publish_image_ ) img_pub_ = it_.advertise( "output_image", 1 );

	if ( use_service_ ) service_srv_ = nh_priv_.advertiseService( "service", &BaseImageProc::serviceCB, this );

	reconfigure_callback_ = boost::bind( &BaseImageProc::reconfigureCB_0, this, _1, _2 );
	reconfigure_srv_.setCallback( reconfigure_callback_ );
}

template<typename _ReconfigureType, typename _ServiceType>
BaseImageProc<_ReconfigureType, _ServiceType>::~BaseImageProc()
{
	//
}

template<typename _ReconfigureType, typename _ServiceType>
void BaseImageProc<_ReconfigureType, _ServiceType>::spin()
{
	spinner_.spin();
}

//virtual
template<typename _ReconfigureType, typename _ServiceType>
void BaseImageProc<_ReconfigureType, _ServiceType>::reconfigureCB( _ReconfigureType &config, uint32_t level )
{
	ROS_DEBUG( "Reconfigure successful in base class" );
}

//virtual
template<typename _ReconfigureType, typename _ServiceType>
cv::Mat BaseImageProc<_ReconfigureType, _ServiceType>::processImage( IplImage * ipl_img )
{
	ROS_DEBUG( "Processed image in base class" );
	return cv::Mat();
}

//virtual
template<typename _ReconfigureType, typename _ServiceType>
cv::Mat BaseImageProc<_ReconfigureType, _ServiceType>::processImage( IplImage * ipl_img, _ServiceRequest & req, _ServiceResponse & resp )
{
	return cv::Mat();
}

template<typename _ReconfigureType, typename _ServiceType>
void BaseImageProc<_ReconfigureType, _ServiceType>::publishCvImage( cv::Mat & img )
{
	ROS_DEBUG( "Published image" );
	IplImage * ipl_img = & ( (IplImage) img );
	img_pub_.publish( bridge_.cvToImgMsg( ipl_img ) );
}

template<typename _ReconfigureType, typename _ServiceType>
void BaseImageProc<_ReconfigureType, _ServiceType>::reconfigureCB_0( _ReconfigureType &config, uint32_t level )
{
	//config_dst_->reconfigureCB( config, level );
	reconfigureCB( config, level );
}

template<typename _ReconfigureType, typename _ServiceType>
void BaseImageProc<_ReconfigureType, _ServiceType>::imageCB( const sensor_msgs::ImageConstPtr& msg )
{
	ROS_DEBUG( "got image" );
	img_mutex_.lock();

	img_ = boost::const_pointer_cast<sensor_msgs::Image>( msg );

	img_mutex_.unlock();

	boost::lock_guard<boost::mutex> guard( flag_mutex_ );
	new_img_ = true;

	if ( reconfigure_initialized_ )
	{
		cv_img_ = processImage( bridge_.imgMsgToCv( img_ ) );

		if ( publish_image_ ) publishCvImage( cv_img_ );
	}
}

template<typename _ReconfigureType, typename _ServiceType>
void BaseImageProc<_ReconfigureType, _ServiceType>::infoCB( const sensor_msgs::CameraInfoConstPtr& msg )
{
	ROS_DEBUG( "got camera info" );
	info_mutex_.lock();

	info_ = *msg;

	info_mutex_.unlock();
}

template<typename _ReconfigureType, typename _ServiceType>
bool BaseImageProc<_ReconfigureType, _ServiceType>::serviceCB( _ServiceRequest & req, _ServiceResponse & resp )
{
	// if the image hasn't changed, just use the last response
	if ( !new_img_ )
	{
		resp = last_response_;
	}
	// if the image has changed, generate and then save a new response (and potentially a new output image)
	else
	{
		new_img_ = false;

		boost::lock_guard<boost::mutex> img_guard( img_mutex_ );


		//IplImg is a more low-level open-cv image type
		IplImage * ipl_img = bridge_.imgMsgToCv( img_ );


		// cv_img_ has the raw image data that should be analyzed
		if ( reconfigure_initialized_ ) cv_img_ = processImage( ipl_img, req, resp );


		// store the last response to save processing time
		last_response_ = resp;
	}

	if ( publish_image_ && reconfigure_initialized_ ) publishCvImage( cv_img_ );


	// notify ROS whether or not everything executed properly
	return true;
}

#endif /* BASE_IMAGE_PROC_H_ */

