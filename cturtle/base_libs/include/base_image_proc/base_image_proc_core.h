/*******************************************************************************
 *
 *      base_image_proc_core
 * 
 *      Copyright (c) 2010, Edward T. Kaszubski (ekaszubski@gmail.com)
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
// for pinhole camera model
#include <image_geometry/pinhole_camera_model.h>
// for the DoSomething service
#include <std_srvs/Empty.h>
#include <base_node/base_node.h>

// _DataType * pixel = getIplPixel( image, y, x );
// returns a reference to the pixel at (x, y) in image
// access channels in the pixel via pixel[channel]
template<class _DataType>
static _DataType * getIplPixel( IplImage * image, unsigned int row, unsigned int col )
{
	return (_DataType *) ( image->imageData + row * image->widthStep + image->nChannels * col * image->depth / 8 );
}

template<typename _ReconfigureType, typename _ServiceType>
class BaseImageProcCore: public BaseNode<_ReconfigureType>
{

protected:
	sensor_msgs::ImagePtr img_;
	sensor_msgs::CameraInfo info_;

	image_transport::Publisher img_pub_;

	image_transport::Subscriber img_sub_;
	image_transport::ImageTransport it_;

	ros::Subscriber info_sub_;

	sensor_msgs::CvBridge bridge_;

	cv::Mat cv_img_;

	boost::mutex img_mutex_, info_mutex_, flag_mutex_;
	bool new_img_;
	std::string image_transport_;
	bool publish_image_;
public:
	BaseImageProcCore( ros::NodeHandle & nh, uint threads = 3 );
	~BaseImageProcCore();

protected:
	virtual void imageCB();
	void publishCvImage( const cv::Mat & img );
	void publishCvImage( const IplImage * ipl_img );

private:
	void imageCB_0( const sensor_msgs::ImageConstPtr& msg );
	void infoCB( const sensor_msgs::CameraInfoConstPtr& msg );

};

template<typename _ReconfigureType, typename _ServiceType>
BaseImageProcCore<_ReconfigureType, _ServiceType>::BaseImageProcCore( ros::NodeHandle & nh, uint threads ) :
	BaseNode<_ReconfigureType>( nh, threads ), it_( this->nh_priv_ ), new_img_( false )
{
	this->nh_priv_.param( "image_transport", image_transport_, std::string( "raw" ) );
	this->nh_priv_.param( "publish_image", publish_image_, true );

	img_sub_ = it_.subscribe( nh.resolveName( "image" ), 1, &BaseImageProcCore::imageCB_0, this, image_transport_ );
	info_sub_ = this->nh_priv_.subscribe( nh.resolveName( "camera_info" ), 1, &BaseImageProcCore::infoCB, this );

	if ( publish_image_ ) img_pub_ = it_.advertise( "output_image", 1 );
}

template<typename _ReconfigureType, typename _ServiceType>
BaseImageProcCore<_ReconfigureType, _ServiceType>::~BaseImageProcCore()
{
	//
}

template<typename _ReconfigureType, typename _ServiceType>
void BaseImageProcCore<_ReconfigureType, _ServiceType>::publishCvImage( const IplImage * ipl_img )
{
	ROS_DEBUG( "Published image" );

	static sensor_msgs::Image::Ptr image_msg;
	image_msg = bridge_.cvToImgMsg( ipl_img );

	img_pub_.publish( image_msg );
}

template<typename _ReconfigureType, typename _ServiceType>
void BaseImageProcCore<_ReconfigureType, _ServiceType>::publishCvImage( const cv::Mat & img )
{
	//IplImage * ipl_img = & ( (IplImage) img );
	static IplImage * ipl_img = NULL;
	ipl_img = new IplImage( img );
	publishCvImage( ipl_img );
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

#endif /* BASE_IMAGE_PROC_CORE_H_ */
