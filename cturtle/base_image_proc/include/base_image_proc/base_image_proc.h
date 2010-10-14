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

#include <base_image_proc/base_image_proc_core.h>

struct BaseImageProcSettings
{
	typedef base_image_proc::EmptyConfig _DefaultReconfigureType;
};

template<typename _ReconfigureType, typename _ServiceType = std_srvs::Empty>
class BaseImageProc: public BaseImageProcCore<_ReconfigureType, _ServiceType>
{
	typedef typename _ServiceType::Request _ServiceRequest;
	typedef typename _ServiceType::Response _ServiceResponse;
protected:

	ros::ServiceServer service_srv_;
	_ServiceResponse last_response_;

public:
	BaseImageProc( ros::NodeHandle & nh, uint threads = 3 );
	~BaseImageProc();

protected:
	// process the last image given the request and provide a response; modification of the image is optional
	// called during serviceCB
	virtual cv::Mat processImage( IplImage * ipl_img, _ServiceRequest & req, _ServiceResponse & resp );
	bool serviceCB( _ServiceRequest & req, _ServiceResponse & resp );
};

template<typename _ReconfigureType> class BaseImageProc<_ReconfigureType, std_srvs::Empty> : public BaseImageProcCore<_ReconfigureType, std_srvs::Empty>
{
public:
	BaseImageProc( ros::NodeHandle & nh, uint threads = 3 );
	~BaseImageProc();

protected:
	// process the last image; the intent is for this method to modify the image
	// called at the end of imageCB
	virtual cv::Mat processImage( IplImage * ipl_img );
	virtual void imageCB();

};

template<typename _ReconfigureType, typename _ServiceType>
BaseImageProc<_ReconfigureType, _ServiceType>::BaseImageProc( ros::NodeHandle & nh, uint threads ) :
	BaseImageProcCore<_ReconfigureType, _ServiceType> ( nh, threads )
{
	service_srv_ = this->nh_priv_.advertiseService( "service", &BaseImageProc::serviceCB, this );
}

template<typename _ReconfigureType>
BaseImageProc<_ReconfigureType, std_srvs::Empty>::BaseImageProc( ros::NodeHandle & nh, uint threads ) :
	BaseImageProcCore<_ReconfigureType, std_srvs::Empty> ( nh, threads )
{
	//
}

template<typename _ReconfigureType, typename _ServiceType>
BaseImageProc<_ReconfigureType, _ServiceType>::~BaseImageProc()
{
	//
}

template<typename _ReconfigureType>
BaseImageProc<_ReconfigureType, std_srvs::Empty>::~BaseImageProc()
{
	//
}

//virtual
template<typename _ReconfigureType, typename _ServiceType>
cv::Mat BaseImageProc<_ReconfigureType, _ServiceType>::processImage( IplImage * ipl_img, _ServiceRequest & req, _ServiceResponse & resp )
{
	ROS_DEBUG( "Processed image in base class" );
	return cv::Mat( ipl_img );
}

//virtual
template<typename _ReconfigureType>
cv::Mat BaseImageProc<_ReconfigureType, std_srvs::Empty>::processImage( IplImage * ipl_img )
{
	ROS_DEBUG( "Processed image in base class" );
	return cv::Mat( ipl_img );
}

template<typename _ReconfigureType, typename _ServiceType>
bool BaseImageProc<_ReconfigureType, _ServiceType>::serviceCB( _ServiceRequest & req, _ServiceResponse & resp )
{
	// if the image hasn't changed, just use the last response
	if ( !this->new_img_ )
	{
		resp = last_response_;
	}
	// if the image has changed, generate and then save a new response (and potentially a new output image)
	else
	{
		this->new_img_ = false;

		boost::lock_guard<boost::mutex> img_guard( this->img_mutex_ );


		//IplImg is a more low-level open-cv image type
		IplImage * ipl_img = this->bridge_.imgMsgToCv( this->img_ );


		// cv_img_ has the raw image data that should be analyzed
		if ( this->reconfigure_initialized_ ) this->cv_img_ = processImage( ipl_img, req, resp );


		// store the last response to save processing time
		last_response_ = resp;
	}

	if ( this->publish_image_ && this->reconfigure_initialized_ ) this->publishCvImage( this->cv_img_ );


	// notify ROS whether or not everything executed properly
	return true;
}

// virtual
template<typename _ReconfigureType>
void BaseImageProc<_ReconfigureType, std_srvs::Empty>::imageCB()
{
	if ( this->reconfigure_initialized_ )
	{
		this->cv_img_ = processImage( this->bridge_.imgMsgToCv( this->img_ ) );

		if ( this->publish_image_ ) this->publishCvImage( this->cv_img_ );
	}
}

#endif /* BASE_IMAGE_PROC_H_ */

