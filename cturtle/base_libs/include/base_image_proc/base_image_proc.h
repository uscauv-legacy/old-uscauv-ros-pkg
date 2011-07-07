/*******************************************************************************
 *
 *      base_image_proc
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

#ifndef BASE_IMAGE_PROC_H_
#define BASE_IMAGE_PROC_H_

#include <base_image_proc/base_image_proc_core.h>

template<typename _ReconfigureType = BaseNodeTypes::_DefaultReconfigureType, typename _ServiceType = std_srvs::Empty>
class BaseImageProc : public BaseImageProcCore<_ReconfigureType, _ServiceType>
{
public:
	typedef typename _ServiceType::Request _ServiceRequest;
	typedef typename _ServiceType::Response _ServiceResponse;

protected:
	ros::ServiceServer service_srv_;
	_ServiceResponse last_response_;

public:
	BaseImageProc( ros::NodeHandle & nh, std::string reconfigure_ns = "reconfigure", std::string service_name = "service", uint threads = 3 ) :
		BaseImageProcCore<_ReconfigureType, _ServiceType> ( nh, reconfigure_ns, threads )
	{
		ROS_INFO( "Setting up base_image_proc..." );

		ROS_INFO( "Setting up service %s...", service_name );
		service_srv_ = this->nh_local_.advertiseService( service_name, &BaseImageProc::serviceCB, this );
		ROS_INFO( "Done setting up service %s", service_name );

		ROS_INFO( "Done setting up base_image_proc" );
	}

	virtual ~BaseImageProc()
	{
		this->image_mutex_.unlock();
	}

protected:
	// process the last image given the request and provide a response; modification of the image is optional
	// called during serviceCB
	virtual IplImage * processImage( IplImage * ipl_image, _ServiceRequest & req, _ServiceResponse & resp )
	{
		if( !this->running_ ) return ipl_image;

		ROS_WARN( "Processed image in base class" );
		return ipl_image;
	}

	bool serviceCB( _ServiceRequest & req, _ServiceResponse & resp )
	{
		if( !this->running_ ) return false;
		IplImage * new_image = NULL;
		IplImage * result = NULL;

		if( !this->image_mutex_.try_lock() ) return false;

		// if the image hasn't changed, just use the last response
		if ( !this->new_image_ )
		{
			resp = last_response_;
		}
		// if the image has changed, generate and then save a new response (and potentially a new output image)
		else
		{
			this->new_image_ = false;

			//IplImg is a more low-level open-cv image type
			new_image = this->image_bridge_.imgMsgToCv( this->image_msg_ );

			// cv_img_ has the raw image data that should be analyzed
			if ( this->reconfigure_initialized_ ) result = processImage( new_image, req, resp );

			if ( result && result->width == 0 && result->height == 0 ) this->ipl_image_ = result;
			else this->ipl_image_ = new_image;

			// store the last response to save processing time
			last_response_ = resp;
		}

		if ( this->publish_image_ && this->reconfigure_initialized_ && this->ipl_image_ ) this->publishCvImage( this->ipl_image_ );

		this->image_mutex_.unlock();

		// notify ROS whether or not everything executed properly
		return result;

	}
};

template<typename _ReconfigureType>
class BaseImageProc<_ReconfigureType, std_srvs::Empty> : public BaseImageProcCore<_ReconfigureType, std_srvs::Empty>
{
public:
	BaseImageProc( ros::NodeHandle & nh, std::string reconfigure_ns = "reconfigure", uint threads = 3 ) :
		BaseImageProcCore<_ReconfigureType, std_srvs::Empty> ( nh, reconfigure_ns, threads )
	{
		ROS_INFO( "Setting up base_image_proc with no service..." );
		ROS_INFO( "Done setting up base_image_proc..." );
	}
	virtual ~BaseImageProc()
	{
		this->image_mutex_.unlock();
	}

protected:
	// process the last image; the intent is for this method to modify the image
	// called at the end of imageCB
	virtual IplImage * processImage( IplImage * ipl_image )
	{
		if( !this->running_ ) return ipl_image;
		ROS_WARN( "Processed image in base class" );
		return ipl_image;
	}

	virtual void imageCB( const sensor_msgs::ImageConstPtr& image_msg )
	{
		if( !this->running_ ) return;
		if ( !this->ignore_reconfigure_ && this->reconfigure_initialized_ )
		{
			this->new_image_ = true;
			this->ipl_image_ = processImage( this->image_bridge_.imgMsgToCv( this->image_msg_ ) );

			if ( this->publish_image_ ) this->publishCvImage( this->ipl_image_ );
		}
		else ROS_WARN( "Dropped image because the reconfigure params have not been set" );
	}

};

#endif /* BASE_IMAGE_PROC_H_ */

