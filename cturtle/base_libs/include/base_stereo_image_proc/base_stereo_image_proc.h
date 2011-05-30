/*******************************************************************************
 *
 *      base_stereo_image_proc
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

#ifndef BASE_STEREO_IMAGE_PROC_H_
#define BASE_STEREO_IMAGE_PROC_H_

#include <base_stereo_image_proc/base_stereo_image_proc_core.h>

template<typename _ReconfigureType = BaseNodeTypes::_DefaultReconfigureType, typename _ServiceType = std_srvs::Empty>
class BaseStereoImageProc: public BaseStereoImageProcCore<_ReconfigureType, _ServiceType>
{
public:
	typedef typename _ServiceType::Request _ServiceRequest;
	typedef typename _ServiceType::Response _ServiceResponse;

protected:
	ros::ServiceServer service_srv_;
	_ServiceResponse last_response_;

public:
	BaseStereoImageProc( ros::NodeHandle & nh,
	                     std::string service_name = "service",
	                     uint threads = 3 );
	~BaseStereoImageProc();

protected:
	// process the last image given the request and provide a response; modification of the image is optional
	// called during serviceCB
	virtual cv::Mat processImages( IplImage * left_image,
	                               IplImage * right_image,
	                               _ServiceRequest & req,
	                               _ServiceResponse & resp );
	bool serviceCB( _ServiceRequest & req,
	                _ServiceResponse & resp );
};

template<typename _ReconfigureType> class BaseStereoImageProc<_ReconfigureType, std_srvs::Empty> : public BaseStereoImageProcCore<_ReconfigureType,
        std_srvs::Empty>
{
public:
	BaseStereoImageProc( ros::NodeHandle & nh,
	                     uint threads = 3 );
	virtual ~BaseStereoImageProc();

protected:
	// process the last image; the intent is for this method to modify the image
	// called at the end of imageCB
	virtual cv::Mat processImages( IplImage * left_image,
	                               IplImage * right_image );
	virtual void validateAndProcessImages();
	virtual void leftImageCB();
	virtual void rightImageCB();

};

template<typename _ReconfigureType, typename _ServiceType>
BaseStereoImageProc<_ReconfigureType, _ServiceType>::BaseStereoImageProc( ros::NodeHandle & nh,
                                                                          std::string service_name,
                                                                          uint threads ) :
	BaseStereoImageProcCore<_ReconfigureType, _ServiceType> ( nh,
	                                                          threads )
{
	service_srv_ = this->nh_priv_.advertiseService( service_name,
	                                                &BaseStereoImageProc::serviceCB,
	                                                this );
}

template<typename _ReconfigureType>
BaseStereoImageProc<_ReconfigureType, std_srvs::Empty>::BaseStereoImageProc( ros::NodeHandle & nh,
                                                                             uint threads ) :
	BaseStereoImageProcCore<_ReconfigureType, std_srvs::Empty> ( nh,
	                                                             threads )
{
	//
}

template<typename _ReconfigureType, typename _ServiceType>
BaseStereoImageProc<_ReconfigureType, _ServiceType>::~BaseStereoImageProc()
{
	//
}

template<typename _ReconfigureType>
BaseStereoImageProc<_ReconfigureType, std_srvs::Empty>::~BaseStereoImageProc()
{
	//
}

//virtual
template<typename _ReconfigureType, typename _ServiceType>
cv::Mat BaseStereoImageProc<_ReconfigureType, _ServiceType>::processImages( IplImage * left_image,
                                                                            IplImage * right_image,
                                                                            _ServiceRequest & req,
                                                                            _ServiceResponse & resp )
{
	ROS_DEBUG( "Processed images in base class" );
	this->copyImagesToCombined();
	return cv::Mat( this->combined_image_mat_ );
}

//virtual
template<typename _ReconfigureType>
cv::Mat BaseStereoImageProc<_ReconfigureType, std_srvs::Empty>::processImages( IplImage * left_image,
                                                                               IplImage * right_image )
{
	ROS_DEBUG( "Processed images in base class" );
	this->copyImagesToCombined();
	return cv::Mat( this->combined_image_mat_ );
}

template<typename _ReconfigureType, typename _ServiceType>
bool BaseStereoImageProc<_ReconfigureType, _ServiceType>::serviceCB( _ServiceRequest & req,
                                                                     _ServiceResponse & resp )
{
	// don't do anything if we don't have a stereo model ready to use
	if ( !this->stereo_model_initialized_ || !this->reconfigure_initialized ) return false;
	// if either image hasn't changed, just use the last response
	if ( !this->new_left_img_ || !this->new_right_img_ )
	{
		resp = last_response_;
	}
	// if both images have changed, generate and then save a new response (and potentially a new output image)
	else if ( this->new_left_img_ && this->new_right_img_ )
	{
		this->new_left_img_ = this->new_right_img_ = false;

		boost::lock_guard<boost::mutex> l_img_guard( this->l_img_mutex_ );
		boost::lock_guard<boost::mutex> r_img_guard( this->r_img_mutex_ );

		this->left_image_mat_ = cv::Mat( this->bridge_.imgMsgToCv( this->left_image_msg_ ) );
		this->right_image_mat_ = cv::Mat( this->bridge_.imgMsgToCv( this->right_image_msg_ ) );


		//IplImg is a more low-level open-cv image type
		this->left_image_ = & ( (IplImage) this->left_image_mat_ );
		this->right_image_ = & ( (IplImage) this->right_image_mat_ );


		// cv_img_ has the raw image data that should be analyzed
		this->combined_image_mat_ = processImages( this->left_image_,
		                                           this->right_image_,
		                                           req,
		                                           resp );


		// store the last response to save processing time
		last_response_ = resp;
	}

	if ( this->publish_image_ && this->reconfigure_initialized_ ) this->publishCvImage( this->combined_image_mat_ );


	// notify ROS whether or not everything executed properly
	return true;
}

// virtual
template<typename _ReconfigureType>
void BaseStereoImageProc<_ReconfigureType, std_srvs::Empty>::validateAndProcessImages()
{
	boost::lock_guard<boost::mutex> limgguard( this->l_img_mutex_ );
	boost::lock_guard<boost::mutex> rimgguard( this->r_img_mutex_ );

	if ( this->new_left_img_ && this->new_right_img_ && this->reconfigure_initialized_ && this->stereo_model_initialized_ )
	{
		this->left_image_mat_ = cv::Mat( this->bridge_.imgMsgToCv( this->left_image_msg_ ) );
		this->right_image_mat_ = cv::Mat( this->bridge_.imgMsgToCv( this->right_image_msg_ ) );

		this->left_image_ = & ( (IplImage) this->left_image_mat_ );
		this->right_image_ = & ( (IplImage) this->right_image_mat_ );

		this->combined_image_mat_ = processImages( this->left_image_,
		                                           this->right_image_ );

		this->new_left_img_ = this->new_right_img_ = false;
		//processed_left_img_ = false;
		//processed_right_img_ = false;

		if ( this->publish_image_ ) this->publishCvImage( this->combined_image_mat_ );

		ROS_DEBUG( "done" );
	}
}

// virtual
template<typename _ReconfigureType>
void BaseStereoImageProc<_ReconfigureType, std_srvs::Empty>::leftImageCB()
{
	validateAndProcessImages();
}

// virtual
template<typename _ReconfigureType>
void BaseStereoImageProc<_ReconfigureType, std_srvs::Empty>::rightImageCB()
{
	validateAndProcessImages();
}

#endif /* BASE_STEREO_IMAGE_PROC_H_ */

