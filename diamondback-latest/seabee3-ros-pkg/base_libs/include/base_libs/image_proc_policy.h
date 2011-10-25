/***************************************************************************
 *  include/base_libs/image_proc_policy.h
 *  --------------------
 * 
 *  Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *  
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer
 *    in the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of seabee3-ros-pkg nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *  
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 **************************************************************************/

#ifndef BASE_LIBS_BASE_LIBS_IMAGE_PROC_POLICY_H_
#define BASE_LIBS_BASE_LIBS_IMAGE_PROC_POLICY_H_

#include <base_libs/node_handle_policy.h>
#include <base_libs/multi_publisher.h>
#include <base_libs/multi_subscriber.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/CvBridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// ## PublisherAdapterStorage for image_transport::Publisher ###########
namespace ros
{

template<>
struct PublisherAdapterStorage<image_transport::Publisher>
{
	image_transport::ImageTransport * image_transport_;
};

// ## PublisherAdapter for image_transport::Publisher ##################

template<class __Message>
class PublisherAdapter<image_transport::Publisher, __Message>
{
public:
	typedef image_transport::Publisher _Publisher;
	
	static _Publisher createPublisher(
		ros::NodeHandle & nh,
		const std::string & topic,
		const unsigned int & cache_size,
		PublisherAdapterStorage<_Publisher> & storage )
	{
		return storage.image_transport_->advertise(
			topic,
			cache_size );
	}
};

// ## SubscriberAdapterStorage for image_transport::Subscriber #########
template<>
struct SubscriberAdapterStorage<image_transport::Subscriber>
{
	image_transport::ImageTransport * image_transport_;
};

// ## SubscriberAdapter for image_transport::Subscriber ################
template<>
class SubscriberAdapter<image_transport::Subscriber>
{
public:
	typedef image_transport::Subscriber _Subscriber;
	
	template<class __Message>
	static _Subscriber createSubscriber(
		ros::NodeHandle & nh,
		const std::string & topic,
		const unsigned int & cache_size,
		const std::function<void( const boost::shared_ptr<__Message const>& )> & callback,
		SubscriberAdapterStorage<_Subscriber> & storage )
	{
		return storage.image_transport_->subscribe(
			topic,
			cache_size,
			boost::function< void( const boost::shared_ptr< __Message const>& )>( callback ) );
	}
};

}

namespace base_libs
{

BASE_LIBS_DECLARE_POLICY( ImageProc, NodeHandlePolicy )

BASE_LIBS_DECLARE_POLICY_CLASS( ImageProc )
{
	BASE_LIBS_MAKE_POLICY_NAME( ImageProc )
protected:
	ros::MultiPublisher<image_transport::Publisher> image_pubs_;
	ros::MultiSubscriber<image_transport::Subscriber> image_subs_;
	
	image_transport::ImageTransport image_transport_;
	
	
	BASE_LIBS_DECLARE_POLICY_CONSTRUCTOR( ImageProc ),
		image_transport_( NodeHandlePolicy::getNodeHandle() )
	{
		printPolicyActionStart( "create", this );
		
		preInit();
		
		printPolicyActionDone( "create", this );
	}
	
	void preInit()
	{
		ros::PublisherAdapterStorage<image_transport::Publisher> publisher_storage;
		ros::SubscriberAdapterStorage<image_transport::Subscriber> subscriber_storage;
		publisher_storage.image_transport_ = &image_transport_;
		subscriber_storage.image_transport_ = &image_transport_;
		
		ros::NodeHandle & nh_rel = NodeHandlePolicy::getNodeHandle();
		
		if( ros::ParamReader<bool, 1>::readParam( nh_rel, "subscribe_to_image", true ) )
			image_subs_.addSubscriber( nh_rel, "image", &ImageProcPolicy::imageCB_0, this, subscriber_storage );
		
		if( ros::ParamReader<bool, 1>::readParam( nh_rel, "show_image", true ) )
			image_pubs_.addPublishers<sensor_msgs::Image>( nh_rel, {"output_image"}, publisher_storage );
	}
	
	virtual IMAGE_PROC_PROCESS_IMAGE( image_ptr )
	{
		/*
		 * Usual behavior: grab an IplImage * or a cvMat from image_ptr and do your thing
		 * When done, if applicable, publish a debug image on the given topic
		 * 
		 * IplImage * image = &IplImage( image_ptr->image );
		 * image_pubs_.publish( image_ptr->toImageMsg(), "output_image" );
		 */
	}
	
	// provided in case a derived class wants to be notified when the
	// raw image comes in, before it's converted
	virtual void imageCB( const sensor_msgs::Image::ConstPtr & image_msg )
	{
		//
	}
	
	void imageCB_0( const sensor_msgs::Image::ConstPtr & image_msg )
	{
		imageCB( image_msg );
		
		cv_bridge::CvImageConstPtr cv_image_ptr;
		
		try
		{
			cv_image_ptr = cv_bridge::toCvShare( image_msg );
		}
		catch (cv_bridge::Exception& e)
		{
			PRINT_ERROR( "cv_bridge exception: %s", e.what() );
			return;
		}
		
		processImage( cv_image_ptr );
	}
	
	// publish specializaiton for sensor_msgs::Image::Ptr
	void publishImages( const std::string & topic, const sensor_msgs::Image::Ptr & image_ptr )
	{
		//PRINT_INFO( "publishing image (CvImageConstPtr) on topic %s", topic.c_str() );
		
		image_pubs_.publish( topic, image_ptr );
	}
	
	// publish specializaiton for cv_bridge::CvImageConstPtr
	void publishImages( const std::string & topic, cv_bridge::CvImageConstPtr & image_ptr )
	{
		//PRINT_INFO( "publishing image (CvImageConstPtr) on topic %s", topic.c_str() );
		
		publishImages( topic, image_ptr->toImageMsg() );
	}
	
	// generic recursive publish with topic/image pairs
	// enabled only if __Rest... is not empty
	template<class __Image, class... __Rest>
	typename std::enable_if<(sizeof...(__Rest) > 0), void>::type
	publishImages( const std::string & topic, __Image & image, __Rest... rest )
	{
		// publish using specialization for __Image
		publishImages( topic, image );
		publishImages( rest... );
	}
	
	sensor_msgs::Image::Ptr fromIplImage( IplImage * image_ptr, std::string frame_id = "" )
	{
		//cv_bridge::CvImage image_wrapper;
		
		//image_wrapper.image = cv::Mat( image_ptr );
		//image_wrapper.encoding = "bgr8";
		//image_wrapper.frame_name = frame_id;
		
		//return image_wrapper;
		
		auto result = sensor_msgs::CvBridge::cvToImgMsg( image_ptr );
		result->header.frame_id = frame_id;
		return result;
	}
	
	//void publishImage( IplImage * image_ptr ){}
		
	
};

}

#endif // BASE_LIBS_BASE_LIBS_IMAGE_PROC_POLICY_H_
