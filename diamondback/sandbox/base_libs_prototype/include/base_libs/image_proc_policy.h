#ifndef BASE_LIBS_BASE_LIBS_IMAGE_PROC_POLICY_H_
#define BASE_LIBS_BASE_LIBS_IMAGE_PROC_POLICY_H_

#include <base_libs/node_handle_policy.h>
#include <base_libs/multi_publisher.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/CvBridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// specializations for image_transport::Publisher
namespace ros
{

template<>
struct PublisherAdapterStorage<image_transport::Publisher>
{
	image_transport::ImageTransport * image_transport_;
};

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

}

namespace base_libs
{

BASE_LIBS_DECLARE_POLICY( ImageProc, NodeHandlePolicy )

BASE_LIBS_DECLARE_POLICY_CLASS( ImageProc )
{
	BASE_LIBS_MAKE_POLICY_NAME( ImageProc )
protected:
	ros::MultiPublisher<image_transport::Publisher> image_publishers_;
	
	image_transport::ImageTransport image_transport_;
	image_transport::Subscriber image_sub_;
	
public:
	ImageProcPolicy( ros::NodeHandle & nh )
	:
		_ImageProcPolicyAdapterType( nh ),
		image_transport_( nh_rel_ ),
		image_sub_( image_transport_.subscribe( nh_rel_.resolveName( "image" ), 1, &ImageProcPolicy::imageCB_0, this ) )
	{
		printPolicyActionStart( "create", this );
		ros::PublisherAdapterStorage<image_transport::Publisher> storage;
		storage.image_transport_ = &image_transport_;
		
		if( ros::ParamReader<bool, 1>::readParam( nh_rel_, "show_image", true ) )
			image_publishers_.addPublishers<sensor_msgs::Image>( nh_rel_, {"output_image"}, storage );
			
		printPolicyActionDone( "create", this );
	}
	
	virtual IMAGE_PROC_PROCESS_IMAGE( image_ptr )
	{
		/*
		 * Usual behavior: grab an IplImage * or a cvMat from image_ptr and do your thing
		 * When done, if applicable, publish a debug image on the given topic
		 * 
		 * IplImage * image = &IplImage( image_ptr->image );
		 * image_publishers_.publish( image_ptr->toImageMsg(), "output_image" );
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
			ROS_ERROR( "cv_bridge exception: %s", e.what() );
			return;
		}
		
		processImage( cv_image_ptr );
	}
	
	void publishImage(){}
	
	template<class... __Topics>
	void publishImage( cv_bridge::CvImageConstPtr & image_ptr, std::string topic, __Topics... topics )
	{
		image_publishers_.publish( topic, image_ptr->toImageMsg() );
		publishImage( topics... );
	}
	
	template<class... __Topics>
	void publishImage( IplImage * image_ptr, std::string topic, __Topics... topics )
	{
		cv_bridge::CvImage image_wrapper;
		image_wrapper.image = cv::Mat( image_ptr );
		
		image_publishers_.publish( topic, image_wrapper.toImageMsg() );
		publishImage( topics... );
	}
	
	//void publishImage( IplImage * image_ptr ){}
		
	
};

}

#endif // BASE_LIBS_BASE_LIBS_IMAGE_PROC_POLICY_H_
