/***************************************************************************
 *  include/uscauv_common/image_tranceiver.h
 *  --------------------
 *
 *  Copyright (c) 2013, Dylan Foster
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
 *  * Neither the name of USC AUV nor the names of its
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


#ifndef USCAUV_USCAUVCOMMON_IMAGETRANCEIVER
#define USCAUV_USCAUVCOMMON_IMAGETRANCEIVER

// ROS
#include <ros/ros.h>

/// ROS images
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

/// cpp11
#include <mutex>
#include <functional>

class ImageTranceiver
{
 private:
  typedef std::function<void( cv_bridge::CvImage::ConstPtr const & )> _ImageCBFunction;

  typedef std::map<std::string, image_transport::Publisher> _NamedPublisherMap;
  typedef std::map<std::string, image_transport::Subscriber> _NamedSubscriberMap;
  typedef std::map<std::string,  _ImageCBFunction> _NamedCallbackMap;
  
  ros::NodeHandle nh_rel_;
  image_transport::ImageTransport image_transport_;
  _NamedPublisherMap  publishers_;
  _NamedSubscriberMap subscribers_;
  _NamedCallbackMap callbacks_;

 public:

 ImageTranceiver():
  nh_rel_("~"),
  image_transport_( nh_rel_ )
  {}

 protected:

  void addImagePublisher( std::string const & topic_rel, uint32_t const & queue_size, bool const & latch = false)
  {
    /// Get full publisher topic. 
    std::string const & topic_resolved = nh_rel_.resolveName( topic_rel, true);
    
    ROS_INFO( "Creating image publisher [ %s ]... ", topic_resolved.c_str() );
    
    _NamedPublisherMap::iterator pub_it = publishers_.find( topic_resolved );

    if ( pub_it != publishers_.end() )
      {
	ROS_WARN("Add publisher [ %s ] requested, but this publisher already exists. Overwriting...", topic_resolved.c_str() );
	pub_it->second.shutdown();
      }

    publishers_[ topic_resolved ] = image_transport_.advertise( topic_rel, queue_size, latch );
    
    ROS_INFO("Created publisher successfully.");

    return;
  }

  void addImageSubscriber( std::string const & topic_rel, uint32_t const & queue_size, _ImageCBFunction const & image_cb )
  {
    /// Get full subscriber topic. 
    std::string const & topic_resolved = nh_rel_.resolveName( topic_rel, true);
    
    ROS_INFO( "Creating image subscriber [ %s ]... ", topic_resolved.c_str() );
    
    _NamedSubscriberMap::iterator sub_it = subscribers_.find( topic_resolved );

    /// TODO: Make sure that callback also gets replaced if the topic has already been subscribed to
    if ( sub_it != subscribers_.end() )
      {
	ROS_WARN("Add subscriber [ %s ] requested, but this subscriber already exists. Overwriting...", topic_resolved.c_str() );
	sub_it->second.shutdown();
      }
    
    callbacks_[ topic_resolved ] = image_cb;

    /**
     * Subscriber callbacks will call ImageTranceiver::imageCallback with topic name, which will then look up the correct external callback and call it.
     * Need to use boost instead of cpp11 for this section because the ImageTransport::subscribe() call expects it
     */
    boost::function<void( sensor_msgs::ImageConstPtr const & )> sub_cb = boost::bind( &ImageTranceiver::imageCallback, this, topic_resolved, _1 );
    
    subscribers_[ topic_resolved ] = image_transport_.subscribe( topic_rel, queue_size, sub_cb); 
    
    ROS_INFO("Created subscriber successfully.");

    return;
  }

 public:

  /** 
   * This function, along with some of the code in addImageSubscriber is responsible
   * for intercepting image callbacks from image_transport::Subscriber class,
   * performing a redundant sensor_msgs::Image -> cv_bridge::CvImage conversion that would
   * otherwise need to be performed by the user, and forwarding the results to a user-defined
   * external callback.
   * Note that this implicitly converts to the bgr8 encoding, which may not always be correct
   * 
   * @param topic_resolved Global path to the topic that this callback corresponds to 
   * @param msg Image message to be forwarded to an external callback
   */
  void imageCallback( std::string const & topic_resolved, sensor_msgs::ImageConstPtr const & msg)
  {
    /// Convert to cv_bridge::CvImage ------------------------------------

    cv_bridge::CvImageConstPtr cv_ptr;

    try
      {
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
    catch (cv_bridge::Exception& e)
      {
	ROS_ERROR("[ %s ] cv_bridge exception: %s", topic_resolved.c_str(), e.what());
	return;
      }

    /// Call the external image callback ------------------------------------

    if ( callbacks_.find( topic_resolved ) == callbacks_.end() )
      {
	ROS_WARN( "[ %s ] image callback is not registered.", topic_resolved.c_str() );
	return;
      }

    /// Perform the function call
    callbacks_[ topic_resolved ]( cv_ptr );
    
    return;
  }

  void publishImage(std::string const & topic_rel, sensor_msgs::ImagePtr const & image ) const
  {
    /// Get full publisher topic. 
    std::string const & topic_resolved = nh_rel_.resolveName( topic_rel, true);

    _NamedPublisherMap::const_iterator pub_it = publishers_.find( topic_resolved );

    if ( pub_it == publishers_.end() )
      {
	ROS_WARN("Requested publish topic [ %s ] has not been advertised.", topic_resolved.c_str() );
	return;
      }
    
    pub_it->second.publish( image );
    return;
  }

  void publishImage(std::string const & topic_rel, cv_bridge::CvImage::ConstPtr const & image ) const
  {
    publishImage( topic_rel, image->toImageMsg() );
  }

  template <class __Image, class... __Args>
    typename std::enable_if< (sizeof...(__Args) % 2) == 0 && (sizeof...(__Args) > 0), void>::type publishImage( std::string const & topic_rel, __Image const & image, __Args && ...args)
  {
    publishImage( topic_rel, image);
    publishImage( std::forward<__Args>(args)... );
    return;
  }
  
};

#endif // USCAUV_USCAUVCOMMON_IMAGETRANCEIVER
