/***************************************************************************
 *  include/uscauv_common/image_transceiver.h
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


#ifndef USCAUV_USCAUVCOMMON_IMAGETRANSCEIVER
#define USCAUV_USCAUVCOMMON_IMAGETRANSCEIVER

// ROS
#include <ros/ros.h>

/// ROS images
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

/// cpp11
#include <functional>

class ImageTransceiver
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

 ImageTransceiver():
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

    /** 
     * This function uses some fancy template metaprogramming to allow for a few different ways of specifying a callback function for the
     * subscriber. Callbacks can be specified in a few different ways, but must return void and take cv_bridge::CvImage::ConstPtr as their argument.
     * The goal here was to replicate the syntax of the ImageTransport::subscribe() function. Examples:
     * Standard C-Style function: addImageSubscriber( ..., ..., &callbackFunction )
     * C++ Member function: addImageSubscriber( ..., ..., &ClassType::callbackFunction, &ClassInstance )
     * std::function object:, addImageSubscriber, ..., ..., FunctionObject )
     * 
     * @param topic_rel Name of the topic that we will subscribe to, relative to node namespace
     * @param queue_size Size of incoming image queue
     * @param encoding Encoding used to interpret incoming images. If it is set to "" or omitted, the encoding in the image header will be used.
     * @param cb_args Callback function arguments as described above
     * 
     */

  template <class... __FuncArgs>
    typename std::enable_if< (sizeof...(__FuncArgs) > 0), void>::type
    addImageSubscriber( std::string const & topic_rel, uint32_t const & queue_size, std::string const & encoding, __FuncArgs&&... cb_args )
  {
    /// Get full subscriber topic. 
    std::string const & topic_resolved = nh_rel_.resolveName( topic_rel, true);
    
    ROS_INFO( "Creating image subscriber [ %s ] with encoding [ %s ]... ", topic_resolved.c_str(), 
	      ( encoding == "" ) ? "from source" : encoding.c_str() );
    
    _NamedSubscriberMap::iterator sub_it = subscribers_.find( topic_resolved );

    if ( sub_it != subscribers_.end() )
      {
	ROS_WARN("Add subscriber [ %s ] requested, but this subscriber already exists. Overwriting...", topic_resolved.c_str() );
	sub_it->second.shutdown();
      }
    
    /// Bind the callback arguments to a single std::function< void( cv_bridge::CvImage::ConstPtr )> object.
    callbacks_[ topic_resolved ] = std::bind( std::forward<__FuncArgs>(cb_args)... , std::placeholders::_1);

    /**
     * Subscriber callbacks will call ImageTransceiver::imageCallback with topic name, which will then look up the correct external callback and call it.
     * Need to use boost instead of cpp11 for this section because the ImageTransport::subscribe() call expects it
     */
    boost::function<void( sensor_msgs::ImageConstPtr const & )> sub_cb = boost::bind( &ImageTransceiver::imageCallback, this, topic_resolved, encoding, _1 );
    
    subscribers_[ topic_resolved ] = image_transport_.subscribe( topic_rel, queue_size, sub_cb );
    
    ROS_INFO("Created subscriber successfully.");

    return;
  }

  /** 
   * Same function as above, but with default image encoding. When image encoding is set to "",
   * the encoding of the incoming image is used.
   * 
   */
  template <class __Arg, class... __FuncArgs>
    typename std::enable_if< (sizeof...(__FuncArgs) > 0) && 
    !std::is_convertible< __Arg, std::string>::value, void>::type
    addImageSubscriber( std::string const & topic_rel, uint32_t const & queue_size, __Arg&& arg, __FuncArgs&&... cb_args )
  {
    
    addImageSubscriber( topic_rel, queue_size, std::string(), std::forward<__Arg>(arg), std::forward<__FuncArgs>(cb_args)... );
    return;
  }

 public:

  /** 
   * This function, along with some of the code in addImageSubscriber is responsible
   * for intercepting image callbacks from image_transport::Subscriber class,
   * performing a sensor_msgs::Image -> cv_bridge::CvImage conversion that would
   * otherwise need to be performed by the user, and forwarding the results to a user-defined
   * external callback.
   * Note that this implicitly converts to the bgr8 encoding, which may not always be correct
   * 
   * @param topic_resolved Global path to the topic that this callback corresponds to 
   * @param msg Image message to be forwarded to an external callback
   */
  void imageCallback( std::string const & topic_resolved, std::string const & encoding, sensor_msgs::ImageConstPtr const & msg)
  {
    /// Convert to cv_bridge::CvImage ------------------------------------

    cv_bridge::CvImageConstPtr cv_ptr;

    try
      {
	cv_ptr = cv_bridge::toCvCopy(msg, encoding);
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

    /** 
     * For a variable number of topic/image pairs, call one of the above publishImage functions for each pair
     * e.g. publishImage( "topic1", image1, "topic2", image2, ...)
     * 
     * @param topic_rel First topic name, relative node namespace
     * @param image First image to publish
     * @param args The rest of the topics and images
     * 
     */
  template <class __Image, class... __Args>
    typename std::enable_if< (sizeof...(__Args) % 2) == 0 && (sizeof...(__Args) > 0), void>::type 
    publishImage( std::string const & topic_rel, __Image const & image, __Args && ...args)
  {
    publishImage( topic_rel, image);
    publishImage( std::forward<__Args>(args)... );
    return;
  }
  
};

#endif // USCAUV_USCAUVCOMMON_IMAGETRANSCEIVER
