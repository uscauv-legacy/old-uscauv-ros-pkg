/***************************************************************************
 *  include/uscauv_utilities/static_image_publisher_node.h
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

#ifndef USCAUV_USCAUVUTILITIES_STATICIMAGEPUBLISHERNODE_H
#define USCAUV_USCAUVUTILITIES_STATICIMAGEPUBLISHERNODE_H

/// ROS
#include <ros/ros.h>

/// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv/cxcore.h>

/// ROS images
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>



class StaticImagePublisherNode
{
 private:
  /// publishers and subscribers
  ros::NodeHandle nh_rel_;
  image_transport::ImageTransport img_transport_;
  image_transport::Publisher image_pub_;
  cv_bridge::CvImage image_;
  
  /// parameters
  double loop_rate_hz_;
  
  /// other
  unsigned int image_count_;
  
 public:

  /** 
   * Default Constructor
   * 
   */
 StaticImagePublisherNode()
    :
    nh_rel_("~"),
    img_transport_( nh_rel_ ),
    image_count_(0)  
    {}
    
 private:
  
  /// Running spin() will cause this function to be called before the node begins looping the spingOnce() function.
  void spinFirst()
  {
    /// Get ROS ready ------------------------------------
    img_transport_ = image_transport::ImageTransport( nh_rel_ );
    image_pub_ = img_transport_.advertise( "image_color", 1);
    
    /// Load image ------------------------------------
    std::string image_path;
    if( !nh_rel_.getParam( "image_path", image_path ) )
      {
	ROS_FATAL( "Couldn't find parameter [image_path]." );
	ros::shutdown();
      }
    
    ROS_INFO("Loading image... [ %s ]", image_path.c_str() );
    
    image_.image = cv::imread( image_path.c_str(), CV_LOAD_IMAGE_COLOR );
    
    if (image_.image.data == NULL )
      {
	ROS_FATAL( "Failed to load image.");
	ros::shutdown();
      }
    
    ROS_INFO("Load successful");
    
    image_.encoding = "bgr8";
    image_.header.frame_id = "static_image";
    
    ROS_INFO( "Finished spinning up." );
    return;
  }

  /// Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
  {
    image_.header.stamp = ros::Time::now();
    image_.header.seq = image_count_;
    ++image_count_;
    
    image_pub_.publish( image_.toImageMsg() );
    
    return;
  }

 public:
  
  void spin()
  {
    if( !nh_rel_.getParam("loop_rate", loop_rate_hz_) )
      {
	ROS_WARN("Parameter [loop_rate] not found. Using default.");
	loop_rate_hz_ = 10.0;
      }

    ros::Rate loop_rate( loop_rate_hz_ );

    ROS_INFO( "Spinning up Static Image Publisher..." );
    spinFirst();

    ROS_INFO( "Static Image Publisher is spinning at %.2f Hz.", loop_rate_hz_ ); 

    while( ros::ok() )
      {
	spinOnce();
	ros::spinOnce();
	loop_rate.sleep();
      }
    
    return;
  }

};

#endif // USCAUV_USCAUVUTILITIES_STATICIMAGEPUBLISHERNODE_H
