/***************************************************************************
 *  include/color_classification/color_classifier_node.h
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

#ifndef USCAUV_COLORCLASSIFICATION_COLORCLASSIFIERNODE_H
#define USCAUV_COLORCLASSIFICATION_COLORCLASSIFIERNODE_H

/// ROS
#include <ros/ros.h>

/// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv/cxcore.h>

/// ROS images
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

/// xmlrpcpp
#include <XmlRpcValue.h>

/// Color Classifier
#include <color_classification/svm_color_classifier.h>


/* typedef std::vector<image_transport::Publisher> _ImgPublisherArray; */
typedef std::map<std::string, image_transport::Publisher> _ColorPublisherMap;


class ColorClassifierNode
{
 private:
  /// publishers and subscribers
  ros::NodeHandle nh_rel_;
  image_transport::ImageTransport img_transport_;
  image_transport::Subscriber image_sub_;
  _ColorPublisherMap classified_image_pub_;

  /// parameters
  double loop_rate_hz_;
  
  /// color classification
  SvmColorClassifier color_classifier_;
  std::vector<std::string> color_names_;
  
 public:

  /** 
   * Default Constructor
   * 
   */
  ColorClassifierNode()
    :
    nh_rel_("~"),
    img_transport_( nh_rel_ )
    {}
    
 private:
  
  /// Running spin() will cause this function to be called before the node begins looping the spingOnce() function.
  void spinFirst()
  {
    /// Get ROS ready ------------------------------------
    ros::NodeHandle nh;
    img_transport_ = image_transport::ImageTransport( nh_rel_ );
    
    /// Load SVMs ------------------------------------
    XmlRpc::XmlRpcValue xml_colors;
    if( !nh.getParam( "model/colors", xml_colors ) )
      {
	ROS_FATAL( "Couldn't find parameter [colors]." );
	ros::shutdown();
      }
    
    /// TODO: check for errors
    color_classifier_.fromXmlRpc( xml_colors );

    /// Advertise classified image topics ------------------------------------
    color_names_ = color_classifier_.getColorNames();
    
    for(std::vector<std::string>::const_iterator color_it = color_names_.begin(); color_it != color_names_.end(); ++color_it )
      {
	ROS_INFO( "Creating publisher... [ %s ]", color_it->c_str() );
	
	image_transport::Publisher color_pub;

	/// We will publish each color classified image to a topic called <color_name>_classified.
	color_pub = img_transport_.advertise( *color_it + "_classified", 1);
	
	classified_image_pub_[ *color_it ] = color_pub;
	
	ROS_INFO( "Created publisher successfully." );
      }
	  
    /// Subscribe to input image topic ------------------------------------
    image_sub_ = img_transport_.subscribe( "image_color", 1, &ColorClassifierNode::imageCallback, this);

    
    ROS_INFO( "Finished spinning up." );
    return;
  }

  /// Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
  {
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

    ROS_INFO( "Spinning up Color Classifier..." );
    spinFirst();

    ROS_INFO( "Color Classifier is spinning at %.2f Hz.", loop_rate_hz_ ); 

    while( ros::ok() )
      {
	spinOnce();
	ros::spinOnce();
	loop_rate.sleep();
      }
    
    return;
  }

 private:

  /** 
   * For each color, classify the incoming image and publish the results
   * 
   * @param msg Color Image
   */
  void imageCallback(const sensor_msgs::ImageConstPtr & msg)
  {
    cv_bridge::CvImagePtr cv_ptr;

    try
      {
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
    catch (cv_bridge::Exception& e)
      {
	ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
      }
    
    for ( _ColorPublisherMap::iterator color_it = classified_image_pub_.begin(); color_it != classified_image_pub_.end(); ++color_it )
      {
	/// New image with the same header as the input image
	cv_bridge::CvImage classified_image( cv_ptr->header , "bgr8");
	
	if ( color_classifier_.classify( color_it->first, cv_ptr->image, classified_image.image) )
	  {
	    ROS_WARN("Failed to classify color [ %s ].", color_it->first.c_str() );
	    continue;
	  }
	
	color_it->second.publish( classified_image.toImageMsg() );
	
      }

    return;
  }

};

#endif // USCAUV_COLORCLASSIFICATION_COLORCLASSIFIERNODE_H
