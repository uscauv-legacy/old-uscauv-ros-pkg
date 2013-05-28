/***************************************************************************
 *  include/shape_matching/shape_matcher.h
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


#ifndef USCAUV_SHAPEMATCHING_SHAPEMATCHER
#define USCAUV_SHAPEMATCHING_SHAPEMATCHER

// ROS
#include <ros/ros.h>

// uscauv
#include <uscauv_common/base_node.h>
#include <uscauv_common/image_transceiver.h>
#include <uscauv_common/multi_reconfigure.h>

/// opencv
#include <opencv2/imgproc/imgproc.hpp>

/// reconfigure
#include <shape_matching/ShapeMatcherConfig.h>

typedef shape_matching::ShapeMatcherConfig _ShapeMatcherConfig;

class ShapeMatcherNode: public BaseNode, public ImageTransceiver, public MultiReconfigure
{

 public:
 ShapeMatcherNode(): BaseNode("ShapeMatcher")
   {
     
   }

 private:

  // Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  void spinFirst()
     {
       addImagePublisher( "image_denoised", 1);
       addImagePublisher( "image_matched", 1);
       
       addImageSubscriber( "input_image", 1, "mono8", &ShapeMatcherNode::imageCallback, this);
       
       addReconfigureServer<_ShapeMatcherConfig>("image_proc", &ShapeMatcherNode::reconfigureCallback, this);
     }  

  // Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
     {

     }

 public:
  
  void imageCallback( cv_bridge::CvImage::ConstPtr const & msg )
  {
    // Denoise the incoming image #####################################
    unsigned int blur_size = getLatestConfig<_ShapeMatcherConfig>("image_proc").kernel_size;
    blur_size = (blur_size % 2) ? blur_size : blur_size + 1;
    
    /// sensor_msgs::image_encodings::MONO8 = "mono8", for reference
    cv_bridge::CvImage::Ptr denoised = boost::make_shared<cv_bridge::CvImage>( msg->header, sensor_msgs::image_encodings::MONO8 );

    cv::GaussianBlur( msg->image, denoised->image, cv::Size(blur_size, blur_size), 0, 0);
    cv::threshold( denoised->image, denoised->image, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);

    publishImage("image_denoised", denoised, "image_matched", msg);
    
    return;
  }
  
  void reconfigureCallback( _ShapeMatcherConfig const & )
  {
    ROS_INFO("Inside Shape Matcher RC callback." );
    

    return;
  }

};

#endif // USCAUV_SHAPEMATCHING_SHAPEMATCHER
