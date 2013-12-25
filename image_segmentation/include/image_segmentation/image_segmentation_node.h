/***************************************************************************
 *  include/image_segmentation/image_segmentation_node.h
 *  --------------------
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Dylan Foster (turtlecannon@gmail.com)
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


#ifndef USCAUV_IMAGESEGMENTATION_IMAGESEGMENTATION
#define USCAUV_IMAGESEGMENTATION_IMAGESEGMENTATION

// ROS
#include <ros/ros.h>

// uscauv
#include <uscauv_common/base_node.h>
#include <uscauv_common/image_transceiver.h>


typedef cv_bridge::CvImage _CvImage;

template<class __Segmentation>
class ImageSegmentationNode: public BaseNode, public ImageTransceiver
{
  
 public:
 ImageSegmentationNode(): BaseNode("ImageSegmentation")
    {
    }

 private:
  __Segmentation segmentation;

 private:

  // Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  void spinFirst()
  {
    addImagePublisher("image_segmented", 1);
    addImageSubscriber("image_color", 1, sensor_msgs::image_encodings::BGR8,
		       &ImageSegmentationNode::imageCallback, this);
    
    segmentation.init();
  }  

  // Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
  {

  }

  void imageCallback( _CvImage::ConstPtr const & msg)
  {
    cv::Mat input = msg->image;

    _CvImage::Ptr output = boost::make_shared<_CvImage>
      (msg->header, sensor_msgs::image_encodings::BGR8 );
    
    output->image = segmentation.segment(input);

    publishImage("image_segmented", output);
  }

};

#endif // USCAUV_IMAGESEGMENTATION_IMAGESEGMENTATION
