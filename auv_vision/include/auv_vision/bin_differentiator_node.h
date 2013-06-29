/***************************************************************************
 *  include/auv_vision/bin_differentiator_node.h
 *  --------------------
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Francesca Nannizzi
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


#ifndef USCAUV_AUVVISION_BINDIFFERENTIATOR
#define USCAUV_AUVVISION_BINDIFFERENTIATOR

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>

// uscauv
#include <uscauv_common/base_node.h>
#include <uscauv_common/image_transceiver.h>
#include <uscauv_common/multi_reconfigure.h>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>

// Tesseract-OCR
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>

// C++
#include "sys/time.h"

typedef cv_bridge::CvImage _CvImage;

class BinDifferentiatorNode: public BaseNode, public ImageTransceiver, public MultiReconfigure
{
  
 public:
  BinDifferentiatorNode(): BaseNode("BinDifferentiator"){}

 private:
  tesseract::TessBaseAPI *ocr_;
  ros::NodeHandle nh_rel_;
  ros::Publisher ocr_text_pub_;
  ros::Publisher ocr_confidence_pub_;

  // Parameters
  double loop_rate_hz;

  // Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  void spinFirst()
  {       
    // Create the image subscriber
    addImageSubscriber("/static_image_publisher/image_color", 
		       1, 
		       sensor_msgs::image_encodings::MONO8, 
		       &BinDifferentiatorNode::imageCallback,
		       this);

    // Create the publishers for text results and confidences
    ocr_text_pub_ =  nh_rel_.advertise<std_msgs::String>("ocr_text", 1000);
    ocr_confidence_pub_ = nh_rel_.advertise<std_msgs::Int32MultiArray>("ocr_confidence", 1000);
 
    // Initialize tesseract baseAPI class
    ocr_ = new tesseract::TessBaseAPI();

    ROS_INFO("Tesseract-ocr version: %s \n", ocr_->Version());
       
    // Confirm that tesseract has initialized properly
    if(ocr_->Init(NULL, "eng")) 
    {
      ROS_ERROR("Could not initialize tesseract. \n");
      return;
    }
  }  

  // Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
  {		
    return;
  }

  void imageCallback( _CvImage::ConstPtr const & msg )
  {
    // Feed the ocr an image
    ocr_->SetImage(msg->image.data,
		  msg->image.cols,
		  msg->image.rows,
		  (msg->image.step[0]/msg->image.cols), 
		  msg->image.step[0]);
	
    // Get the text output from the ocr
    char* output = ocr_->GetUTF8Text();

    // Publish the ocr text output
    std_msgs::String text_output_msg;
    std::stringstream ss;
    ss << output;
    text_output_msg.data = ss.str();
    ocr_text_pub_.publish(text_output_msg);
   
    // Get the confidence of each word from the ocr
    int* confidences = ocr_->AllWordConfidences();
    int i = 0;

    // Publish the confidences
    std_msgs::Int32MultiArray confidence_output_msg;
    while(confidences[i] != -1)
    {
      confidence_output_msg.data.push_back(confidences[i]);
      i++;
    }
    ocr_confidence_pub_.publish(confidence_output_msg);
    
    // Clear ocr results
    ocr_->Clear();
    delete output;
    delete confidences;

    return;

  }
};

#endif // USCAUV_AUVVISION_BINDIFFERENTIATOR
