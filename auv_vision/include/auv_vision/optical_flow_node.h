/***************************************************************************
 *  include/auv_vision/optical_flow_node.h
 *  --------------------
 *
 *  Software License Agreement (BSD License)
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


#ifndef USCAUV_AUVVISION_OPTICALFLOW
#define USCAUV_AUVVISION_OPTICALFLOW

// ROS
#include <ros/ros.h>

// uscauv
#include <uscauv_common/base_node.h>
#include <uscauv_common/image_transceiver.h>
#include <uscauv_common/multi_reconfigure.h>
#include <uscauv_common/graphics.h>

/// reconfigure
#include <auv_vision/OpticalFlowConfig.h>

/// opencv2
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>

#define USCAUV_AUVVISION_OPTICALFLOW_MAXFEATURES 1024

typedef cv_bridge::CvImage _CvImage;
typedef std::vector< cv::Point2f > _FeatureVector;
typedef std::vector<unsigned char> _StatusVector;

using namespace auv_vision;

class OpticalFlowNode: public BaseNode, public ImageTransceiver, public MultiReconfigure
{
 private:
  cv::Mat prev_image_;
  _FeatureVector prev_features_;
  bool ready_;
  
  OpticalFlowConfig* config_;
  
 public:
 OpticalFlowNode(): BaseNode("OpticalFlow"),
    ready_( false )
   {
   }

 private:

  // Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  void spinFirst()
     {
       addImagePublisher("image_debug", 1);
       
       addImageSubscriber("image_mono", 1, sensor_msgs::image_encodings::MONO8,
			  &OpticalFlowNode::monoCallback, this);

       addReconfigureServer<OpticalFlowConfig>("image_proc");
       /// note: won't be able to do this in a multithreaded node without a mutex
       config_ = &getLatestConfig<OpticalFlowConfig>("image_proc");

     }

  // Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
     {

     }

  void monoCallback( _CvImage::ConstPtr const & msg )
  {
    /// TODO: Store previous image header so that dt can be calculated
    
    cv::Mat new_image = msg->image;
    _FeatureVector new_features;
    _FeatureVector matched_features;
    _StatusVector match_status;
    std::vector<float> err;
    
    double const & feature_quality = config_->feature_quality;
    unsigned int const & feature_radius = config_->feature_distance / 2;

    // ################################################################
    // Get down to business ###########################################
    // ################################################################

    cv::goodFeaturesToTrack( msg->image , new_features, 
			     USCAUV_AUVVISION_OPTICALFLOW_MAXFEATURES,
			     feature_quality, 2*feature_radius);

    if( !ready_ )
      {
	new_image.copyTo( prev_image_ );
	prev_features_ = new_features;
	ready_ = true;
	return;
      }
    
    /// TODO: Play with OPTFLOW_USE_INITIAL_FLOW flag
    /// TODO: More variable arguments
    cv::calcOpticalFlowPyrLK( prev_image_, new_image, prev_features_, matched_features,
			      match_status, err );



    // ################################################################
    // Draw output image and publish ##################################
    // ################################################################

    cv_bridge::CvImage::Ptr output = 
      boost::make_shared<cv_bridge::CvImage>
      ( msg->header, sensor_msgs::image_encodings::BGR8 );

    /// this function can process the image in place
    cv::cvtColor( new_image, output->image, CV_GRAY2BGR );
    
    _FeatureVector::const_iterator matched_it = matched_features.begin();
    _StatusVector::const_iterator status_it = match_status.begin();

    for( _FeatureVector::const_iterator previous_it = prev_features_.begin();
	 previous_it != prev_features_.end(); 
	 ++previous_it, ++matched_it, ++status_it )
      {
	/// set circle radius so that the closest circles are tangent

	if( ! *status_it )
	  {
	    /// draw features that couldn't be matched
	    cv::circle( output->image, *previous_it,
			feature_radius, uscauv::CV_PINK_BGR, 2);
	    continue;
	  }

	/// draw features that were matched
	cv::circle( output->image, *previous_it,
		    feature_radius, uscauv::CV_DENIM_BGR, 2);
	
	/// draw estimate
	/* cv::circle( output->image, *matched_it, */
	/* 	    feature_radius, uscauv::CV_LIME_BGR, 2); */

	/// ray between estimate and original feature
	cv::line( output->image, *previous_it, *matched_it,
		  cv::Scalar(200, 200, 200), 1);
	
      }

    

    publishImage( "image_debug", output );

    /// update previous frame
    new_image.copyTo( prev_image_ );
    prev_features_ = new_features;

    return;
  }


};

#endif // USCAUV_AUVVISION_OPTICALFLOW
