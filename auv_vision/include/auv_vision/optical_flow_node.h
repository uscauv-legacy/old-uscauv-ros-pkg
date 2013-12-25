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
#include <uscauv_common/RANSACConfig.h>

/// reconfigure
#include <auv_vision/OpticalFlowConfig.h>

/// opencv2
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>

/// cpp11
#include <random>

/// Arbitrary
#define USCAUV_AUVVISION_OPTICALFLOW_MAXFEATURES 1024

typedef cv_bridge::CvImage _CvImage;
typedef std::vector< cv::Point2f > _FeatureVector;
typedef std::vector<unsigned char> _StatusVector;

typedef auv_vision::OpticalFlowConfig _OpticalFlowConfig;
typedef uscauv_common::RANSACConfig _RANSACConfig;

class OpticalFlowNode: public BaseNode, public ImageTransceiver, public MultiReconfigure
{
 private:
  cv::Mat prev_image_;
  _FeatureVector prev_features_;
  bool ready_;
  
  _OpticalFlowConfig* of_config_;
  _RANSACConfig*      ransac_config_;
  
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
       
       addReconfigureServer<_OpticalFlowConfig>("image_proc");
       addReconfigureServer<_RANSACConfig>("RANSAC");
       /// note: won't be able to do this in a multithreaded node without mutexes
       of_config_     = &getLatestConfig<_OpticalFlowConfig>("image_proc");
       ransac_config_ = &getLatestConfig<_RANSACConfig>("RANSAC");

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
    
    double const & feature_quality = of_config_->feature_quality;
    unsigned int const & feature_radius = of_config_->feature_distance / 2;

    // ################################################################
    // Get down to business (aka optical flow) ########################
    // ################################################################

    cv::goodFeaturesToTrack( msg->image , new_features, 
			     USCAUV_AUVVISION_OPTICALFLOW_MAXFEATURES,
			     feature_quality, 2*feature_radius);

    /// Deal with the case where no features are found
    if( !new_features.size() )
      {
	ready_ = false;
	return;
      }

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
    _FeatureVector velocity_vectors;
    
    cv_bridge::CvImage::Ptr output = 
      boost::make_shared<cv_bridge::CvImage>
      ( msg->header, sensor_msgs::image_encodings::BGR8 );

    /// this function can process the image in place
    cv::cvtColor( new_image, output->image, CV_GRAY2BGR );

    cv::Size output_size = output->image.size();
    
    cv::Point2f output_center( output_size.width/2, output_size.height/2);
    
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
	
	velocity_vectors.push_back( *matched_it - *previous_it );

	/// ray between estimate and original feature
	cv::line( output->image, *previous_it, *matched_it,
		  cv::Scalar(200, 200, 200), 1);
	
      }

    // ################################################################
    // Do RANSAC ######################################################
    // ################################################################

    _FeatureVector ransac_vectors;
    double ransac_mse;
    cv::Point2f ransac_velocity;
    
    /// text params
    std::string const status_text = "RANSAC: ";
    int const font = cv::FONT_HERSHEY_SIMPLEX;
    double const font_scale = 0.5;
    int const font_thickness = 1.5;
    int baseline = 0;
    cv::Point text_origin( 5, 20 );
    cv::Size status_size = cv::getTextSize( status_text, font, font_scale, 
					    font_thickness, &baseline );

    cv::putText( output->image, status_text, text_origin, 
		 font, font_scale, uscauv::CV_GREEN_BGR,
		 font_thickness );
    
    if( RANSAC( velocity_vectors, ransac_vectors, ransac_mse, ransac_velocity ))
      {
	ROS_DEBUG("RANSAC failed.");
	cv::putText( output->image, "Failure", text_origin + cv::Point( status_size.width, 0), 
		     font, font_scale, uscauv::CV_RED_BGR,
		     font_thickness );
		     
      }
    else
      {
	std::stringstream vel, mse;
	vel << "Velocity: (" << ransac_velocity.x << ", " << ransac_velocity.y << ")";
	mse << "MSE: " << ransac_mse;

	ROS_DEBUG("RANSAC: ( %.4f, %.4f ) with MSE %.4f. In: %lu, Out: %lu.",
		 ransac_velocity.x, ransac_velocity.y, ransac_mse,
		 velocity_vectors.size(), ransac_vectors.size() );
	cv::putText( output->image, "Good", text_origin + cv::Point( status_size.width, 0), 
		     font, font_scale, uscauv::CV_GREEN_BGR,
		     font_thickness );
	cv::putText( output->image, vel.str(), text_origin + cv::Point( 0, status_size.height+10), 
		     font, font_scale, uscauv::CV_GREEN_BGR,
		     font_thickness );
	cv::putText( output->image, mse.str(), text_origin + cv::Point( 0, 2*(status_size.height+10)), 
		     font, font_scale, uscauv::CV_GREEN_BGR,
		     font_thickness );


      }
    cv::line( output->image, output_center, ransac_velocity + output_center,
	      uscauv::CV_GREEN_BGR, 3);

    publishImage( "image_debug", output );

    /// update previous frame
    new_image.copyTo( prev_image_ );
    prev_features_ = new_features;

    return;
  }
  
  int RANSAC( _FeatureVector const & in, _FeatureVector & out, 
	       double & error, cv::Point2f & model )
  {
    error = 0.0;
    model = cv::Point2f(0, 0);
    out = _FeatureVector();
    
    unsigned int const n = ransac_config_->sample_size;
    unsigned int const k = ransac_config_->max_iterations;
    unsigned int const d = ransac_config_->min_inliers;
    double const t = ransac_config_->inlier_max_error;

    if( in.size() < n )
      {
	ROS_DEBUG("RANSAC input is smaller than sample size. Quitting...");
	return -1;
      }
    
    bool error_init = false;
    
    for(unsigned int i = 0; i < k; ++i)
      {
	_FeatureVector  maybe_inliers, rest;
	getSamplePoints( in, n, maybe_inliers, rest );
	cv::Point2f const maybe_model = cvPointMean( maybe_inliers );
	_FeatureVector consensus_set = maybe_inliers;
	
	for( _FeatureVector::const_iterator rest_it = rest.begin();
	     rest_it != rest.end(); ++rest_it)
	  {
	    if( cv::norm( maybe_model - *rest_it ) < t )
	      consensus_set.push_back( *rest_it );
	  }
	
	if( consensus_set.size() > d )
	  {
	    cv::Point2f consensus_model = cvPointMean( consensus_set );
	    double model_error = cvPointMSE( consensus_set, consensus_model );
	    
	    if( !error_init || model_error < error )
	      {
		error_init = true;
		error = model_error;
		model = consensus_model;
		out = consensus_set;
	      }
	  }
      }

    return (error_init) ? 0 : -1;
    
  }
  
  cv::Point2f cvPointMean( _FeatureVector const & in )
    {
      cv::Point2f sum( 0.0f, 0.0f);
      
      for( _FeatureVector::const_iterator point_it = in.begin();
	   point_it != in.end(); ++point_it)
	{
	  sum += *point_it;
	}

      return sum *= double(1.0L / in.size());
    }

  double cvPointMSE( _FeatureVector const & in, cv::Point2f const & point )
    {
      double sum = 0;
      
      for( _FeatureVector::const_iterator point_it = in.begin();
	   point_it != in.end(); ++point_it)
	{
	  sum += pow(cv::norm(*point_it - point), 2);
	}

      return sum *= double(1.0L / (in.size() -1));
    }

  /// TODO: Switch rest to a list. Erasing elements is inefficient with vectors
  _FeatureVector getSamplePoints(_FeatureVector const & in, unsigned int const & num, _FeatureVector & sample_points, _FeatureVector & rest)
    {
      rest = in;
      
      std::default_random_engine generator;
      std::uniform_int_distribution<int>  distribution(0, in.size() - 1);

      for(unsigned int i = 0; i < num; ++i)
	{
	  int const n = distribution(generator);
	  
	  sample_points.push_back( rest[ n ] );
	  rest.erase( rest.begin() + n );
	}
      
      return sample_points;
    }

};

#endif // USCAUV_AUVVISION_OPTICALFLOW
