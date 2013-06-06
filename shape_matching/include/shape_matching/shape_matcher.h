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
#include <uscauv_common/image_loader.h>
#include <uscauv_common/graphics.h>

/// opencv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/// reconfigure
#include <shape_matching/ShapeMatcherConfig.h>

/// messages
#include <auv_msgs/MatchedShape.h>
#include <auv_msgs/MatchedShapeArray.h>

typedef shape_matching::ShapeMatcherConfig _ShapeMatcherConfig;

typedef auv_msgs::MatchedShape      _MatchedShape;
typedef auv_msgs::MatchedShapeArray _MatchedShapeArray;

typedef std::vector<cv::Point2i> _Contour;
typedef std::vector<cv::Point2f> _Contour2f;
typedef cv::Mat                  _Signature;

#define DEBUG_SIZE(X, __String)			\
  ROS_INFO("%s has rows: %d, cols: %d, channels: %d", __String, (X).rows, (X).cols, (X).channels() );

struct ContourData
{
  _Contour2f contour_;   /// original contour in cartesian, for drawing later  (normalized)
  _Signature signature_; /// radial histogram for EMD, see Rubner EMD paper
  cv::Point2f mean_;
  cv::Mat eigenvec_;
  cv::Mat eigenval_;
  double radius_; /// radius of bounding circle
  double rotation_; /// rotation from XY in radians (right-handed)
  
};

typedef std::map<std::string, ContourData> _NamedContourData;
typedef std::map<std::string, _Contour> _NamedContourMap;

class ShapeMatcherNode: public BaseNode, public ImageTransceiver, public MultiReconfigure
{
 private:
  typedef uscauv::ImageLoader _ImageLoader;

  _NamedContourData templates_;
  _NamedContourMap template_contours_;
  _ImageLoader template_images_;
  _ShapeMatcherConfig* config_;
  
  /// ros interfaces
  ros::Publisher match_pub_;
  ros::NodeHandle nh_rel_;
  
  /// cost matrix for EMD algorithm
  cv::Mat emd_cost_;

 public:
 ShapeMatcherNode(): BaseNode("ShapeMatcher"), nh_rel_("~")
    {
      
    }

 private:

  // Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  void spinFirst()
     {
       addImagePublisher( "image_denoised", 1);
       addImagePublisher( "image_contours", 1);
       addImagePublisher( "image_matched", 1);
       
       addImageSubscriber( "image_mono", 1, "mono8", &ShapeMatcherNode::imageCallback, this);
       
       /// TODO: Make a MultiPublisher class to make this a little nice
       match_pub_ = nh_rel_.advertise<_MatchedShapeArray>("matched_shapes", 10);

       
       /// relative to global namespace, not node namespace
       if(template_images_.loadImagesAt("model/shapes", CV_LOAD_IMAGE_GRAYSCALE ))
	 ROS_ERROR("Failed to load shape templates.");

    for(_ImageLoader::const_iterator template_it = template_images_.begin();
	template_it != template_images_.end(); ++template_it)
      {
	ROS_INFO("Analyzing template contours [ %s ]...", template_it->first.c_str() );
	
	std::vector<std::vector<cv::Point2i> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours( template_it->second, contours, hierarchy, 
			  CV_RETR_TREE, CV_CHAIN_APPROX_NONE );
	
	if( contours.size() == 0)
	  {
	    ROS_WARN("No contours found. Skipping...");
	    continue;
	  }
	else if( contours.size() > 1)
	  {
	    ROS_WARN("Only single-contour templates are supported. Skipping...");
	    continue;
	  }
	
	template_contours_[ template_it->first ] = contours[0];
	ROS_INFO("Analysis successful.");
      }
    
    /// This needs to go after the template loading part so that contours are available when
    /// reconfigurecallback is first called.
    addReconfigureServer<_ShapeMatcherConfig>("image_proc", &ShapeMatcherNode::reconfigureCallback, this);
    config_ = &getLatestConfig<_ShapeMatcherConfig>("image_proc");

     }  

  // Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
     {

     }

 public:
  
  void imageCallback( cv_bridge::CvImage::ConstPtr const & msg )
  {
    
    // ################################################################
    // Apply a gaussian blur and threshold ############################
    // ################################################################
    cv::Mat denoised;

    int kernel_size = config_->kernel_size;
    kernel_size = (kernel_size % 2) ? kernel_size : kernel_size + 1;
    
    cv::GaussianBlur( msg->image, denoised, cv::Size(kernel_size, kernel_size), 0, 0);
    cv::threshold( denoised, denoised, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
    /* cv::adaptiveThreshold( msg->image, denoised, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,  */
    /* 			   cv::THRESH_BINARY, kernel_size,  */
    /* 			   getLatestConfig<_ShapeMatcherConfig>("image_proc").c ); */

    // ################################################################
    // Segment out contours ############################################
    // ################################################################
        
    cv::Mat contour_image;
    denoised.copyTo(contour_image);
    
    std::vector<std::vector<cv::Point2i> > contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours( contour_image, contours, hierarchy, 
		      CV_RETR_TREE, CV_CHAIN_APPROX_NONE );
    
    cv::cvtColor( contour_image, contour_image, CV_GRAY2BGR );    


    for(unsigned int idx = 0; idx < contours.size(); ++idx)
      {
	/// If the contour has a parent; it is a child
	if( hierarchy[idx][3] != -1 )
	  {
	    cv::drawContours(contour_image, contours, idx, uscauv::CV_PINK_BGR,
			     2, 8, hierarchy);
	  }
	else
	  cv::drawContours(contour_image, contours, idx, uscauv::CV_GREEN_BGR,
			   2, 8, hierarchy);
      }

    // ################################################################
    // Analyze contours and match shapes ##############################
    // ################################################################

    /// TODO: Populate this with hierarchy
    _MatchedShapeArray matches;
    /// so that time and frame data is preserved
    matches.header = msg->header;
    matches.image_rows = msg->image.rows;
    matches.image_cols = msg->image.cols;

    cv::Mat match_image;
    contour_image.copyTo(match_image);
    
    for(unsigned int idx = 0; idx < contours.size(); ++idx )
      {
	ContourData result;
	if(analyzeContour( contours[ idx ], result, config_->signature_size ))
	  continue;
	
	for(_NamedContourData::const_iterator template_it = templates_.begin();
	    template_it != templates_.end(); ++template_it )
	  {
	    /// calculate EMD using our custom cost matrix
	    double emd = cv::EMD( result.signature_, template_it->second.signature_,
				  CV_DIST_USER, emd_cost_ );
	    ROS_DEBUG("[ %s ] EMD: %f", template_it->first.c_str(), emd );
	    
	    if( emd < config_->emd_boundary )
	      {
		/// Draw 
		ROS_DEBUG("Match detected.");
		/* result.contour_ = template_it->second.contour_; */
		drawContour(match_image, result, template_it->first);

		/// Populate match message
		_MatchedShape match;
		/// switch from image coordinates to camera coordinates
		match.x = (result.mean_.x - msg->image.cols/2);
		/// negate because the image has a flipped y axis
		match.y = -(result.mean_.y - msg->image.rows/2);
		/// same deal as y
		match.theta = -result.rotation_;
		match.scale = result.radius_;
		
		match.color = "blaze_orange"; /// TODO: Refactor color classifier publishing scheme so that this isn't hard-coded
		match.type = template_it->first;

		/// Arbitrary measure of confidence. Covariance matrix is diagonal to reflect uncorrelatedness of parameters.
		/// TODO: Analyze this a further
		match.covariance = { {emd, 0, 0, 0,
				      0, emd, 0, 0,
				      0, 0, emd, 0,
				      0, 0, 0, emd} };

		matches.shapes.push_back( match );
	      }
	  }
	
	/// finish analyzing, draw
	/* cv::Point2f const & mean = result.mean_; */

	/* ROS_INFO("Got mean %f, %f", mean.x, mean.y ); */
	/* ROS_INFO("Got rotation %f.", result.rotation_ * 180 / M_PI); */
	/* ROS_INFO("Got bounding circle radius: %f", result.radius_ ); */
	/* cv::circle(match_image, mean, result.radius_, uscauv::CV_RED_BGR, 2); */
	
      }
    

    // ################################################################
    // Publish results ################################################
    // ################################################################
       
    /// sensor_msgs::image_encodings::MONO8 = "mono8", for reference
    cv_bridge::CvImage::Ptr denoised_output = boost::make_shared<cv_bridge::CvImage>
      ( msg->header, sensor_msgs::image_encodings::MONO8, denoised );
    cv_bridge::CvImage::Ptr contour_output = boost::make_shared<cv_bridge::CvImage>
      ( msg->header, sensor_msgs::image_encodings::BGR8, contour_image );
    cv_bridge::CvImage::Ptr match_output = boost::make_shared<cv_bridge::CvImage>
      ( msg->header, sensor_msgs::image_encodings::BGR8, match_image );

    publishImage(
		 "image_contours", contour_output, 
		 "image_denoised", denoised_output,
		 "image_matched", match_output 
		 );

    /// publish matched shapes
    if (matches.shapes.size() > 0 )
      match_pub_.publish( matches );

    return;
  }

  /// prefer to use config instead of config_ within this function
  void reconfigureCallback( _ShapeMatcherConfig const & config )
  {
    /// TODO: Cost type as a config argument
    circularCostEuclidian( emd_cost_, config.signature_size );
    /* ROS_INFO("Computed [ %dx%d ] circulant cost matrix.", emd_cost_.rows, emd_cost_.cols); */

    for(_NamedContourMap::const_iterator contour_it = template_contours_.begin();
	contour_it != template_contours_.end(); ++contour_it)
      {
	ROS_INFO("Generating template signature [ %s ]...", contour_it->first.c_str() );
	ContourData result;
	if(analyzeContour( contour_it->second, result,config.signature_size ))
	  ROS_WARN("Signaure generation failed.");
	else
	  {
	    templates_[ contour_it->first ] = result;
	    ROS_INFO("Signature generation success.");
	  }
      }


    return;
  }

 private:
  
  /// TODO: Fill the contour before doing mean/rotation ops
  int analyzeContour( _Contour const & input, ContourData & result, int nd )
  {
    /// TODO: Figure out exactly causes issues when data is this small
    if( input.size() <= 1)
      return -1;
        
    result = ContourData();
    _Contour2f output_contour;
    _Signature output_signature;
    float rotation;
    double max_radius;
    /// mean is a 1x2 row vector, cov is a 2x2 symmetric matrix
    cv::Mat mean(1, 2, CV_32F), cov(2,2, CV_32F), 
      eigenvec, eigenval, sort_idx, contour_sorted;
    
    cv::Mat contour; cv::Mat(input).convertTo(contour, CV_32F);
    contour = contour.reshape(1, 0); 
    /// contour is now an nx2 vector where each row is a datapoint
    
    /**
     * Absolutely need to set last arg to correct matrix type. This function 
     * uses old C api and can't deduce type internally
     */
    cv::calcCovarMatrix( contour, cov, mean, 
			 CV_COVAR_NORMAL + CV_COVAR_ROWS, CV_32F );

    if ( !cv::eigen( cov, eigenval, eigenvec ) )
      ROS_WARN("Eigendecomposition failed.");

    /// so that column eigenvectors are in rows, making them easier to access.
    cv::transpose( cov, cov );

    ROS_DEBUG("Got eigenvals: %f, %f", eigenval.at<float>(0, 0), eigenval.at<float>(1,0));
    ROS_DEBUG("Got eigenvectors: [%f, %f; %f, %f].",
	     eigenvec.at<float>(0, 0), eigenvec.at<float>(0,1),
	     eigenvec.at<float>(1, 0), eigenvec.at<float>(1,1));
    
    /// atan is on the interval [-pi/2, pi/2]
    float* ev1 = eigenvec.ptr<float>(0);
    rotation = atan(ev1[1]/ev1[0]);

    /// center contour at zero
    float* mean_ptr = mean.ptr<float>(0);
    cv::subtract(contour.col(0), cv::Scalar(mean_ptr[0]), contour.col(0));
    cv::subtract(contour.col(1), cv::Scalar(mean_ptr[1]), contour.col(1));

    /// has consistently been 0,0
    /* ROS_INFO("new mean: %f, %f", cv::mean(contour.col(0))[0], cv::mean(contour.col(1))[0] ); */

    /// convert to polar form, with angle in the left col and radius in right
    for(int idx = 0; idx < contour.rows; ++idx)
      {
	float* row = contour.ptr<float>(idx);
	float theta = cv::fastAtan2( row[1], row[0] );
	/// rotate to zero
	theta = theta - (rotation*180/M_PI); 
	/// Make sure that theta stays in the range [0, 2pi]
	theta = 
	  ((theta < 0 ) ? 360 - theta: 
	   (theta > 360 ) ? -360 + theta: 
	   theta) * M_PI / 180;
	float rad = sqrt(pow(row[0], 2) + pow(row[1], 2));
	row[0] = theta; row[1] = rad;
      }
 
    /// get the radius of the bounding circle
    cv::minMaxLoc( contour.col(1), NULL, &max_radius );
    /// normalize radius to 1
    contour.col(1) = (1.0f/max_radius) * contour.col(1);
    
    /// sort by angle, so that angle(0)=0, ..., angle(n)=360
    contour_sorted = cv::Mat( contour.rows, contour.cols, CV_32F);
    cv::sortIdx( contour.col(0), sort_idx, CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING );
    int* sort = sort_idx.ptr<int>(0);
    for(int idx = 0; idx < sort_idx.rows; ++idx)
      {
	/// funny syntax due to cv::Mat::row being sorta broken, see the documentation
	contour.row( sort[idx] ).copyTo( contour_sorted.row( idx ));
	/* ROS_INFO("%d->%d: Current theta: %f,%f", sort[idx], idx, contour.at<float>(idx,0), */
	/* 	 contour_sorted.at<float>(idx, 0)); */
      }
    
    /// Convert back to cartesian to create a contour that we will use to draw later
    for(int idx = 0; idx < contour_sorted.rows; ++idx)
      {
	float* row = contour_sorted.ptr<float>(idx);
	output_contour.push_back( cv::Point2f( row[1]*cos(row[0]), row[1]*sin(row[0])));
      }    

    /// create the final signature
    int bin = 1;
    int idx = 0;
    while(bin <= nd )
      {
	float ub = bin * 2*M_PI / nd;
	float acc = 0.0f;
	int n = 0;
	/* DEBUG_SIZE( contour_sorted, "cs"); */
	
	while( contour_sorted.at<float>(idx,0) < ub && idx < contour_sorted.rows)
	  {
	    acc += contour_sorted.at<float>(idx,1);
	    ++n;
	    ++idx;
	  }
	float output = (n)? acc/n: 0;
	output_signature.push_back(output);
	
	/* ROS_INFO("%d: Signatured: %f (idx %d) (n %d)", bin, output, idx, n); */
	
	++bin;
      }
    /// Turn signature into pdf
    cv::Scalar signature_sum = cv::sum( output_signature);
    output_signature *= 1/signature_sum[0];

    result.mean_      = cv::Point2f( mean_ptr[0], mean_ptr[1] );
    result.eigenval_  = eigenval;
    result.eigenvec_  = eigenvec;
    result.rotation_  = rotation;
    result.radius_    = max_radius;
    result.contour_   = output_contour;
    result.signature_ = output_signature;

    return 0;
  }

  /// I copied and pasted a bunch of code from the analyzeContours function because I'm lazy!
  void drawContour(cv::Mat & img, ContourData const & contour_data, std::string const & name = "")
  {
    int const pc_size = 10;
    int const thickness = 1.5;

    int const font = cv::FONT_HERSHEY_SIMPLEX;
    double const font_scale = 0.5;
    int const font_thickness = 1.5;

    cv::Point2i mean( contour_data.mean_.x, contour_data.mean_.y); 
    
    _Contour output_contour;
    std::vector<_Contour> contours;
    cv::Mat contour; cv::Mat(contour_data.contour_).convertTo(contour, CV_32F);
    contour = contour.reshape(1, 0); 

    for(int idx = 0; idx < contour.rows; ++idx)
      {
	float* row = contour.ptr<float>(idx);
	float theta = cv::fastAtan2( row[1], row[0] );
	/// rotate to zero
	theta += (contour_data.rotation_*180/M_PI); 
	/// Make sure that theta stays in the range [0, 2pi]
	theta = 
	  ((theta < 0 ) ? 360 - theta: 
	   (theta > 360 ) ? -360 + theta: 
	   theta) * M_PI / 180;
	float rad = sqrt(pow(row[0], 2) + pow(row[1], 2)) * contour_data.radius_;
	row[0] = theta; row[1] = rad;
      }
    /// TODO: Combine these loops
    for(int idx = 0; idx < contour.rows; ++idx)
      {
	float* row = contour.ptr<float>(idx);
	cv::Point2i mp = cv::Point2i( row[1]*cos(row[0]), row[1]*sin(row[0]))+mean;
	output_contour.push_back(mp);
      }    
    contours.push_back(output_contour);

    /// TODO: fix this
    /// draw the contour
    cv::drawContours(img, contours, 0, uscauv::CV_USCCARDINAL_BGR, 2);
    /// draw principal components
    const float* evec = contour_data.eigenvec_.ptr<float>(0);
    cv::Point2i e1( evec[0]*pc_size, evec[1]*pc_size), 
      e2( evec[2]*pc_size, evec[3]*pc_size);

    cv::line(img, mean - e1, mean + e1,
	     uscauv::CV_USCCARDINAL_BGR, thickness );
    cv::line(img, mean - e2, mean + e2,
	     uscauv::CV_USCCARDINAL_BGR, thickness );

    /// draw text
    if( name != "" )
      cv::putText( img, name, mean, font, font_scale, 
		   uscauv::CV_USCGOLD_BGR, font_thickness );
    
   }

  /**
   * Generate a cost matrix for the cv::EMD function. Our signatures
   * live in polar coordinates and thus have a circular distance function.
   */

  int algebraic_mod(int a, int b){ return ((a%b)+b)%b; }
  
  void circularCostEuclidian( cv::Mat & cost, int nd )
  {
    cost.create( nd, nd, CV_32FC1 );

    for(int idy = 0; idy < nd; ++idy )
      {
	float * cost_row = cost.ptr<float>( idy );
	for(int idx = 0; idx < nd; ++idx )
	  {
	    cost_row[ idx ] = std::min( algebraic_mod(idx - idy, nd), 
					algebraic_mod(idy - idx, nd) );
	  }
      }
  }

  void circularCostExp( cv::Mat & cost, int nd, float c = 1 )
  {
    circularCostEuclidian( cost, nd );
    
    float* cost_ptr = cost.ptr<float>(0);
    for(int idx= 0; idx < nd*nd;  ++idx )
      {
	cost_ptr[idx] = exp( -cost_ptr[idx]*c);
      }
  }
  

};

#endif // USCAUV_SHAPEMATCHING_SHAPEMATCHER
