/***************************************************************************
 *  include/color_classification/svm_color_classifier.h
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

#ifndef USCAUV_COLORCLASSIFICATION_SVMCOLORCLASSIFIER_H
#define USCAUV_COLORCLASSIFICATION_SVMCOLORCLASSIFIER_H

/// ROS
#include <ros/ros.h>

/// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv/cxcore.h>

/// xmlrpcpp
#include <XmlRpcValue.h>

/** #################################### **/
/**
 * This class uses Support Vector Machines to classify colors in images.
 * It supports loading of multiple named cv::SVMs from yaml.
 * Once the SVM is loaded, the classify() function can be called with the 
 * name of the SVM to classify an input image.
 */
/** #################################### **/
class SvmColorClassifier
{
 private:
  typedef std::map< std::string, cv::SVM* > _ColorSvmMap;

  _ColorSvmMap color_svm_map_;

  const unsigned char false_color_, true_color_;
  
 public:

  /** 
   * Default Constructor
   * 
   */
 SvmColorClassifier()
   :
  false_color_( 0 ),
  true_color_( 255 )
    {}

  /** 
   * Destructor 
   *
   */
  ~SvmColorClassifier()
    {
      for( _ColorSvmMap::iterator svm_it = color_svm_map_.begin(); svm_it != color_svm_map_.end(); ++svm_it)
	{
	  if( svm_it->second != NULL )
	    delete svm_it->second;
	  
	}
    }
  
  int fromXmlRpc(XmlRpc::XmlRpcValue & xml_colors)
  {
    unsigned int color_count = 0;
    
    for(std::map<std::string, XmlRpc::XmlRpcValue>::iterator color_it = xml_colors.begin(); color_it != xml_colors.end(); ++color_it)
      {
	/// TODO: Catch XMLRPC errors
	
	/// name of the color
	std::string const & color_name = color_it->first;

	/// Path to yaml file containing svm params
	std::string const & color_path = color_it->second;
	
	/// File I/O datatypes
	cv::SVM * color_svm =         NULL;
	CvFileStorage * svm_storage = NULL;
	CvFileNode * svm_node =       NULL;
	
	/// Open file storage
	svm_storage = cvOpenFileStorage(color_path.c_str(), NULL, CV_STORAGE_READ);
	if (svm_storage == NULL)
	  {
	    ROS_WARN( "Failed to open SVM. [ %s ] [ %s ]", color_name.c_str(), color_path.c_str() );
	    continue;
	  }

	/// Search for a yaml node with the name of the color from the highest level.
	svm_node = cvGetFileNodeByName( svm_storage, NULL, color_name.c_str() );
	if (svm_node == NULL)
	  {
	    ROS_WARN( "Failed to find SVM file node. [ %s ]", color_name.c_str() );
	    continue;
	  }
	
	color_svm = new cv::SVM;

	/// populate the fields the the cv::SVM
	color_svm->read( svm_storage, svm_node );
	
	/// Add to the map
	color_svm_map_[ color_name ] = color_svm;
	
	cvReleaseFileStorage( &svm_storage );
	
	++color_count;
	
	ROS_INFO( "Loaded SVM successfully. [ %s ]", color_name.c_str() );
      }

    if( !color_count )
      ROS_WARN( "No SVMs were loaded." );
    
    return 0;
  }
  
  int classify(std::string const & color_name, cv::Mat const & input_image,
		cv::Mat & classified_image)
  {
    unsigned int match_count = 0;
    
    /// Look up the requested color
    _ColorSvmMap::iterator svm_it = color_svm_map_.find( color_name );

    /// Check for existence of color
    if( svm_it == color_svm_map_.end() )
      {
	ROS_WARN( "Received a color classification request for color [ %s ], but the SVM is not loaded.", color_name.c_str() );
	return -1;
      }

    /// convert to HSV
    cv::cvtColor(input_image, input_image, CV_BGR2HSV);
    
    cv::Mat input_hs( input_image.rows, input_image.cols, CV_8UC2 ), input_v( input_image.rows, input_image.cols, CV_8UC1 );
    cv::Mat mix_out[] { input_hs, input_v };
    int from_to[] = { 0,0, 1,1, 2,2 };
    cv::mixChannels( &input_image, 1, mix_out, 2, from_to, 3);
    
    
    cv::Mat input_float;
    input_hs.convertTo( input_float, CV_32F );
    
    classified_image = cv::Mat( input_image.size(), CV_8UC1 );
    
    /// Classify the input image.
    cv::MatIterator_<unsigned char> cl_it = classified_image.begin<unsigned char>();
    cv::MatConstIterator_<cv::Vec2f> in_it = input_float.begin<cv::Vec2f>();

    for(; in_it != input_float.end<cv::Vec2f>(); ++in_it, ++cl_it)
      {
	float response = svm_it->second->predict( cv::Mat(*in_it) );

	if ( response == -1.0)
	  *cl_it = false_color_;
	else if ( response == 1.0 )
	  {
	    *cl_it = true_color_;
	    ++match_count;
	  }
	else
	  {
	    ROS_WARN( "[ %s ] SVM has incorrect output format. Image will not be classifed. Valid output: {-1, 1}", color_name.c_str() );
	    return -1;
	  }
      }

    ROS_DEBUG("[ %s ] SVM matched %d pixels. ", color_name.c_str(),match_count);
    
    /* cv::namedWindow("Classify Test", CV_WINDOW_AUTOSIZE); */
    /* cv::imshow("Classify Test", classified_image); */
    /* cv::waitKey(0); */
        
    return 0;
  }
  
  /** 
   * Get all of the names of colors for which SVMs are loaded.
   * 
   * @return Vector containing color name strings.
   */
  std::vector<std::string> getColorNames()
    {
      std::vector<std::string> colors;

      /// Retrieve all of the color keys from the color -> svm map
      for(_ColorSvmMap::iterator color_it = color_svm_map_.begin(); color_it != color_svm_map_.end(); ++color_it)
	  colors.push_back( color_it->first );
      
      return colors;
    }
  

  /// TODO: Add function to get support vector data for a given color

  /// TODO: Add function to classify all colors so that input image doesn't need to be converted to a float for every color
};

#endif // USCAUV_COLORCLASSIFICATION_SVMCOLORCLASSIFIER_H
