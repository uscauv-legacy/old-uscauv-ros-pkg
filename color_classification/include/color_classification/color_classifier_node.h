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
/* #include <color_classification/svm_color_classifier.h> */

/// cpp11
#include <thread>
#include <functional>

/// uscauv
#include <uscauv_common/color_codec.h>
#include <uscauv_common/param_loader.h>
#include <uscauv_common/tic_toc.h>

typedef std::map<std::string, image_transport::Publisher> _ColorPublisherMap;

struct ClassifyThreadStorage
{
  enum class State{ READY, PROCESSED };
  State state_;

  std::condition_variable cv_;
  std::mutex m_;

  cv::Mat input_, output_;

  /// cv::SVM doesn't have proper copy assignment so we do this to avoid copying
  std::shared_ptr<cv::SVM> svm_;
  
  ClassifyThreadStorage(){ state_ = State::PROCESSED; }
};

typedef std::map<std::string, std::shared_ptr<ClassifyThreadStorage> > _ColorThreadMap;

class ColorClassifierNode
{
 private:
  /// publishers and subscribers
  ros::NodeHandle nh_rel_;
  image_transport::ImageTransport image_transport_;
  image_transport::Subscriber image_sub_;
  _ColorPublisherMap classified_image_pub_;
  _ColorThreadMap thread_storage_;
  uscauv::EncodedColorPublisher encoded_image_pub_;  

  /// parameters
  double loop_rate_hz_;
  
  /// color classification
  std::vector<std::string> color_names_;
  
 public:

  /** 
   * Default Constructor
   * 
   */
  ColorClassifierNode()
    :
    nh_rel_("~"),
    image_transport_( nh_rel_ )
    {}
    
 private:
    
    /// Spawn one thread that runs this function per color definition
    void classifyThread(std::shared_ptr<ClassifyThreadStorage> storage)
    {
      while(true)
	{
      
	  /// Wait for the main thread to signal that the image is ready for processing
	  {
	    std::unique_lock<std::mutex> lock( storage->m_ );
	    storage->cv_.wait( lock, [&]{ return storage->state_ == ClassifyThreadStorage::State::READY; });
	  }
	  
	  unsigned int match_count = 0;

	  cv::Mat input_image = storage->input_;

	  /// convert to HSV
	  cv::cvtColor(input_image, input_image, CV_BGR2HSV);
    
	  cv::Mat input_hs( input_image.rows, input_image.cols, CV_8UC2 ), input_v( input_image.rows, input_image.cols, CV_8UC1 );
	  cv::Mat mix_out[] { input_hs, input_v };
	  int from_to[] = { 0,0, 1,1, 2,2 };
	  cv::mixChannels( &input_image, 1, mix_out, 2, from_to, 3);
    
	  cv::Mat input_float;
	  input_hs.convertTo( input_float, CV_32F );
    
	  cv::Mat classified_image = cv::Mat( input_image.size(), CV_8UC1 );
    
	  /// Classify the input image.
	  cv::MatIterator_<unsigned char> cl_it = classified_image.begin<unsigned char>();
	  cv::MatConstIterator_<cv::Vec2f> in_it = input_float.begin<cv::Vec2f>();
  
	  for(; in_it != input_float.end<cv::Vec2f>(); ++in_it, ++cl_it)
	    {
	      float response = storage->svm_->predict( cv::Mat(*in_it) );
	      
	      if ( response == -1.0)
	  	*cl_it = 0;
	      else if ( response == 1.0 )
	  	{
	  	  *cl_it = 255;
	  	  ++match_count;
	  	}
	      else
	  	{
	  	  ROS_WARN( "SVM has incorrect output format. Image will not be classifed. Valid output: {-1, 1}");
	  	  /* return -1; */
	  	}
	    }

	  storage->output_ = classified_image;
	  
	  /* ROS_DEBUG("[ %s ] SVM matched %d pixels. ", color_name.c_str(),match_count); */
	  
	  /// Notify the main thread that processing is complete
	  {
	    std::lock_guard<std::mutex> lock( storage->m_ );
	    storage->state_ = ClassifyThreadStorage::State::PROCESSED;
	  }
	  storage->cv_.notify_one();

	}
      return;
    }
  
  /// Running spin() will cause this function to be called before the node begins looping the spingOnce() function.
  void spinFirst()
  {
    /// Get ROS ready ------------------------------------
    ros::NodeHandle nh;
    image_transport_ = image_transport::ImageTransport( nh_rel_ );
    
    /// Load SVMs ------------------------------------
    XmlRpc::XmlRpcValue xml_colors = uscauv::loadParam<XmlRpc::XmlRpcValue>( nh, "model/colors" );

    unsigned int color_count = 0;
    for(std::map<std::string, XmlRpc::XmlRpcValue>::iterator color_it = xml_colors.begin(); color_it != xml_colors.end(); ++color_it)
      {
	/// TODO: Catch XMLRPC errors

	std::shared_ptr<ClassifyThreadStorage> storage = std::make_shared<ClassifyThreadStorage>();
	
	/// name of the color
	std::string const & color_name = color_it->first;

	/// Path to yaml file containing svm params
	std::string const & color_path = color_it->second;
	
	/// File I/O datatypes
	std::shared_ptr<cv::SVM> color_svm = std::make_shared<cv::SVM>();
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
	
	/// populate the fields the the cv::SVM
	color_svm->read( svm_storage, svm_node );
	
	/// Add to the map
	/* color_svm_map_[ color_name ] = color_svm; */
	/// TODO: Add to thread map
	cvReleaseFileStorage( &svm_storage );

	storage->svm_ = color_svm;
	thread_storage_[ color_name ] = storage;
	std::thread classify_thread( &ColorClassifierNode::classifyThread, this, 
				     thread_storage_[ color_name ] );
	classify_thread.detach();
	
	++color_count;
	ROS_INFO( "Loaded SVM successfully. [ %s ]", color_name.c_str() );

    	ROS_INFO( "Creating publisher... [ %s ]", color_name.c_str() );
	
    	image_transport::Publisher color_pub;

    	/// We will publish each color classified image to a topic called <color_name>_classified.
    	color_pub = image_transport_.advertise( color_name + "_classified", 1);
	
    	classified_image_pub_[ color_name ] = color_pub;
	
    	ROS_INFO( "Created publisher successfully." );


      }

    if( !color_count )
      {
	ROS_FATAL( "No SVMs were loaded." );
	ros::shutdown();
	return;
      }

    encoded_image_pub_.advertise( nh_rel_, "encoded", 1 );
	  
    /// Subscribe to input image topic ------------------------------------
    image_sub_ = image_transport_.subscribe( "image_color", 1, &ColorClassifierNode::imageCallback, this);

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
    uscauv::ColorEncoder encoder;

    try
      {
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
    catch (cv_bridge::Exception& e)
      {
	ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
      }

    /* tic; */
    
    for( _ColorThreadMap::iterator thread_it = thread_storage_.begin(); thread_it != thread_storage_.end(); ++thread_it )
      {

	std::shared_ptr<ClassifyThreadStorage> storage = thread_it->second;
	
	/// Copy image into buffer, signal thread to process
	{
	  std::lock_guard<std::mutex> lock( storage->m_ );
	  cv_ptr->image.copyTo( storage->input_ );
	  storage->state_ = ClassifyThreadStorage::State::READY;
	}
	storage->cv_.notify_one();
      }

    for( _ColorThreadMap::iterator thread_it = thread_storage_.begin(); thread_it != thread_storage_.end(); ++thread_it )
      {	

	std::shared_ptr<ClassifyThreadStorage> storage = thread_it->second;

	cv_bridge::CvImage classified_image( cv_ptr->header,
    					     sensor_msgs::image_encodings::MONO8 );

	/// Wait for thread to finish processing the image
	{
	  std::unique_lock<std::mutex> lock( storage->m_ );
	  storage->cv_.wait( lock, [&]{return storage->state_ == ClassifyThreadStorage::State::PROCESSED;} );
	  encoder.addImage( storage->output_, thread_it->first );
	  classified_image.image = storage->output_;
	}
	
	classified_image_pub_[ thread_it->first ].publish ( classified_image.toImageMsg() );
      }



    /* toc_info_stream( std::chrono::milliseconds, "Classify all"); */
    
    encoded_image_pub_.publish( encoder, msg->header );
    return;
  }

};

#endif // USCAUV_COLORCLASSIFICATION_COLORCLASSIFIERNODE_H
