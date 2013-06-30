/***************************************************************************
 *  include/uscauv_common/color_codec.h
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


#ifndef USCAUV_USCAUVCOMMON_COLORCODEC
#define USCAUV_USCAUVCOMMON_COLORCODEC

// ROS
#include <ros/ros.h>

// images
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <auv_msgs/ColorEncodedImage.h>

namespace uscauv
{
  static char const * const COLOR_CODEC_IMAGE_TYPE = "mono16";

  typedef std::map<std::string, cv::Mat> ColorImageMap;
  typedef std::shared_ptr<ColorImageMap> ColorImageMapPtr;
  typedef std::shared_ptr<ColorImageMap const> ColorImageMapConstPtr;
  
  class ColorEncoder
  {
  private:
    cv::Mat image_;
    std::vector<std::string> names_;
    unsigned int color_idx_;
    
  public:
  ColorEncoder(): color_idx_(0){}

    void addImage( cv::Mat const & input, std::string const & name)
    {
      if( image_.empty())
	{
	  image_ = cv::Mat( input.size(), cv_bridge::getCvType( COLOR_CODEC_IMAGE_TYPE ));
	  image_.setTo(0);
	}
      cv::Mat encoded; input.copyTo(encoded); 
      /// convert to mono16 before encoding so that mono8 images don't clip when image count exceeds 8
      encoded.convertTo(encoded, CV_8UC1);
      /// Set the pixels where encoded is non-zero to 2^color_idx
      cv::add(image_, (1 << color_idx_), image_, encoded);
      names_.push_back(name);
      ++color_idx_;
    }

    friend class EncodedColorPublisher;
  };
  
  class EncodedColorPublisher
  {
  private:
    ros::Publisher pub_;

  public:
    void advertise( ros::NodeHandle nh, std::string const & topic, int const & queue_size = 1)
    {
      pub_ = nh.advertise<auv_msgs::ColorEncodedImage>(topic, queue_size );
    }
    
    void publish( ColorEncoder const & encoder,  std_msgs::Header const & header)
    {
      auv_msgs::ColorEncodedImage msg;
      cv_bridge::CvImage image_out(header, COLOR_CODEC_IMAGE_TYPE, encoder.image_ );
      image_out.toImageMsg(msg.image);
      msg.encoding = encoder.names_;
      pub_.publish( msg );

    }
    
  };

  class EncodedColorSubscriber
  {
  private:
    
    ros::Subscriber sub_;
    std::function< void( ColorImageMapPtr const &, std_msgs::Header const &)> external_callback;

  public:
    template<class... __BoundArgs>
      void subscribe( ros::NodeHandle nh, std::string const & topic, int const & queue_size, __BoundArgs&&... bound_args)
      {
	sub_ = nh.subscribe<auv_msgs::ColorEncodedImage>(topic, queue_size,
							 &EncodedColorSubscriber::decode, this);
	external_callback = std::bind( std::forward<__BoundArgs>(bound_args)...,
				       std::placeholders::_1, std::placeholders::_2);
      }
    
  private:
    void decode( auv_msgs::ColorEncodedImage::ConstPtr const & msg)
    {
      ColorImageMapPtr decoded = std::make_shared<ColorImageMap>();
      
      unsigned int color_idx = 0;
      for(std::vector<std::string>::const_iterator name_it = msg->encoding.begin();
	  name_it != msg->encoding.end(); ++name_it, ++color_idx)
	{
	  cv::Mat output = cv_bridge::toCvCopy( msg->image, COLOR_CODEC_IMAGE_TYPE )->image;
	  cv::bitwise_and(output, (1 << color_idx) , output);
	  /* ROS_DEBUG("Decoding with key %d", (1 << color_idx)); */
	  output.convertTo( output, CV_8UC1 );
	  output.setTo( 255, output );
	  decoded->insert( std::pair<std::string, cv::Mat>( *name_it, output ));
	}
      
      if( external_callback )
	external_callback( decoded, msg->image.header );
    }
    
  };

    
} // uscauv

#endif // USCAUV_USCAUVCOMMON_COLORCODEC
