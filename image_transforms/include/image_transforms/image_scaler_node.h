/***************************************************************************
 *  include/image_transforms/image_scaler_node.h
 *  --------------------
 *
 *  Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
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
 *  * Neither the name of seabee3-ros-pkg nor the names of its
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

#ifndef IMAGETRANSFORMS_IMAGESCALERNODE_H_
#define IMAGETRANSFORMS_IMAGESCALERNODE_H_

#include <quickdev/node.h>

// objects
#include <quickdev/multi_publisher.h>
#include <quickdev/multi_subscriber.h>
#include <quickdev/param_reader.h>
#include <camera_info_manager/camera_info_manager.h>

// policy
#include <quickdev/image_proc_policy.h>

// msgs
#include <sensor_msgs/CameraInfo.h>

typedef sensor_msgs::CameraInfo _CameraInfoMsg;

typedef quickdev::ImageProcPolicy _ImageProcPolicy;

typedef camera_info_manager::CameraInfoManager _CameraInfoManager;

QUICKDEV_DECLARE_NODE( ImageScaler, _ImageProcPolicy )

QUICKDEV_DECLARE_NODE_CLASS( ImageScaler )
{
protected:
    ros::MultiPublisher<> multi_pub_;
    ros::MultiSubscriber<> multi_sub_;

    _CameraInfoManager camera_info_manager_;

    _CameraInfoMsg::ConstPtr static_camera_info_;

    _CameraInfoMsg::ConstPtr last_camera_info_;

    bool debayer_;
    bool scale_camera_info_;
    bool publish_static_camera_info_;
    double scale_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( ImageScaler ),
        camera_info_manager_( quickdev::getFirstOfType<ros::NodeHandle>( args... ) )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        initPolicies<_ImageProcPolicy, quickdev::RunablePolicy>( 
				       "image_callback_param", quickdev::auto_bind( &ImageScalerNode::imageCB, this ), 
				       "publish_image_param", true,
				       "image_topic_param", std::string("image"),
				       "image_cache_size_param", 1,
				       "output_image_topic_param", std::string("output_image"),
				       "spin_ros_thread_param", false
					);

        debayer_ = quickdev::ParamReader::readParam<decltype( debayer_ )>( nh_rel, "debayer", true );
        scale_camera_info_ = quickdev::ParamReader::readParam<decltype( scale_camera_info_ )>( nh_rel, "scale_camera_info", true );
        publish_static_camera_info_ = quickdev::ParamReader::readParam<decltype( publish_static_camera_info_ )>( nh_rel, "publish_static_camera_info", false );
        scale_ = quickdev::ParamReader::readParam<decltype( scale_ )>( nh_rel, "scale", 1.0 );

        auto const camera_name = quickdev::ParamReader::readParam<std::string>( nh_rel, "camera_name", "camera" );

        if( publish_static_camera_info_ )
        {
            camera_info_manager_.setCameraName( camera_name );
            auto const camera_info_url = quickdev::ParamReader::readParam<std::string>( nh_rel, "camera_info_url" );

            if( camera_info_manager_.validateURL( camera_info_url ) ) camera_info_manager_.loadCameraInfo( camera_info_url );
            else PRINT_WARN( "Camera info URL %s not valid.", camera_info_url.c_str() );

            _CameraInfoMsg camera_info_msg = camera_info_manager_.getCameraInfo();
            camera_info_msg.header.frame_id = camera_name;

            static_camera_info_ = quickdev::make_const_shared( camera_info_msg );
        }

        multi_sub_.addSubscriber( nh_rel, "camera_info_in", &ImageScalerNode::cameraInfoCB, this );
        multi_pub_.addPublishers<_CameraInfoMsg>( nh_rel, { "camera_info_out" } );

        initPolicies<quickdev::policy::ALL>();
    }

    QUICKDEV_DECLARE_IMAGE_CALLBACK( imageCB )
    {
      cv::Mat const & image = image_msg->image;

      cv::Mat output_image;
      cv::Mat scaled_image;

      if( debayer_ ) cv::cvtColor( image, output_image, CV_BayerBG2BGR );
      else image.copyTo( output_image );
      cv::resize( output_image, scaled_image, cv::Size(), scale_, scale_ );

      auto output_image_msg = quickdev::opencv_conversion::fromMat( scaled_image, "", "bgr8" );
      output_image_msg->header = image_msg->header;

      if( last_camera_info_ )
        {
	  _CameraInfoMsg output_camera_info = *last_camera_info_;
	  if( output_camera_info.header.frame_id != image_msg->header.frame_id )
	    {
	      ROS_WARN("Camera info frame id does not match image frame id. Switching info frame...");
	      output_camera_info.header = image_msg->header;
	    }
	  if( scale_camera_info_ )
	    {
	      /// Both 0 and 1 mean no binning, so we account for this when we apply the scaling transformation
	      output_camera_info.binning_x = (output_camera_info.binning_x) ? output_camera_info.binning_x/scale_ : 1.0/scale_;
	      output_camera_info.binning_y = (output_camera_info.binning_y) ? output_camera_info.binning_y/scale_ : 1.0/scale_;
            }
	  multi_pub_.publish( "camera_info_out", output_camera_info );
        }

      if( publish_static_camera_info_ )
        {
	  _CameraInfoMsg camera_info = *static_camera_info_;
	  camera_info.header = image_msg->header;

	  multi_pub_.publish( "camera_info_out", camera_info );
        }

      _ImageProcPolicy::publishImages( "output_image", output_image_msg );
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( cameraInfoCB, _CameraInfoMsg )
      {
        last_camera_info_ = msg;
      }

    QUICKDEV_SPIN_ONCE()
      {
        //
      }
};

#endif // IMAGETRANSFORMS_IMAGESCALERNODE_H_
