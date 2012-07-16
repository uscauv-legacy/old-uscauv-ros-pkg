/***************************************************************************
 *  include/image_server/image_server_node.h
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

#ifndef IMAGESERVER_IMAGESERVERNODE_H_
#define IMAGESERVER_IMAGESERVERNODE_H_

#include <quickdev/node.h>

// objects
#include <quickdev/image_loader.h>
#include <camera_info_manager/camera_info_manager.h>

// policies
#include <quickdev/image_proc_policy.h>
#include <quickdev/reconfigure_policy.h>

// msgs
#include <sensor_msgs/CameraInfo.h>

// cfgs
#include <image_server/ImageServerConfig.h>

typedef image_server::ImageServerConfig _ImageServerConfig;
typedef sensor_msgs::CameraInfo _CameraInfoMsg;

typedef quickdev::ImageProcPolicy _ImageProcPolicy;
typedef quickdev::ReconfigurePolicy< _ImageServerConfig > _ImageServerConfigPolicy;

typedef camera_info_manager::CameraInfoManager _CameraInfoManager;

QUICKDEV_DECLARE_NODE( ImageServer, _ImageProcPolicy, _ImageServerConfigPolicy )

QUICKDEV_DECLARE_NODE_CLASS( ImageServer )
{
private:
    ImageLoader image_loader_;
    _CameraInfoManager camera_info_manager_;
    ros::MultiPublisher<> multi_pub_;

public:
    bool last_next_image_state_;
    bool last_prev_image_state_;
    unsigned int current_frame_;
    unsigned int direction_;

    _CameraInfoMsg::ConstPtr camera_info_msg_ptr_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( ImageServer ),
        image_loader_( quickdev::getFirstOfType<ros::NodeHandle>( args... ) ),
        camera_info_manager_( quickdev::getFirstOfType<ros::NodeHandle>( args... ) ),
        last_next_image_state_( false ),
        last_prev_image_state_( false ),
        current_frame_( 0 ),
        direction_( 0 )
    {

    }

    void spinFirst()
    {
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        multi_pub_.addPublishers<_CameraInfoMsg>( nh_rel, { "camera_info" } );

        std::string node_name = nh_rel.getNamespace();
        if( node_name.substr( 0, 1 ) == "/" ) node_name = node_name.substr( 1 );
        camera_info_manager_.setCameraName( node_name );

        auto const camera_info_url = quickdev::ParamReader::readParam<std::string>( nh_rel, "camera_info_url" );

        if( camera_info_manager_.validateURL( camera_info_url ) ) camera_info_manager_.loadCameraInfo( camera_info_url );
        else PRINT_WARN( "Camera info URL %s not valid.", camera_info_url.c_str() );

        _CameraInfoMsg camera_info_msg = camera_info_manager_.getCameraInfo();
        camera_info_msg.header.frame_id = node_name;

        camera_info_msg_ptr_ = quickdev::make_const_shared( camera_info_msg );

        _ImageServerConfigPolicy::registerCallback( quickdev::auto_bind( &ImageServerNode::reconfigureCB, this ) );

        initPolicies<_ImageProcPolicy>
        (
            "subscribe_to_image_param", false
        );

        initPolicies<quickdev::policy::ALL>();

        image_loader_.loadImages();
    }

    void spinOnce()
    {
        if ( image_loader_.images_loaded_ )
        {
            if ( current_frame_ < image_loader_.image_cache_.size() )
            {
                ROS_INFO( "Publishing image %d [%dx%d]", current_frame_, image_loader_.image_cache_[current_frame_]->width, image_loader_.image_cache_[current_frame_]->height );

                publishImages( "output_image", quickdev::opencv_conversion::fromIplImage( image_loader_.image_cache_[current_frame_], "image_server", "bgr8" ) );

                multi_pub_.publish( "camera_info", camera_info_msg_ptr_ );

                //publishCvImage( image_loader_.image_cache_[current_frame_] );
                if ( config_.auto_advance )
                {
                    switch ( direction_ )
                    {
                    case 0:
                        nextFrame();
                        break;
                    case 1:
                        prevFrame();
                        break;
                    }
                }
            }
        }
    }

    void nextFrame()
    {
        if ( current_frame_ < image_loader_.image_cache_.size() - 1 )
        {
            ++current_frame_;
        }
        else if ( config_.loop )
        {
            current_frame_ = 0;
        }
    }

    void prevFrame()
    {
        if ( current_frame_ > 0 && image_loader_.image_cache_.size() > 0 )
        {
            --current_frame_;
        }
        else if ( config_.loop )
        {
            current_frame_ = image_loader_.image_cache_.size() - 1;
        }
    }

    QUICKDEV_DECLARE_RECONFIGURE_CALLBACK( reconfigureCB, _ImageServerConfig )
    {
        if ( config.auto_advance )
        {
            if ( config.next_image && !config.prev_image ) direction_ = 0;
            else if ( !config.next_image && config.prev_image ) direction_ = 1;
            else direction_ = 0;

            return;
        }

        if ( last_next_image_state_ != config.next_image )
        {
            nextFrame();
        }
        else if ( last_prev_image_state_ != config.prev_image )
        {
            prevFrame();
        }

        last_next_image_state_ = config.next_image;
        last_prev_image_state_ = config.prev_image;
    }
};

#endif /* IMAGESERVER_IMAGESERVERNODE_H_ */
