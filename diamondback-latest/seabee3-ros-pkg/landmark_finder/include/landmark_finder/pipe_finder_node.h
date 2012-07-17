/***************************************************************************
 *  include/landmark_finder/pipe_finder_node.h
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

#ifndef LANDMARKFINDER_PIPEFINDERNODE_H_
#define LANDMARKFINDER_PIPEFINDERNODE_H_

#include <quickdev/node.h>

// policies
#include <quickdev/reconfigure_policy.h>
#include <quickdev/tf_tranceiver_policy.h>

// objects
#include <condition_variable>
#include <quickdev/multi_subscriber.h>
#include <quickdev/multi_publisher.h>
#include <image_geometry/pinhole_camera_model.h>

// utils
#include <contour_matcher/contour.h>
#include <seabee3_common/recognition_primitives.h>

// msgs
#include <seabee3_msgs/ContourArray.h>
#include <sensor_msgs/CameraInfo.h>

// config
#include <landmark_finder/PipeFinderConfig.h>

typedef seabee3_msgs::ContourArray _ContourArrayMsg;
typedef sensor_msgs::CameraInfo _CameraInfoMsg;

typedef landmark_finder::PipeFinderConfig _PipeFinderCfg;

typedef quickdev::ReconfigurePolicy<_PipeFinderCfg> _PipeFinderReconfigurePolicy;
typedef quickdev::TfTranceiverPolicy _TfTranceiverPolicy;

// gives us various typedefs including _PinholeCameraModel
using namespace seabee;

QUICKDEV_DECLARE_NODE( PipeFinder, _PipeFinderReconfigurePolicy, _TfTranceiverPolicy )

QUICKDEV_DECLARE_NODE_CLASS( PipeFinder )
{
protected:
    ros::MultiSubscriber<> multi_sub_;
    ros::MultiPublisher<> multi_pub_;

    _PinholeCameraModel camera_model_;

    std::condition_variable process_contours_condition_;
    std::mutex process_contours_mutex_;

    _CameraInfoMsg::ConstPtr camera_info_msg_ptr_;
    std::mutex camera_info_mutex_;

    _ContourArrayMsg::ConstPtr contours_msg_ptr_;
    std::mutex contours_mutex_;

    boost::shared_ptr<boost::thread> process_contours_thread_ptr_;

    std::multiset<Pipe> pipes_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( PipeFinder )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        multi_sub_.addSubscriber( nh_rel, "contours", &PipeFinderNode::contoursCB, this );
        multi_sub_.addSubscriber( nh_rel, "camera_info", &PipeFinderNode::cameraInfoCB, this );
        multi_pub_.addPublishers<_LandmarkArrayMsg, _MarkerArrayMsg>( nh_rel, { "landmarks", "/visualization_marker_array" } );

        initPolicies<quickdev::policy::ALL>();

        process_contours_thread_ptr_ = boost::make_shared<boost::thread>( &PipeFinderNode::processContours, this );
    }

    void processContours()
    {
        while( QUICKDEV_GET_RUNABLE_POLICY()::running() )
        {
            auto process_contours_lock = quickdev::make_unique_lock( process_contours_mutex_ );
            process_contours_condition_.wait( process_contours_lock );

            if( !camera_info_msg_ptr_ )
            {
                PRINT_WARN( "Camera model not initialized." );
                continue;
            }

            // get a copy of the contours so we block other processes for the minimum amount of time
            _ContourArrayMsg contours_msg;
            {
                auto contours_lock = quickdev::make_unique_lock( contours_mutex_ );
                contours_msg = *contours_msg_ptr_;
            }

            auto const contours = contours_msg.contours;

            // fit dem ellipses

            PRINT_INFO( "Evaluating %zu contours", contours.size() );

            pipes_.clear();

            for( auto contour_it = contours.cbegin(); contour_it != contours.cend(); ++contour_it )
            {
                auto const & contour_msg = *contour_it;

                if( contour_msg.points.size() < 6 ) continue;

                _Contour contour = unit::make_unit( contour_msg );

                cv::RotatedRect rect = cv::minAreaRect( cv::Mat( contour ) );

                // we consider the larger dimension of the object to be its diameter
                double const max_diameter = std::max( rect.size.width, rect.size.height );
                double const min_diameter = std::min( rect.size.width, rect.size.height );
                double const aspect_ratio = max_diameter / min_diameter;

                // if( ( object is not too small ) and ( object has appropriate aspect ratio ) )
                if( max_diameter > config_.diameter_min && fabs( config_.aspect_ratio_mean - aspect_ratio ) < config_.aspect_ratio_variance )
                {
                    Pipe pipe( Pose( Position( rect.center.x, rect.center.y ), Orientation( -rect.angle ) ), Size( rect.size.width, rect.size.height ) );
                    pipe.projectTo3d( camera_model_ );
                    _TfTranceiverPolicy::publishTransform( btTransform( btQuaternion( 0, -M_PI_2, 0 ) * btQuaternion( pipe.pose_.orientation_.yaw_, 0, 0 ), unit::convert<btVector3>( pipe.pose_.position_ ) ) , "/seabee/camera2", pipe.getUniqueName() );
                    pipes_.insert( pipe );
                }
                else
                {
                    PRINT_WARN( "Aspect ratio (%f) or diameter (%f) outside of constraints", aspect_ratio, max_diameter );
                }
            }

            PRINT_INFO( "Found %zu pipes", pipes_.size() );

            _LandmarkArrayMsg pipes_msg;
            _MarkerArrayMsg markers_msg;

            size_t marker_id = 0;
            for( auto pipe_it = pipes_.cbegin(); pipe_it != pipes_.cend(); ++pipe_it )
            {
                auto const & pipe = *pipe_it;

                pipes_msg.landmarks.push_back( pipe );
                _MarkerMsg marker_msg = pipe;
                marker_msg.id = marker_id ++;

                markers_msg.markers.push_back( marker_msg );
            }

            multi_pub_.publish( "landmarks", pipes_msg );

            if( !pipes_msg.landmarks.empty() ) multi_pub_.publish( "/visualization_marker_array", markers_msg );
        }
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( contoursCB, _ContourArrayMsg )
    {
        auto lock = quickdev::make_unique_lock( contours_mutex_, std::try_to_lock );

        if( !lock ) return;

        contours_msg_ptr_ = msg;

        // wake up processContours() for one cycle
        process_contours_condition_.notify_all();
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( cameraInfoCB, _CameraInfoMsg )
    {
        auto lock = quickdev::make_unique_lock( camera_info_mutex_, std::try_to_lock );

        if( !lock ) return;

        camera_info_msg_ptr_ = msg;

        camera_model_.fromCameraInfo( msg );
    }

    QUICKDEV_SPIN_ONCE()
    {
        // just update ROS
    }
};

#endif // LANDMARKFINDER_PIPEFINDERNODE_H_
