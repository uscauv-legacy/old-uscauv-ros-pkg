/***************************************************************************
 *  include/landmark_finder/buoy_finder_node.h
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

#ifndef LANDMARKFINDER_BUOYFINDERNODE_H_
#define LANDMARKFINDER_BUOYFINDERNODE_H_

#include <quickdev/node.h>

// policies
#include <quickdev/reconfigure_policy.h>

// objects
#include <quickdev/multi_subscriber.h>
#include <quickdev/multi_publisher.h>

// utils
#include <contour_matcher/contour.h>
#include <seabee3_common/recognition_primitives.h>

// msgs
#include <seabee3_msgs/ContourArray.h>

// config
#include <landmark_finder/BuoyFinderConfig.h>

typedef seabee3_msgs::ContourArray _ContourArrayMsg;

typedef landmark_finder::BuoyFinderConfig _BuoyFinderCfg;

typedef quickdev::ReconfigurePolicy<_BuoyFinderCfg> _BuoyFinderReconfigurePolicy;

using namespace seabee;

QUICKDEV_DECLARE_NODE( BuoyFinder, _BuoyFinderReconfigurePolicy )

QUICKDEV_DECLARE_NODE_CLASS( BuoyFinder )
{
protected:
    ros::MultiSubscriber<> multi_sub_;
    ros::MultiPublisher<> multi_pub_;

    std::mutex process_contours_mutex_;

    _ContourArrayMsg::ConstPtr contours_msg_ptr_;
    std::mutex contours_mutex_;

    boost::shared_ptr<boost::thread> process_contours_thread_ptr_;

    std::multiset<Buoy> buoys_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( BuoyFinder )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        multi_sub_.addSubscriber( nh_rel, "contours", &BuoyFinderNode::contoursCB, this );
        multi_pub_.addPublishers<_LandmarkArrayMsg, _MarkerArrayMsg>( nh_rel, { "landmarks", "markers" } );

        initPolicies<quickdev::policy::ALL>();

        process_contours_thread_ptr_ = boost::make_shared<boost::thread>( &BuoyFinderNode::processContours, this );
    }

    void processContours()
    {
        while( QUICKDEV_GET_RUNABLE_POLICY()::running() )
        {
            // make sure we're locked once
            process_contours_mutex_.try_lock();
            // wait for external unlock
            auto process_contours_lock = quickdev::make_unique_lock( process_contours_mutex_ );

            // get a copy of the contours so we block other processes for the minimum amount of time
            _ContourArrayMsg contours_msg;
            {
                auto contours_lock = quickdev::make_unique_lock( contours_mutex_ );
                contours_msg = *contours_msg_ptr_;
            }

            auto const contours = contours_msg.contours;

            // fit dem ellipses

            PRINT_INFO( "Evaluating %zu contours", contours.size() );

            buoys_.clear();

            for( auto contour_it = contours.cbegin(); contour_it != contours.cend(); ++contour_it )
            {
                auto const & contour_msg = *contour_it;

                if( contour_msg.points.size() < 6 ) continue;

                _Contour contour = unit::make_unit( contour_msg );

                cv::RotatedRect rect = cv::fitEllipse( cv::Mat( contour ) );

                double const diameter = std::min( rect.size.width, rect.size.height );
                double const aspect_ratio = (double)rect.size.height / (double)rect.size.width;

                if( diameter > config_.diameter_min && fabs( config_.aspect_ratio_mean - aspect_ratio ) < config_.aspect_ratio_variance )
                {
                    buoys_.insert( Buoy( Pose( Position( rect.center.x, rect.center.y ) ), Color( contour_msg.name ), Size( rect.size.width, rect.size.height ) ) );
                }
                else
                {
                    PRINT_WARN( "Aspect ratio (%f) or diameter (%f) outside of constraints", aspect_ratio, diameter );
                }
            }

            PRINT_INFO( "Found %zu buoys", buoys_.size() );

            _LandmarkArrayMsg buoys_msg;
            _MarkerArrayMsg markers_msg;

            size_t marker_id = 0;
            for( auto buoy_it = buoys_.cbegin(); buoy_it != buoys_.cend(); ++buoy_it )
            {
                auto const & buoy = *buoy_it;

                buoys_msg.landmarks.push_back( buoy );
                _MarkerMsg marker_msg = buoy;
                marker_msg.id = marker_id ++;

                markers_msg.markers.push_back( marker_msg );
            }

            if( !buoys_msg.landmarks.empty() ) multi_pub_.publish( "landmarks", buoys_msg, "markers", markers_msg );
        }
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( contoursCB, _ContourArrayMsg )
    {
        auto lock = quickdev::make_unique_lock( contours_mutex_, std::try_to_lock );

        if( !lock ) return;

        contours_msg_ptr_ = msg;

        // wake up processContours() for one cycle
        process_contours_mutex_.unlock();
    }

    QUICKDEV_SPIN_ONCE()
    {
        // just update ROS
    }
};

#endif // LANDMARKFINDER_BUOYFINDERNODE_H_
