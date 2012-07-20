/***************************************************************************
 *  include/seabee3_demo/buoy_task_node.h
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
 *  * Neither the name of usc-ros-pkg nor the names of its
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

#ifndef SEABEE3DEMO_BUOYTASKNODE_H_
#define SEABEE3DEMO_BUOYTASKNODE_H_

#include <quickdev/node.h>

// policies
#include <seabee3_common/seabee_movement_policy.h>
#include <seabee3_common/seabee_recognition_policy.h>

// utils
#include <seabee3_common/recognition_primitives.h>

// msgs
#include <seabee3_msgs/KillSwitch.h>
#include <geometry_msgs/Twist.h>

using namespace seabee;

typedef seabee3_msgs::KillSwitch _KillSwitchMsg;
typedef geometry_msgs::Twist _TwistMsg;

typedef SeabeeMovementPolicy _SeabeeMovementPolicy;
typedef quickdev::TfTranceiverPolicy _TfTranceiverPolicy;
typedef SeabeeRecognitionPolicy _SeabeeRecognitionPolicy;

typedef quickdev::SimpleActionToken SimpleActionToken;

QUICKDEV_DECLARE_NODE( BuoyTask, _SeabeeMovementPolicy, _SeabeeRecognitionPolicy )

QUICKDEV_DECLARE_NODE_CLASS( BuoyTask )
{
protected:
    ros::MultiSubscriber<> multi_sub_;

    boost::shared_ptr<boost::thread> main_loop_thread_ptr_;
    boost::shared_ptr<boost::thread> detect_killed_thread_ptr_;

    _KillSwitchMsg::ConstPtr last_kill_switch_msg_;
    std::mutex last_kill_switch_msg_mutex_;

    std::mutex detect_killed_mutex_;

    std::mutex kill_switch_enabled_mutex_;
    std::condition_variable kill_switch_enabled_condition_;

    std::mutex kill_switch_disabled_mutex_;
    std::condition_variable kill_switch_disabled_condition_;

    std::vector<Landmark> desired_landmarks_;
    std::vector<Landmark>::const_iterator current_landmark_it_;

    SimpleActionToken depth_token_;
    SimpleActionToken heading_token_;
    SimpleActionToken move_relative_token_;
    SimpleActionToken move_at_velocity_token_;
    SimpleActionToken find_landmark_token_;
    SimpleActionToken rotate_search_token_;

    XmlRpc::XmlRpcValue params_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( BuoyTask ),
        desired_landmarks_( { Landmark( Buoy( Color( "orange" ) ) ), Landmark( Buoy( Color( "yellow" ) ) ) } ),
        current_landmark_it_( desired_landmarks_.cbegin() )
    {
        //
    }

    ~BuoyTaskNode()
    {
        kill_switch_enabled_condition_.notify_all();
        kill_switch_disabled_condition_.notify_all();
    }

    QUICKDEV_SPIN_FIRST()
    {
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        initPolicies<quickdev::policy::ALL>();

        multi_sub_.addSubscriber( nh_rel, "/seabee3/kill_switch", &BuoyTaskNode::killSwitchCB, this );

        params_ = quickdev::ParamReader::readParam<decltype( params_ )>( nh_rel, "params" );

        main_loop_thread_ptr_ = boost::make_shared<boost::thread>( &BuoyTaskNode::mainLoop, this );
        detect_killed_thread_ptr_ = boost::make_shared<boost::thread>( &BuoyTaskNode::detectKilled, this );
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( killSwitchCB, _KillSwitchMsg )
    {
        auto lock = quickdev::make_unique_lock( last_kill_switch_msg_mutex_, std::try_to_lock );

        if( !lock ) return;

        // detect change from killed to not killed
        if( last_kill_switch_msg_ && last_kill_switch_msg_->is_killed && !msg->is_killed ) kill_switch_enabled_condition_.notify_all();
        // detect change from not killed to killed
        if( last_kill_switch_msg_ && !last_kill_switch_msg_->is_killed && msg->is_killed ) kill_switch_disabled_condition_.notify_all();

        last_kill_switch_msg_ = msg;
    }

    bool isKilled()
    {
        return !last_kill_switch_msg_ || last_kill_switch_msg_->is_killed;
    }

    void detectKilled()
    {
        while( QUICKDEV_GET_RUNABLE_POLICY()::running() )
        {
            auto detect_killed_lock = quickdev::make_unique_lock( detect_killed_mutex_ );
            kill_switch_disabled_condition_.wait( detect_killed_lock );

            PRINT_INFO( "Detected kill signal; resetting all tokens." );

            move_relative_token_.cancel();
            move_at_velocity_token_.cancel();
            depth_token_.cancel();
            heading_token_.cancel();
            find_landmark_token_.cancel();
            rotate_search_token_.cancel();
        }
    }

/*
    std::unique_lock<std::mutex> findBuoys( std::set<std::string> const & colors )
    {
        auto landmarks_map_lock = quickdev::make_unique_lock( landmarks_map_mutex_, std::defer_lock );

        findBuoys( colors, landmarks_map_lock );

        return landmarks_map_lock;
    }
*/
/*
    void findBuoys( std::set<std::string> const & colors, std::unique_lock<std::mutex> landmarks_map_lock )
    {
        while( true )
        {
            auto find_buoys_lock = quickdev::make_unique_lock( find_buoys_mutex_ );
            find_buoys_condition_.wait( find_buoys_lock );

            if( !QUICKDEV_GET_RUNABLE_POLICY()::running() ) return;

            landmarks_map_lock.lock();

            std::set<std::string> colors_remaining = colors;

            for( auto landmark_it = landmarks_map_.cbegin(); landmark_it != landmarks_map_.cend(); ++landmark_it )
            {
                auto const & landmark = landmark_it->second;

                std::string const color = landmark.color_;

                if( colors.count( color ) )
                {
                    colors_remaining.erase( color );
                    if( colors_remaining.empty() ) break;
                }
            }

            if( colors_remaining.empty() ) return;
        }
    }
*/
    void boopBuoy()
    {
        move_at_velocity_token_ = _SeabeeMovementPolicy::moveAtVelocity( btTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( 0.2, 0, 0 ) ) );
        move_at_velocity_token_.wait( 6 );
        move_at_velocity_token_.cancel();

        move_at_velocity_token_ = _SeabeeMovementPolicy::moveAtVelocity( btTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( -0.4, 0, 0 ) ) );
        move_at_velocity_token_.wait( 6 );
        move_at_velocity_token_.cancel();
    }

    void mainLoop()
    {
        btTransform heading_transform;

        while( QUICKDEV_GET_RUNABLE_POLICY()::running() )
        {
            PRINT_INFO( "Waiting for kill switch..." );
            // wait for kill switch to be un-killed
            {
                auto kill_switch_enabled_lock = quickdev::make_unique_lock( kill_switch_enabled_mutex_ );
                kill_switch_enabled_condition_.wait( kill_switch_enabled_lock );

                heading_transform = _TfTranceiverPolicy::tryLookupTransform( "/world", "/seabee3/sensors/imu" );
            }

            PRINT_INFO( "Diving" );
            // dive
            {
                depth_token_ = _SeabeeMovementPolicy::diveTo( -0.3 );
                heading_token_ = _SeabeeMovementPolicy::faceTo( unit::convert<btVector3>( heading_transform.getRotation() ).getZ() );

                if( !isKilled() && depth_token_.wait( 5.0 ) ) PRINT_INFO( "At depth" );
                if( !isKilled() && heading_token_.wait( 5.0 ) ) PRINT_INFO( "At heading" );
            }

            PRINT_INFO( "Moving forward" );
            // drive forward at 0.2 m/s for 60 seconds
            {
                move_at_velocity_token_ = _SeabeeMovementPolicy::moveAtVelocity( btTransform( btQuaternion( 0, 0, 0 ), btVector3( 0.2, 0, 0 ) ) );
                move_at_velocity_token_.wait( 3 );
                move_at_velocity_token_.cancel();
            }

            auto const heading_transform_ = _TfTranceiverPolicy::tryLookupTransform( "/world", "/seabee3/sensors/imu" );

            PRINT_INFO( "Searching for landmarks" );
            // rotate left and right while looking for buoys
            for( auto landmark_it = desired_landmarks_.cbegin(); landmark_it != desired_landmarks_.cend() && !isKilled(); ++landmark_it )
            {
                {
                    auto lock = quickdev::make_unique_lock( landmarks_map_mutex_ );
                    landmarks_map_.clear();
                }
                auto const & current_landmark = *landmark_it;

                PRINT_INFO( "Looking for landmark: %s", current_landmark.name_.c_str() );

                find_landmark_token_ = _SeabeeRecognitionPolicy::findLandmark( current_landmark );

                size_t attempts = 0;

                while( attempts < 3 && !find_landmark_token_.success() && find_landmark_token_.ok() )
                {
                    heading_token_.cancel();
                    // start a search
                    PRINT_INFO( "Searching for landmark (attempt %zu)", attempts );
                    rotate_search_token_ = _SeabeeMovementPolicy::rotateSearch( find_landmark_token_, Radian( Degree( -45 ) ), Radian( Degree( 45.0 ) ), Radian( Degree( 5 ) ) );

                    // if the search successfully completes, we can move on to trying to hit the buoy
                    if( rotate_search_token_.wait( 90 / 5 ) )
                    {
                        PRINT_INFO( "Landmark found." );
                        break;
                    }
                    // otherwise if the search timed out, cancel it and prepare to move forward to start another search
                    else
                    {
                        PRINT_INFO( "Landmark not found; retrying." );
                        rotate_search_token_.cancel();
                    }

                    // face our initial heading
                    PRINT_INFO( "Resetting heading" );
                    heading_token_ = _SeabeeMovementPolicy::faceTo( unit::convert<btVector3>( heading_transform.getRotation() ).getZ() );
                    heading_token_.wait( 5 );

                    // move forward a bit
                    PRINT_INFO( "Moving forward." );
                    move_at_velocity_token_ = _SeabeeMovementPolicy::moveAtVelocity( btTransform( btQuaternion( 0, 0, 0 ), btVector3( 0.2, 0, 0 ) ) );
                    move_at_velocity_token_.wait( 10 );
                    move_at_velocity_token_.cancel();

                    attempts ++;
                }

                PRINT_INFO( "Halting landmark search." );
                // we either found the buoy or timed out at this point, so we can cancel the buoy finding code
                find_landmark_token_.cancel();

                PRINT_INFO( "Checking for landmark in list of %zu landmarks.", landmarks_map_.size() );

                auto landmark_it = landmarks_map_.find( current_landmark.getUniqueName() );
                if( landmark_it != landmarks_map_.end() )
                {
                    PRINT_INFO( "Landmark %s found; aligning to buoy", landmark_it->first.c_str() );

                    depth_token_.cancel();

                    auto const & buoy = landmark_it->second;

                    move_relative_token_ = _SeabeeMovementPolicy::moveRelativeTo( landmark_it->first, btTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( -0.5, 0, 0 ) ) );
                    if( move_relative_token_.wait( 10 ) )
                    {
                        PRINT_INFO( "Hitting buoy" );
                        boopBuoy();
                    }
                }
                else
                {
                    PRINT_INFO( "Landmark not found; moving to next" );
                }
            }
        }
    }

    QUICKDEV_SPIN_ONCE()
    {
        // just update ROS
    }
};

#endif // SEABEE3DEMO_BUOYTASKNODE_H_
