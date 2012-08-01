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
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTA L,
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
        desired_landmarks_( { Landmark( Buoy( Color( "orange" ) ) ) /*, Landmark( Buoy( Color( "yellow" ) ) )*/ } ),
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
        move_at_velocity_token_ = _SeabeeMovementPolicy::moveAtVelocity( btTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( 0.2, 0, 0 ) ), quickdev::auto_bind( &BuoyTaskNode::isKilled, this ) );
        move_at_velocity_token_.wait( 6 );
        move_at_velocity_token_.cancel();

        move_at_velocity_token_ = _SeabeeMovementPolicy::moveAtVelocity( btTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( -0.4, 0, 0 ) ), quickdev::auto_bind( &BuoyTaskNode::isKilled, this ) );
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
                heading_token_ = _SeabeeMovementPolicy::faceTo( unit::convert<btVector3>( heading_transform.getRotation() ).getZ(), quickdev::auto_bind( &BuoyTaskNode::isKilled, this ) );
                depth_token_ = _SeabeeMovementPolicy::diveTo( -0.35, quickdev::auto_bind( &BuoyTaskNode::isKilled, this ) );

                if( depth_token_.wait( 4.0 ) ) PRINT_INFO( "At depth" );
                if( heading_token_.wait( 5.0 ) ) PRINT_INFO( "At heading" );

                PRINT_INFO( "Done diving" );
            }

            bool buoy_found = false;

            ros::Rate find_buoy_rate( 20 );

            while( ros::ok() && !isKilled() && !buoy_found )
            {
                try
                {
                    auto world_to_buoy = _TfTranceiverPolicy::lookupTransform( "/world", "/buoy_orange" );
                    buoy_found = true;
                }
                catch( std::exception e )
                {
                    PRINT_INFO( "Waiting for buoy" );
                }

                find_buoy_rate.sleep();
            }

            bool aligned = false;
            ros::Rate align_to_buoy_rate( 20.0 );

            depth_token_.cancel();

            do
            {
                try
                {
                    auto world_to_buoy = _TfTranceiverPolicy::lookupTransform( "/world", "/buoy_orange" );
                    auto world_to_self = _TfTranceiverPolicy::lookupTransform( "/world", "/seabee3/current_pose" );

                    _TfTranceiverPolicy::publishTransform( btTransform( world_to_self.getRotation(), btVector3( world_to_buoy.getOrigin().getX() - 1.0, world_to_buoy.getOrigin().getY(), world_to_buoy.getOrigin().getZ() ) ), "/world", "/seabee3/desired_pose" );

                    auto world_to_desired = _TfTranceiverPolicy::lookupTransform( "/world", "/seabee3/desired_pose" );

                    if( world_to_self.getOrigin().distance( world_to_desired.getOrigin() ) < 0.2 )
                    {
                        depth_token_ = _SeabeeMovementPolicy::diveTo( world_to_buoy.getOrigin().getZ(), quickdev::auto_bind( &BuoyTaskNode::isKilled, this ) );
                        aligned = true;
                    }
                    else
                    {
                        PRINT_INFO( "Aligning to buoy" );
                    }
                }
                catch( std::exception e )
                {
                    PRINT_INFO( "%s", e.what() );
                }

                align_to_buoy_rate.sleep();
            }
            while( ros::ok() && !isKilled() && !aligned );

            if( ros::ok() && !isKilled() ) boopBuoy();

            PRINT_INFO( "Done" );
        }
    }

    QUICKDEV_SPIN_ONCE()
    {
        // just update ROS
    }
};

#endif // SEABEE3DEMO_BUOYTASKNODE_H_
