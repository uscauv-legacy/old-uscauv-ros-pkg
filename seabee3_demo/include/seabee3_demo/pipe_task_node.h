/***************************************************************************
 *  include/seabee3_demo/pipe_task_node.h
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

#ifndef SEABEE3DEMO_PIPETASKNODE_H_
#define SEABEE3DEMO_PIPETASKNODE_H_

#include <quickdev/node.h>

// policies
#include <seabee3_common/seabee_movement_policy.h>
#include <seabee3_common/seabee_recognition_policy.h>
#include <quickdev/service_client_policy.h>

// utils
#include <seabee3_common/recognition_primitives.h>

// msgs
#include <seabee3_msgs/KillSwitch.h>
#include <geometry_msgs/Twist.h>

#include <seabee3_msgs/CalibrateRPY.h> // for CalibrateRPY

using namespace seabee;

typedef seabee3_msgs::KillSwitch _KillSwitchMsg;
typedef geometry_msgs::Twist _TwistMsg;

typedef seabee3_msgs::CalibrateRPY _CalibrateRPYSrv;

typedef SeabeeMovementPolicy _SeabeeMovementPolicy;
typedef quickdev::TfTranceiverPolicy _TfTranceiverPolicy;
typedef SeabeeRecognitionPolicy _SeabeeRecognitionPolicy;

typedef quickdev::SimpleActionToken SimpleActionToken;
typedef quickdev::ServiceClientPolicy<_CalibrateRPYSrv> _CalibrateRPYOriServiceClientPolicy;

QUICKDEV_DECLARE_NODE( PipeTask, _SeabeeMovementPolicy, _SeabeeRecognitionPolicy, _CalibrateRPYOriServiceClientPolicy )

QUICKDEV_DECLARE_NODE_CLASS( PipeTask )
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

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( PipeTask ),
        desired_landmarks_( { Landmark( Pipe( Pose( Orientation( Radian( Degree( 90 ) ) ) ) ) ) /*, Landmark( Pipe( Color( "yellow" ) ) )*/ } ),
        current_landmark_it_( desired_landmarks_.cbegin() )
    {
        //
    }

    ~PipeTaskNode()
    {
        kill_switch_enabled_condition_.notify_all();
        kill_switch_disabled_condition_.notify_all();
    }

    QUICKDEV_SPIN_FIRST()
    {
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        initPolicies<_CalibrateRPYOriServiceClientPolicy>( "service_name_param", std::string( "/xsens_driver/calibrate_rpy_ori" ) );

        initPolicies<quickdev::policy::ALL>();

        multi_sub_.addSubscriber( nh_rel, "/seabee3/kill_switch", &PipeTaskNode::killSwitchCB, this );

        params_ = quickdev::ParamReader::readParam<decltype( params_ )>( nh_rel, "params" );

        main_loop_thread_ptr_ = boost::make_shared<boost::thread>( &PipeTaskNode::mainLoop, this );
        detect_killed_thread_ptr_ = boost::make_shared<boost::thread>( &PipeTaskNode::detectKilled, this );
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
        return last_kill_switch_msg_ && last_kill_switch_msg_->is_killed;
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

    void mainLoop()
    {
        btTransform heading_transform;

        _CalibrateRPYSrv calibrate_ori_srv;
        calibrate_ori_srv.request.num_samples = 55;
        _CalibrateRPYOriServiceClientPolicy::callService( calibrate_ori_srv );

        move_relative_token_ = _SeabeeMovementPolicy::moveRelativeTo( "/pipe_orange", btTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( 0, -0.155575, 0.6 ) ) );

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
                heading_token_ = _SeabeeMovementPolicy::faceTo( unit::convert<btVector3>( heading_transform.getRotation() ).getZ(), quickdev::auto_bind( &PipeTaskNode::isKilled, this ) );
                depth_token_ = _SeabeeMovementPolicy::diveTo( -0.20, quickdev::auto_bind( &PipeTaskNode::isKilled, this ) );

                if( depth_token_.wait( 4.0 ) ) PRINT_INFO( "At depth" );
                if( heading_token_.wait( 5.0 ) ) PRINT_INFO( "At heading" );

                PRINT_INFO( "Done diving" );
            }

            find_landmark_token_ = _SeabeeRecognitionPolicy::findLandmark( *current_landmark_it_, quickdev::auto_bind( &PipeTaskNode::isKilled, this ) );

            PRINT_INFO( "Moving forward" );
            // drive forward
            {
                move_at_velocity_token_ = _SeabeeMovementPolicy::moveAtVelocity( btTransform( btQuaternion( 0, 0, 0 ), btVector3( 0.2, 0, 0 ) ), quickdev::action_token::make_term_criteria( find_landmark_token_ ) );
                move_at_velocity_token_.wait( 20 );
                move_at_velocity_token_.cancel();
                find_landmark_token_.cancel();

                PRINT_INFO( "Done moving forward" );
            }

            heading_transform = _TfTranceiverPolicy::tryLookupTransform( "/world", "/seabee3/sensors/imu" );
/*
            move_relative_token_ = _SeabeeMovementPolicy::moveRelativeTo( Landmark( Pipe( Color( "orange" ) ) ), btTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( -0.5, 0, 0 ) ) );
            if( move_relative_token_.wait( 60 ) )
            {
                PRINT_INFO( "Hitting pipe" );
                boopPipe();
            }
            else move_relative_token_.cancel();
*/

            auto const & current_landmark = *current_landmark_it_;

            PRINT_INFO( "Looking for landmark: %s", current_landmark.name_.c_str() );

            find_landmark_token_ = _SeabeeRecognitionPolicy::findLandmark( current_landmark, quickdev::auto_bind( &PipeTaskNode::isKilled, this ) );

            PRINT_INFO( "Checking for landmark in list of %zu landmarks.", landmarks_map_.size() );

            auto landmark_it = landmarks_map_.find( current_landmark.getUniqueName() );
            if( landmark_it != landmarks_map_.end() )
            {
                PRINT_INFO( "Landmark %s found; aligning to pipe", landmark_it->first.c_str() );

                depth_token_.cancel();
                heading_token_.cancel();

                auto const & pipe = landmark_it->second;

                heading_token_ = _SeabeeMovementPolicy::faceTo( pipe.pose_.orientation_.yaw_, quickdev::auto_bind( &PipeTaskNode::isKilled, this ) );
                move_relative_token_ = _SeabeeMovementPolicy::moveRelativeTo( landmark_it->first, btTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( 0, 0, 0.8 ) ), quickdev::auto_bind( &PipeTaskNode::isKilled, this ) );
                if( move_relative_token_.wait( 20 ) )
                {
                    PRINT_INFO( "Aligned to pipe" );
                }
            }
            else
            {
                PRINT_INFO( "Landmark not found; moving to next" );
            }
        }
    }

    QUICKDEV_SPIN_ONCE()
    {
        // just update ROS
    }
};

#endif // SEABEE3DEMO_PIPETASKNODE_H_
