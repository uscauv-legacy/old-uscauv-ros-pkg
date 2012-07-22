/***************************************************************************
 *  include/seabee3_demo/gate_task_node.h
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

#ifndef SEABEE3DEMO_GATETASKNODE_H_
#define SEABEE3DEMO_GATETASKNODE_H_

#include <quickdev/node.h>

// policies
#include <seabee3_common/seabee_movement_policy.h>
#include <quickdev/tf_tranceiver_policy.h>

// objects
#include <quickdev/multi_subscriber.h>
#include <quickdev/multi_publisher.h>

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

typedef quickdev::SimpleActionToken SimpleActionToken;

QUICKDEV_DECLARE_NODE( GateTask, _SeabeeMovementPolicy )

QUICKDEV_DECLARE_NODE_CLASS( GateTask )
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

    SimpleActionToken depth_token_;
    SimpleActionToken heading_token_;
    SimpleActionToken move_relative_token_;
    SimpleActionToken move_at_velocity_token_;

    XmlRpc::XmlRpcValue actions_;
    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( GateTask )
    {
        //
    }

    ~GateTaskNode()
    {
        kill_switch_enabled_condition_.notify_all();
        kill_switch_disabled_condition_.notify_all();
    }

    QUICKDEV_SPIN_FIRST()
    {
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        initPolicies<quickdev::policy::ALL>();

        multi_sub_.addSubscriber( nh_rel, "/seabee3/kill_switch", &GateTaskNode::killSwitchCB, this );

        actions_ = quickdev::ParamReader::readParam<decltype( actions_ )>( nh_rel, "actions" );

        main_loop_thread_ptr_ = boost::make_shared<boost::thread>( &GateTaskNode::mainLoop, this );
        detect_killed_thread_ptr_ = boost::make_shared<boost::thread>( &GateTaskNode::detectKilled, this );
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

    void detectKilled()
    {
        while( QUICKDEV_GET_RUNABLE_POLICY()::running() )
        {
            auto detect_killed_lock = quickdev::make_unique_lock( detect_killed_mutex_ );
            kill_switch_disabled_condition_.wait( detect_killed_lock );

            PRINT_INFO( "Detected kill signal; resetting all tokens." );

            if( move_relative_token_.ok() )
            {
                PRINT_INFO( "Cancelled move_relative token" );
                move_relative_token_.cancel();
            }

            if( move_at_velocity_token_.ok() )
            {
                PRINT_INFO( "Cancelled move_at_velocity token" );
                move_at_velocity_token_.cancel();
            }

            if( depth_token_.ok() )
            {
                PRINT_INFO( "Cancelled depth token" );
                depth_token_.cancel();
            }

            if( heading_token_.ok() )
            {
                PRINT_INFO( "Cancelled heading token" );
                heading_token_.cancel();
            }
        }
    }

    void diveAction( XmlRpc::XmlRpcValue & action )
    {
        double const timeout = quickdev::ParamReader::getXmlRpcValue<double>( action, "timeout" );
        double const value = quickdev::ParamReader::getXmlRpcValue<double>( action, "value" );

        PRINT_INFO( "Diving to %f for %f seconds", value, timeout );

        depth_token_.cancel();
        depth_token_ = _SeabeeMovementPolicy::diveTo( value );
        depth_token_.wait( timeout );
    }

    void velocityAction( XmlRpc::XmlRpcValue & action )
    {
        double const timeout = quickdev::ParamReader::getXmlRpcValue<double>( action, "timeout" );
        XmlRpc::XmlRpcValue velocity = quickdev::ParamReader::getXmlRpcValue<XmlRpc::XmlRpcValue>( action, "value" );
        btVector3 linear_velocity( velocity[0], velocity[1], velocity[2] );
        btQuaternion angular_velocity( (double)Radian( Degree( velocity[5] ) ), (double)Radian( Degree( velocity[4] ) ), (double)Radian( Degree( velocity[3] ) ) );

        PRINT_INFO( "Moving at velocity [ %f %f %f ] [ %f %f %f ] for %f seconds", linear_velocity.getX(), linear_velocity.getY(), linear_velocity.getZ(), (double)velocity[3], (double)velocity[4], (double)velocity[5], timeout );

        move_at_velocity_token_ = _SeabeeMovementPolicy::moveAtVelocity( btTransform( angular_velocity, linear_velocity ) );
        move_at_velocity_token_.wait( timeout );
        move_at_velocity_token_.cancel();
    }

    void headingAction( XmlRpc::XmlRpcValue & action )
    {
        double const timeout = quickdev::ParamReader::getXmlRpcValue<double>( action, "timeout" );
        double const angle = quickdev::ParamReader::getXmlRpcValue<double>( action, "angle" );

        PRINT_INFO( "Rotating to %f for %f seconds", angle, timeout );

        heading_token_.cancel();
        heading_token_ = _SeabeeMovementPolicy::faceTo( Radian( Degree( angle ) ) );
        heading_token_.wait( timeout );
    }

    void headingRelativeAction( XmlRpc::XmlRpcValue & action )
    {
        auto heading_transform = _TfTranceiverPolicy::tryLookupTransform( "/world", "/seabee3/sensors/imu" );
        double const timeout = quickdev::ParamReader::getXmlRpcValue<double>( action, "timeout" );
        double const angle = quickdev::ParamReader::getXmlRpcValue<double>( action, "angle");

        PRINT_INFO( "Rotating relative %f for %f seconds", angle, timeout );

        heading_token_.cancel();
        heading_token_ = _SeabeeMovementPolicy::faceTo( unit::convert<btVector3>( heading_transform.getRotation() ).getZ() + Radian( Degree( angle ) ) );
        heading_token_.wait( timeout );
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

                QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );
                actions_ = quickdev::ParamReader::readParam<decltype( actions_ )>( nh_rel, "actions" );

                heading_transform = _TfTranceiverPolicy::tryLookupTransform( "/world", "/seabee3/sensors/imu" );
            }

            heading_token_ = _SeabeeMovementPolicy::faceTo( unit::convert<btVector3>( heading_transform.getRotation() ).getZ() );

            for( int i = 0; i < actions_.size() && !isKilled(); ++i )
            {
                auto & action = actions_[i];
                auto type = quickdev::ParamReader::getXmlRpcValue<std::string>( action, "type", "unknown" );

                if( type == "depth" ) diveAction( action );
                else if( type == "velocity" ) velocityAction( action );
                else if( type == "heading" ) headingAction( action );
                else if( type == "heading_relative" ) headingRelativeAction( action );
                else
                {
                    PRINT_WARN( "Unknown action %s", type.c_str() );
                    exit( -1 );
                }
            }
/*
            PRINT_INFO( "Diving" );
            // dive
            {
                depth_token_ = _SeabeeMovementPolicy::diveTo( quickdev::ParamReader::getXmlRpcValue<double>( actions_, "depth", -3.0 ) );
                heading_token_ = _SeabeeMovementPolicy::faceTo( unit::convert<btVector3>( heading_transform.getRotation() ).getZ() );

                if( depth_token_.wait( 8.0 ) ) PRINT_INFO( "At depth" );
                if( heading_token_.wait( 5.0 ) ) PRINT_INFO( "At heading" );
            }

            {
                double const velocity = quickdev::ParamReader::getXmlRpcValue<double>( "
                PRINT_INFO( "Moving forward at %f%% thrust" );
                move_at_velocity_token_ = _SeabeeMovementPolicy::moveAtVelocity( btTransform( btQuaternion( 0, 0, 0 ), btVector3( 0.6, 0, 0 ) ) );
                move_at_velocity_token_.wait( quickdev::ParamReader::getXmlRpcValue<int>( params_, "time", 240 ) );
                move_at_velocity_token_.cancel();
            }
*/
        }
    }

    bool isKilled()
    {
        return last_kill_switch_msg_ && last_kill_switch_msg_->is_killed;
    }

    QUICKDEV_SPIN_ONCE()
    {
        //
    }
};

#endif // SEABEE3DEMO_GATETASKNODE_H_
