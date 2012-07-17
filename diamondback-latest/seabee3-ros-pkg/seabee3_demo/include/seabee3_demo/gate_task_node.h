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

QUICKDEV_DECLARE_NODE( GateTask, _SeabeeMovementPolicy )

QUICKDEV_DECLARE_NODE_CLASS( GateTask )
{
protected:
    ros::MultiSubscriber<> multi_sub_;
    ros::MultiPublisher<> multi_pub_;

    boost::shared_ptr<boost::thread> main_loop_thread_ptr_;

    std::mutex kill_switch_mutex_;
    std::condition_variable kill_switch_condition_;

    XmlRpc::XmlRpcValue params_;
    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( GateTask )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        initPolicies<quickdev::policy::ALL>();

        multi_sub_.addSubscriber( nh_rel, "/seabee3/kill_switch", &GateTaskNode::killSwitchCB, this );
        multi_pub_.addPublishers<_TwistMsg>( nh_rel, { "/seabee3/cmd_vel" } );

        params_ = quickdev::ParamReader::readParam<decltype( params_ )>( nh_rel, "params" );

        main_loop_thread_ptr_ = boost::make_shared<boost::thread>( &GateTaskNode::mainLoop, this );
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( killSwitchCB, _KillSwitchMsg )
    {
        PRINT_INFO( "Got killswitch message." );
        auto lock = quickdev::make_unique_lock( kill_switch_mutex_, std::try_to_lock );

        if( !lock ) return;

        if( !msg->is_killed ) kill_switch_condition_.notify_all();
    }

    void mainLoop()
    {
        btTransform heading_transform;

        PRINT_INFO( "Waiting for kill switch..." );
        // wait for kill switch to be un-killed
        {
            auto kill_switch_lock = quickdev::make_unique_lock( kill_switch_mutex_);
            kill_switch_condition_.wait( kill_switch_lock );

            _TfTranceiverPolicy::tryLookupTransform( "/world", "/seabee3/sensors/imu" );
        }

        PRINT_INFO( "Diving" );
        // dive to 0.65 m
        {
            /*
            auto token = _SeabeeMovementPolicy::moveRelativeTo( "current_pose", btTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( 0, 0, -0.75 ) ) );
            token.wait();
            */

            auto start = ros::Time::now();

            while( ros::ok() && ( ros::Time::now() - start ).toSec() < 5 )
            {
                _TfTranceiverPolicy::publishTransform( btTransform( heading_transform.getRotation(), btVector3( 0, 0, -1 ) ), "/world", "/seabee3/desired_pose" );
            }
        }

        PRINT_INFO( "Moving forward" );
        // drive forward at 0.2 m/s for 60 seconds
        {
            auto start = ros::Time::now();

            _TwistMsg velocity;
            velocity.linear.x = 0.2;

            while( ros::ok() && ( ros::Time::now() - start ).toSec() < 60 )
            {
                multi_pub_.publish( "/seabee3/cmd_vel", velocity );
            }
            //auto token = _SeabeeMovementPolicy::moveAtVelocity( btTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( 0.2, 0, 0 ) ) );
            //ros::Duration( 60 ).sleep();
            //token.cancel();
        }
    }

    QUICKDEV_SPIN_ONCE()
    {
        //
    }
};

#endif // SEABEE3DEMO_GATETASKNODE_H_
