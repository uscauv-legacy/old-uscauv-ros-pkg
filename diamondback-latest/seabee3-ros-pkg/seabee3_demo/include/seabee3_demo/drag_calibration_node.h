/***************************************************************************
 *  include/seabee3_demo/drag_calibration_node.h
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

#ifndef SEABEE3DEMO_DRAGCALIBRATIONNODE_H_
#define SEABEE3DEMO_DRAGCALIBRATIONNODE_H_

#include <quickdev/node.h>

// objects
#include <quickdev/multi_publisher.h>

// policies
#include <quickdev/tf_tranceiver_policy.h>

// msgs
#include <geometry_msgs/Twist.h>

typedef quickdev::TfTranceiverPolicy _TfTranceiverPolicy;

typedef geometry_msgs::Twist _TwistMsg;

QUICKDEV_DECLARE_NODE( DragCalibration, _TfTranceiverPolicy )

QUICKDEV_DECLARE_NODE_CLASS( DragCalibration )
{
    ros::MultiPublisher<> multi_pub_;

    double forward_velocity_;
    btTransform starting_transform_;
    btTransform output_transform_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( DragCalibration )
    {
        initPolicies<QUICKDEV_GET_RUNABLE_POLICY()>();
    }

    QUICKDEV_SPIN_FIRST()
    {
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        multi_pub_.addPublishers<_TwistMsg>( nh_rel, { "cmd_vel" } );

        // do an initial dive

        _TwistMsg velocity_msg;
        velocity_msg.linear.z = -0.05;

        btTransform current_pose;

        do
        {
            multi_pub_.publish( "cmd_vel", velocity_msg );
            QUICKDEV_GET_RUNABLE_POLICY()::getLoopRate()->sleep();
            current_pose = _TfTranceiverPolicy::waitForAndLookupTransform( "/world", "/seabee3/desired_pose", 10, 0 );
        }
        while( current_pose.getOrigin().getZ() >= -0.1534 );

        initPolicies<quickdev::policy::ALL>();
    }

    QUICKDEV_SPIN_ONCE()
    {
        auto current_pose = _TfTranceiverPolicy::waitForAndLookupTransform( "/world", "/seabee3/desired_pose", 10, 0 );

        if( current_pose.getOrigin().getX() > 0.5 ) return QUICKDEV_GET_RUNABLE_POLICY()::interrupt();

        _TwistMsg velocity_msg;
        velocity_msg.linear.x = 0.2;
        multi_pub_.publish( "cmd_vel", velocity_msg );
    }
};

#endif // SEABEE3DEMO_DRAGCALIBRATIONNODE_H_
