/***************************************************************************
 *  include/seabee3_sensor_aggregator/seabee3_sensor_aggregator_node.h
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

#ifndef SEABEE3SENSORAGGREGATOR_SEABEE3SENSORAGGREGATORNODE_H_
#define SEABEE3SENSORAGGREGATOR_SEABEE3SENSORAGGREGATORNODE_H_

#include <quickdev/node.h>

// policies
#include <quickdev/tf_tranceiver_policy.h>

// objects
#include <quickdev/multi_subscriber.h>
#include <quickdev/multi_publisher.h>

// utils
#include <quickdev/geometry_message_conversions.h>

// msgs
#include <seabee3_msgs/Depth.h>
#include <seabee3_msgs/Imu.h>

typedef seabee3_msgs::Depth _DepthMsg;
typedef seabee3_msgs::Imu _ImuMsg;

typedef quickdev::TfTranceiverPolicy _TfTranceiverPolicy;

QUICKDEV_DECLARE_NODE( Seabee3SensorAggregator, _TfTranceiverPolicy )

QUICKDEV_DECLARE_NODE_CLASS( Seabee3SensorAggregator )
{
    ros::MultiPublisher<> multi_pub_;
    ros::MultiSubscriber<> multi_sub_;
    tf::StampedTransform combined_frame_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( Seabee3SensorAggregator )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        multi_sub_.addSubscriber( nh_rel, "imu", &Seabee3SensorAggregatorNode::imuCB, this );
        multi_sub_.addSubscriber( nh_rel, "depth", &Seabee3SensorAggregatorNode::depthCB, this );

        combined_frame_.frame_id_ = "/world";
        combined_frame_.child_frame_id_ = "/seabee3/observed_pose";

        //multi_sub_.addSubscriber( nh_rel, "physics_state", &Seabee3SensorAggregatorNode::physicsStateCB, this );

        //multi_pub_.addPublisher(  );

        initPolicies<quickdev::policy::ALL>();
    }

    QUICKDEV_SPIN_ONCE()
    {
        _TfTranceiverPolicy::publishTransform( combined_frame_ );
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( imuCB, _ImuMsg )
    {
        combined_frame_.setRotation( unit::implicit_convert( msg->ori ) );
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( depthCB, _DepthMsg )
    {
        combined_frame_.getOrigin().setZ( -msg->value );
    }
};

#endif // SEABEE3SENSORAGGREGATOR_SEABEE3SENSORAGGREGATORNODE_H_
