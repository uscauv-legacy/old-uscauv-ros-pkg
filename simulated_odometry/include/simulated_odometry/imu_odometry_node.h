/***************************************************************************
 *  include/simulated_odometry/imu_odometry_node.h
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

#ifndef SIMULATEDODOMETRY_IMUODOMETRYNODE_H_
#define SIMULATEDODOMETRY_IMUODOMETRYNODE_H_

#include <quickdev/node.h>

#include <quickdev/tf_tranceiver_policy.h>

#include <quickdev/multi_publisher.h>
#include <quickdev/multi_subscriber.h>

#include <quickdev/geometry_message_conversions.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

typedef quickdev::TfTranceiverPolicy _TfTranceiverPolicy;

QUICKDEV_DECLARE_NODE( IMUOdometry, _TfTranceiverPolicy )

QUICKDEV_DECLARE_NODE_CLASS( IMUOdometry )
{
protected:
    typedef nav_msgs::Odometry _OdometryMsg;
    typedef sensor_msgs::Imu _ImuMsg;

    ros::MultiPublisher<> multi_pub_;
    ros::MultiSubscriber<> multi_sub_;

    btTransform pose_;
    btVector3 linear_velocity_;
    btVector3 angular_velocity_;

    btVector3 net_compensated_linear_acceleration_;
    size_t num_samples_;

    _ImuMsg::ConstPtr last_imu_msg_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( IMUOdometry ),
        pose_( btQuaternion( 0, 0, 0, 1 ) ),
        num_samples_( 0 )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        multi_sub_.addSubscriber( nh_rel, "imu", &IMUOdometryNode::imuCB, this );
        multi_pub_.addPublishers<_OdometryMsg>( nh_rel, { "odometry" } );

        initPolicies<quickdev::policy::ALL>();
    }

    // the IMU can give us angular position, angular velocity, and linear acceleration
    QUICKDEV_DECLARE_MESSAGE_CALLBACK( imuCB, _ImuMsg )
    {
        _ImuMsg::ConstPtr last_imu_msg = last_imu_msg_;
        if( last_imu_msg )
        {
            double const dt = ( msg->header.stamp - last_imu_msg->header.stamp ).toSec();

            btVector3 const linear_acceleration = unit::implicit_convert( msg->linear_acceleration );

            printf( "dt: %f\n", dt );
            printf( "accel: %f %f %f (%f)\n", linear_acceleration.getX(), linear_acceleration.getY(), linear_acceleration.getZ(), linear_acceleration.length() );
//            printf( "rot comp accel: %f %f %f (%f)\n", rotation_compensated_linear_acceleration.getX(), rotation_compensated_linear_acceleration.getY(), rotation_compensated_linear_acceleration.getZ(), rotation_compensated_linear_acceleration.length() );

            btVector3 const change_in_linear_velocity = linear_acceleration * dt;

            printf( "change lin vel: %f %f %f\n", change_in_linear_velocity.getX(), change_in_linear_velocity.getY(), change_in_linear_velocity.getZ() );

            linear_velocity_ += change_in_linear_velocity;

            printf( "lin vel: %f %f %f\n", linear_velocity_.getX(), linear_velocity_.getY(), linear_velocity_.getZ() );

            angular_velocity_ = unit::implicit_convert( msg->angular_velocity );

            printf( "ang vel: %f %f %f\n", angular_velocity_.getX(), angular_velocity_.getY(), angular_velocity_.getZ() );

            btVector3 const change_in_position = linear_velocity_ * dt;

            printf( "change pos: %f %f %f\n", change_in_position.getX(), change_in_position.getY(), change_in_position.getZ() );

//            btVector3 const change_in_orientation_rpy = angular_velocity_ * dt;
//            btQuaternion const change_in_orientation = unit::implicit_convert( change_in_orientation_rpy );

//            printf( "change ori: %f %f %f\n", change_in_orientation_rpy.getX(), change_in_orientation_rpy.getY(), change_in_orientation_rpy.getZ() );

            pose_.setRotation( unit::implicit_convert( msg->orientation ) );

            btTransform change_in_pose( btQuaternion( 0, 0, 0, 1 ), change_in_position );

            pose_ *= change_in_pose;
        }

        last_imu_msg_ = msg;

        _OdometryMsg odometry_msg;
        odometry_msg.header.stamp = msg->header.stamp;
        odometry_msg.header.frame_id = "/world";
        odometry_msg.child_frame_id = msg->header.frame_id;

        odometry_msg.pose.pose = unit::implicit_convert( pose_ );
        odometry_msg.twist.twist.linear = unit::implicit_convert( linear_velocity_ );
        odometry_msg.twist.twist.angular = unit::implicit_convert( angular_velocity_ );

        multi_pub_.publish( "odometry", odometry_msg );

        _TfTranceiverPolicy::publishTransform( pose_, odometry_msg.header.frame_id, "/seabee3/odometry/imu", odometry_msg.header.stamp );
    }

    QUICKDEV_SPIN_ONCE()
    {
        //
    }
};

#endif // SIMULATEDODOMETRY_IMUODOMETRYNODE_H_
