/***************************************************************************
 *  include/seabee3_controls/simple_controls_node.h
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

#ifndef SEABEE3CONTROLS_SIMPLECONTROLSNODE_H_
#define SEABEE3CONTROLS_SIMPLECONTROLSNODE_H_

#include <quickdev/node.h>

// policies
#include <quickdev/service_server_policy.h>
#include <quickdev/reconfigure_policy.h>
#include <quickdev/tf_tranceiver_policy.h>

// objects
#include <quickdev/controllers/reconfigurable_pid.h>
#include <quickdev/multi_publisher.h>
#include <quickdev/multi_subscriber.h>

// utils
#include <quickdev/geometry_message_conversions.h>
#include <seabee3_common/movement.h>
#include <quickdev/math.h>

// msgs
#include <seabee3_msgs/MotorVals.h>
#include <geometry_msgs/Twist.h>

// srvs
#include <std_srvs/Empty.h>

// cfgs
#include <seabee3_controls/Seabee3ControlsConfig.h>

typedef seabee3_msgs::MotorVals _MotorValsMsg;
typedef geometry_msgs::Twist _TwistMsg;

typedef std_srvs::Empty _ResetPoseService;

typedef seabee3_controls::Seabee3ControlsConfig _Seabee3ControlsCfg;

typedef quickdev::TfTranceiverPolicy _TfTranceiverPolicy;
typedef quickdev::ServiceServerPolicy<_ResetPoseService> _ResetPoseServiceServer;
typedef quickdev::ReconfigurePolicy<_Seabee3ControlsCfg> _Seabee3ControlsLiveParams;

using namespace seabee3_common;

QUICKDEV_DECLARE_NODE( SimpleControls, _ResetPoseServiceServer, _Seabee3ControlsLiveParams, _TfTranceiverPolicy )

QUICKDEV_DECLARE_NODE_CLASS( SimpleControls )
{
protected:
    ros::MultiPublisher<> multi_pub_;
    ros::MultiSubscriber<> multi_sub_;

    // pid for depth and heading
    typedef quickdev::ReconfigurablePID<2> _Pid2D;
    _Pid2D pid_;

    _TwistMsg::ConstPtr last_velocity_msg_ptr_;
    std::mutex last_velocity_mutex_;

    btTransform world_to_desired_pose_tf_;
    std::mutex world_to_desired_pose_mutex_;

    btTransform world_to_current_pose_tf_;
    std::mutex world_to_current_pose_mutex_;

    ros::Time last_velocity_update_time_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( SimpleControls ),
        last_velocity_update_time_( ros::Time( 0 ) ),
        world_to_desired_pose_tf_( btTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( 0, 0, 0 ) ) )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        multi_sub_.addSubscriber( nh_rel, "cmd_vel", &SimpleControlsNode::cmdVelCB, this );
        multi_pub_.addPublishers<_MotorValsMsg>( nh_rel, { "motor_vals" } );

        initPolicies<_ResetPoseServiceServer>( "service_name_param", std::string( "/seabee3/reset_pose" ) );

        initPolicies<quickdev::policy::ALL>();

        pid_.applySettings(
            quickdev::make_shared( new _Pid2D::_Settings( "linear/z" ) ),
            quickdev::make_shared( new _Pid2D::_Settings( "angular/z" ) )
        );
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( cmdVelCB, _TwistMsg )
    {
        auto const now = ros::Time::now();

        auto lock = quickdev::make_unique_lock( last_velocity_mutex_ );

        last_velocity_msg_ptr_ = msg;

        if( ( now - last_velocity_update_time_ ).toSec() <= 2 )
        {
            auto world_to_desired_pose_tf = _TfTranceiverPolicy::tryLookupTransform( "/world", "/seabee3/desired_pose" );
            // update depth according to incoming linear z velocity
            world_to_desired_pose_tf.getOrigin().setZ( world_to_desired_pose_tf.getOrigin().getZ() + msg->linear.z * ( now - last_velocity_update_time_ ).toSec() );
            // update heading according to incoming angular z velocity
            world_to_desired_pose_tf.setRotation( world_to_desired_pose_tf.getRotation() * btQuaternion( msg->angular.z * ( now - last_velocity_update_time_ ).toSec(), 0, 0 ) );
            _TfTranceiverPolicy::publishTransform( btTransform( world_to_desired_pose_tf ), "/world", "/seabee3/desired_pose" );
        }

        last_velocity_update_time_ = now;
    }

    QUICKDEV_DECLARE_SERVICE_CALLBACK( resetPoseCB, _ResetPoseService )
    {
        auto lock = quickdev::make_unique_lock( world_to_current_pose_mutex_ );
        _TfTranceiverPolicy::publishTransform( world_to_current_pose_tf_, "/world", "/seabee3/desired_pose" );
        return true;
    }

    void normalizeMotorValsMsg( _MotorValsMsg & msg )
    {
        // do thruster pair scaling
        for( size_t i = 0; i < movement::ThrusterPairs::values.size(); ++i )
        {
            auto const & motor1_id = movement::ThrusterPairs::values[i][0];
            auto const & motor2_id = movement::ThrusterPairs::values[i][1];

            if( motor1_id < 0 || motor2_id < 0 ) continue;

            if( msg.mask[motor1_id] && msg.mask[motor2_id] )
            {
                auto & value1 = msg.motors[motor1_id];
                auto & value2 = msg.motors[motor2_id];
                auto const magnitude = std::max( abs( value1 ), abs( value2 ) );

//                printf( "normalizing axis [%zu] [%i, %i : %i]\n", i, value1, value2, magnitude );

                if( magnitude > 100 )
                {
                    double const scale = 100.0 / (double) magnitude;
                    value1 *= scale;
                    value2 *= scale;
                }

//                printf( "-> [%i, %i]\n", value1, value2 );
            }
        }
        // apply deadzones and floor values on a per-thruster basis
        /*
        for( size_t i = 0; i < msg.motors.size(); ++i )
        {
            if( !msg.mask[i] ) continue;

            auto & value = msg.motors[i];
            // if outputs are nonzero, normalize them to be within [-100, -15] [15, 100]
            if( abs( value ) > 5 ) value = quickdev::sign( value ) * 5 + value * (double)( 100 - 5 ) / 100.0;
            else value = 0;
        }
        */
    }

    template<int __Axis__, typename std::enable_if<(__Axis__ == movement::Axes::SPEED), int>::type = 0>
    void updateMotorValsMsgComponent( _MotorValsMsg & msg, int const & motor1_id, int const & motor2_id, btVector3 const & linear_vec, btVector3 const & angular_vec )
    {
        msg.motors[motor1_id] += -linear_vec.x();
        msg.motors[motor2_id] += -linear_vec.x();
    }

    template<int __Axis__, typename std::enable_if<(__Axis__ == movement::Axes::STRAFE), int>::type = 0>
    void updateMotorValsMsgComponent( _MotorValsMsg & msg, int const & motor1_id, int const & motor2_id, btVector3 const & linear_vec, btVector3 const & angular_vec )
    {
        msg.motors[motor1_id] += linear_vec.y();
        msg.motors[motor2_id] += -linear_vec.y();
    }

    template<int __Axis__, typename std::enable_if<(__Axis__ == movement::Axes::YAW), int>::type = 0>
    void updateMotorValsMsgComponent( _MotorValsMsg & msg, int const & motor1_id, int const & motor2_id, btVector3 const & linear_vec, btVector3 const & angular_vec )
    {
        msg.motors[motor1_id] += angular_vec.z();
        msg.motors[motor2_id] += -angular_vec.z();
    }

    template<int __Axis__, typename std::enable_if<(__Axis__ == movement::Axes::DEPTH), int>::type = 0>
    void updateMotorValsMsgComponent( _MotorValsMsg & msg, int const & motor1_id, int const & motor2_id, btVector3 const & linear_vec, btVector3 const & angular_vec )
    {
        msg.motors[motor1_id] += linear_vec.z();
        msg.motors[motor2_id] += -linear_vec.z();
    }

    // how to set motor values for any axis
    template<int __Axis__>
    void updateMotorValsMsg( _MotorValsMsg & msg, btVector3 const & linear_vec, btVector3 const & angular_vec )
    {
        auto const & motor1_id = movement::ThrusterPairs::values[__Axis__][0];
        auto const & motor2_id = movement::ThrusterPairs::values[__Axis__][1];

        updateMotorValsMsgComponent<__Axis__>( msg, motor1_id, motor2_id, linear_vec, angular_vec );

        msg.mask[motor1_id] = 1;
        msg.mask[motor2_id] = 1;
    }

    QUICKDEV_SPIN_ONCE()
    {
        _TwistMsg last_velocity_msg;
        {
            auto velocity_lock = quickdev::make_unique_lock( last_velocity_mutex_ );

            if( !last_velocity_msg_ptr_ ) return;

            last_velocity_msg = *last_velocity_msg_ptr_;
        }

        auto const world_to_imu_tf = _TfTranceiverPolicy::tryLookupTransform( "/world", "/seabee3/sensors/imu" );
        auto const world_to_depth_tf = _TfTranceiverPolicy::tryLookupTransform( "/world", "/seabee3/sensors/depth" );

        btTransform current_to_desired_tf;

        {
            auto world_to_desired_pose_lock = quickdev::make_unique_lock( world_to_desired_pose_mutex_ );
            world_to_desired_pose_tf_ = _TfTranceiverPolicy::tryLookupTransform( "/world", "/seabee3/desired_pose" );

            auto world_to_current_pose_lock = quickdev::make_unique_lock( world_to_current_pose_mutex_ );
            world_to_current_pose_tf_.setRotation( world_to_imu_tf.getRotation() );
            world_to_current_pose_tf_.setOrigin( world_to_depth_tf.getOrigin() );

            current_to_desired_tf = world_to_current_pose_tf_.inverse() * world_to_desired_pose_tf_;
        }

        btVector3 const linear_error_vec = unit::implicit_convert( current_to_desired_tf.getOrigin() );
        btVector3 const angular_error_vec = unit::implicit_convert( current_to_desired_tf.getRotation() );

        // motor value = linear velocity * 100; motor value : [ -100, 100 ]; linear velocity : [ -1.0, 1.0 ]
        auto linear_output_vec = ( 100.0 / 1.0 ) * unit::convert<btVector3>( last_velocity_msg.linear );
        // motor value = angular velocity * 100 / ( PI/2 ); motor value : [ -100, 100 ]; linear velocity : [ -PI/2, PI/2 ]
        auto angular_output_vec = ( 100.0 / M_PI_2 ) * unit::convert<btVector3>( last_velocity_msg.angular );

        // get control signal for depth
        linear_output_vec.setZ( -pid_.pids_[0].update( 0, linear_error_vec.z() ) );
        // get control signal for heading
        angular_output_vec.setZ( -pid_.pids_[1].update( 0, angular_error_vec.z() ) );

        _MotorValsMsg motor_vals_msg;

        updateMotorValsMsg<movement::Axes::SPEED>( motor_vals_msg, linear_output_vec, angular_output_vec );
        updateMotorValsMsg<movement::Axes::STRAFE>( motor_vals_msg, linear_output_vec, angular_output_vec );
        updateMotorValsMsg<movement::Axes::DEPTH>( motor_vals_msg, linear_output_vec, angular_output_vec );
        updateMotorValsMsg<movement::Axes::YAW>( motor_vals_msg, linear_output_vec, angular_output_vec );

        // ensure all motor values are properly normalized
        normalizeMotorValsMsg( motor_vals_msg );

        multi_pub_.publish( "motor_vals", motor_vals_msg );
    }
};

#endif // SEABEE3CONTROLS_SIMPLECONTROLSNODE_H_
