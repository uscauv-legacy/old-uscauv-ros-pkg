/***************************************************************************
 *  include/seabee3_controls/seabee3_controls_node.h
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

#ifndef SEABEE3CONTROLS_SEABEE3CONTROLSNODE_H_
#define SEABEE3CONTROLS_SEABEE3CONTROLSNODE_H_

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
#include <quickdev/math.h>
#include <seabee3_common/movement.h>
#include <quickdev/geometry_message_conversions.h>

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

QUICKDEV_DECLARE_NODE( Seabee3Controls, _ResetPoseServiceServer, _Seabee3ControlsLiveParams, _TfTranceiverPolicy )

QUICKDEV_DECLARE_NODE_CLASS( Seabee3Controls )
{
public:
    typedef quickdev::ReconfigurablePID<6> _Pid6D;

protected:
    _Pid6D pid_;

    ros::MultiPublisher<> multi_pub_;
    ros::MultiSubscriber<> multi_sub_;

    btTransform transform_to_target_;
    std::mutex transform_to_target_mutex_;

    ros::Time last_velocity_update_time_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( Seabee3Controls ),
        last_velocity_update_time_( ros::Time( 0 ) )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        _ResetPoseServiceServer::registerCallback( quickdev::auto_bind( &Seabee3ControlsNode::resetPoseCB, this ) );
//        _Seabee3ControlsLiveParams::registerCallback( quickdev::auto_bind( &Seabee3ControlsNode::reconfigureCB, this ) );

        multi_sub_.addSubscriber( nh_rel, "cmd_vel", &Seabee3ControlsNode::cmdVelCB, this );

        multi_pub_.addPublishers<_MotorValsMsg>( nh_rel, { "motor_vals" } );

        initPolicies<_ResetPoseServiceServer>( "service_name_param", std::string( "/seabee3/reset_pose" ) );

        initPolicies<quickdev::policy::ALL>();

        pid_.applySettings(
            quickdev::make_shared( new _Pid6D::_Settings( "linear/x" ) ),
            quickdev::make_shared( new _Pid6D::_Settings( "linear/y" ) ),
            quickdev::make_shared( new _Pid6D::_Settings( "linear/z" ) ),
            quickdev::make_shared( new _Pid6D::_Settings( "angular/x" ) ),
            quickdev::make_shared( new _Pid6D::_Settings( "angular/y" ) ),
            quickdev::make_shared( new _Pid6D::_Settings( "angular/z" ) )
        );

        initPolicies<quickdev::policy::ALL>();

//        btVector3 const rotation = unit::make_unit( btQuaternion( 0.1, 0.2, 0.3 ) );
//        printf( "%f %f %f\n", rotation.x(), rotation.y(), rotation.z() );
    }

    template<int __Axis__, typename std::enable_if<(__Axis__ == movement::Axes::SPEED), int>::type = 0>
    void updateMotorValsMsgComponent( _MotorValsMsg & msg, int const & motor1_id, int const & motor2_id, btVector3 const & linear_vec, btVector3 const & angular_vec )
    {
        msg.motors[motor1_id] += linear_vec.x();
        msg.motors[motor2_id] += linear_vec.x();
    }

    template<int __Axis__, typename std::enable_if<(__Axis__ == movement::Axes::STRAFE), int>::type = 0>
    void updateMotorValsMsgComponent( _MotorValsMsg & msg, int const & motor1_id, int const & motor2_id, btVector3 const & linear_vec, btVector3 const & angular_vec )
    {
        msg.motors[motor1_id] += -linear_vec.y();
        msg.motors[motor2_id] += linear_vec.y();
    }

    template<int __Axis__, typename std::enable_if<(__Axis__ == movement::Axes::DEPTH), int>::type = 0>
    void updateMotorValsMsgComponent( _MotorValsMsg & msg, int const & motor1_id, int const & motor2_id, btVector3 const & linear_vec, btVector3 const & angular_vec )
    {
        msg.motors[motor1_id] += linear_vec.z();
        msg.motors[motor2_id] += linear_vec.z();
    }

    template<int __Axis__, typename std::enable_if<(__Axis__ == movement::Axes::YAW), int>::type = 0>
    void updateMotorValsMsgComponent( _MotorValsMsg & msg, int const & motor1_id, int const & motor2_id, btVector3 const & linear_vec, btVector3 const & angular_vec )
    {
        msg.motors[motor1_id] += angular_vec.z();
        msg.motors[motor2_id] += -angular_vec.z();
    }

    template<int __Axis__, typename std::enable_if<(__Axis__ == movement::Axes::PITCH), int>::type = 0>
    void updateMotorValsMsgComponent( _MotorValsMsg & msg, int const & motor1_id, int const & motor2_id, btVector3 const & linear_vec, btVector3 const & angular_vec )
    {
        msg.motors[motor1_id] += -angular_vec.z();
        msg.motors[motor2_id] += angular_vec.z();
    }

    template<int __Axis__, typename std::enable_if<(__Axis__ == movement::Axes::ROLL), int>::type = 0>
    void updateMotorValsMsgComponent( _MotorValsMsg & msg, int const & motor1_id, int const & motor2_id, btVector3 const & linear_vec, btVector3 const & angular_vec )
    {
        msg.motors[motor1_id] += angular_vec.z();
        msg.motors[motor2_id] += angular_vec.z();
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
        for( size_t i = 0; i < msg.motors.size(); ++i )
        {
            if( !msg.mask[i] ) continue;

            auto & value = msg.motors[i];
            // if outputs are nonzero, normalize them to be within [-100, -15] [15, 100]
            if( abs( value ) > config_.motor_speed_deadzone ) value = quickdev::sign( value ) * config_.motor_speed_floor + value * (double)( 100 - config_.motor_speed_floor ) / 100.0;
            else value = 0;
        }
    }

    QUICKDEV_SPIN_ONCE()
    {
        _MotorValsMsg motor_vals_msg;
        try
        {
            btVector3 linear_error_vec;
            btVector3 angular_error_vec;
            {
                auto transform_to_target_lock = quickdev::make_unique_lock( transform_to_target_mutex_ );
                transform_to_target_ = _TfTranceiverPolicy::tryLookupTransform( "/seabee3/current_pose", "/seabee3/desired_pose" );

                // calculate pose error
                linear_error_vec = unit::make_unit( transform_to_target_.getOrigin() );
                angular_error_vec = unit::make_unit( transform_to_target_.getRotation() );
            }
/*
            printf( "error [%f %f %f] [%f %f %f]\n",
                linear_error_vec.x(),
                linear_error_vec.y(),
                linear_error_vec.z(),
                angular_error_vec.x(),
                angular_error_vec.y(),
                angular_error_vec.z()
            );
*/
            btVector3 linear_output_vec;
            btVector3 angular_output_vec;

            // update PIDs
            linear_output_vec.setX( pid_.linear_.x_.update( 0, linear_error_vec.x() ) );
            linear_output_vec.setY( pid_.linear_.y_.update( 0, linear_error_vec.y() ) );
            linear_output_vec.setZ( pid_.linear_.z_.update( 0, linear_error_vec.z() ) );

            angular_output_vec.setX( pid_.angular_.x_.update( 0, angular_error_vec.x() ) );
            angular_output_vec.setY( pid_.angular_.y_.update( 0, angular_error_vec.y() ) );
            angular_output_vec.setZ( pid_.angular_.z_.update( 0, angular_error_vec.z() ) );
/*
            printf( "pid [%f %f %f] [%f %f %f]\n",
                linear_output_vec.x(),
                linear_output_vec.y(),
                linear_output_vec.z(),
                angular_output_vec.x(),
                angular_output_vec.y(),
                angular_output_vec.z()
            );
*/
            // convert axis output values into motor values
            updateMotorValsMsg<movement::Axes::SPEED>( motor_vals_msg, linear_output_vec, angular_output_vec );
            updateMotorValsMsg<movement::Axes::STRAFE>( motor_vals_msg, linear_output_vec, angular_output_vec );
            updateMotorValsMsg<movement::Axes::DEPTH>( motor_vals_msg, linear_output_vec, angular_output_vec );
            updateMotorValsMsg<movement::Axes::YAW>( motor_vals_msg, linear_output_vec, angular_output_vec );
            updateMotorValsMsg<movement::Axes::PITCH>( motor_vals_msg, linear_output_vec, angular_output_vec );
            updateMotorValsMsg<movement::Axes::ROLL>( motor_vals_msg, linear_output_vec, angular_output_vec );

            // ensure all motor values are properly normalized
            normalizeMotorValsMsg( motor_vals_msg );

        }
        catch( std::exception const & ex )
        {
            PRINT_ERROR( "%s", ex.what() );
        }

        multi_pub_.publish( "motor_vals", motor_vals_msg );
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( cmdVelCB, _TwistMsg )
    {
        auto const now = ros::Time::now();

        auto lock = quickdev::make_unique_lock( transform_to_target_mutex_ );

        _TwistMsg old_transform_to_target_twist = unit::make_unit( transform_to_target_ );
        _TwistMsg new_transform_to_target_twist = old_transform_to_target_twist;

        if( msg->linear.x ) new_transform_to_target_twist.linear.x = msg->linear.x;
        if( msg->linear.y ) new_transform_to_target_twist.linear.y = msg->linear.y;

        if( msg->angular.x ) new_transform_to_target_twist.angular.x = msg->angular.x;
        if( msg->angular.y ) new_transform_to_target_twist.angular.y = msg->angular.y;

        if( last_velocity_update_time_ != ros::Time( 0 ) && ( now - last_velocity_update_time_ ).toSec() <= 2.0 )
        {
            if( msg->linear.z ) new_transform_to_target_twist.linear.z = msg->linear.z * ( now - last_velocity_update_time_ ).toSec();
            if( msg->angular.z ) new_transform_to_target_twist.angular.z = msg->angular.z  * ( now - last_velocity_update_time_ ).toSec();
        }

        transform_to_target_ = unit::make_unit( new_transform_to_target_twist );
        _TfTranceiverPolicy::publishTransform( transform_to_target_, "/seabee/current_pose", "/seabee/desired_pose" );

        last_velocity_update_time_ = now;
    }

    QUICKDEV_DECLARE_SERVICE_CALLBACK( resetPoseCB, _ResetPoseService )
    {
        auto lock = quickdev::make_unique_lock( transform_to_target_mutex_ );
        transform_to_target_ = btTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( 0, 0, 0 ) );
        _TfTranceiverPolicy::publishTransform( transform_to_target_, "/seabee/current_pose", "/seabee/desired_pose" );
        return true;
    }
/*
    QUICKDEV_DECLARE_RECONFIGURE_CALLBACK( reconfigureCB, _Seabee3ControlsCfg )
    {
        //
    }
*/
};

#endif // SEABEE3CONTROLS_SEABEE3CONTROLSNODE_H_
