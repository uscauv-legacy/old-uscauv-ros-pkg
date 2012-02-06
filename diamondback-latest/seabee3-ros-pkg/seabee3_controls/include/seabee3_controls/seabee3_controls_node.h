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
#include <quickdev/robot_controller_policy.h>
#include <quickdev/controllers/reconfigurable_pid.h>
#include <seabee3_common/movement.h>
#include <seabee3_driver/MotorVals.h>

typedef seabee3_driver::MotorVals _MotorValsMsg;
typedef quickdev::RobotControllerPolicy<_MotorValsMsg> _RobotController;

using namespace seabee3_common;

QUICKDEV_DECLARE_NODE( Seabee3Controls, _RobotController )

QUICKDEV_DECLARE_NODE_CLASS( Seabee3Controls )
{
    typedef quickdev::ReconfigurablePID<6> _Pid6D;

    _Pid6D pid_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( Seabee3Controls )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        initPolicies<_RobotController>( "robot_name_param", std::string( "seabee3" ) );

        pid_.applySettings(
            quickdev::make_shared( new _Pid6D::_Settings( "linear/x" ) ),
            quickdev::make_shared( new _Pid6D::_Settings( "linear/y" ) ),
            quickdev::make_shared( new _Pid6D::_Settings( "linear/z" ) ),
            quickdev::make_shared( new _Pid6D::_Settings( "angular/x" ) ),
            quickdev::make_shared( new _Pid6D::_Settings( "angular/y" ) ),
            quickdev::make_shared( new _Pid6D::_Settings( "angular/z" ) )
        );

        initPolicies<quickdev::policy::ALL>();

        btVector3 const rotation = unit::make_unit( btQuaternion( 0.1, 0.2, 0.3 ) );
        printf( "%f %f %f\n", rotation.x(), rotation.y(), rotation.z() );
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

    template<int __Axis__, typename std::enable_if<(__Axis__ == movement::Axes::YAW), int>::type = 0>
    void updateMotorValsMsgComponent( _MotorValsMsg & msg, int const & motor1_id, int const & motor2_id, btVector3 const & linear_vec, btVector3 const & angular_vec )
    {
        msg.motors[motor1_id] += -angular_vec.z();
        msg.motors[motor2_id] += -angular_vec.z();
    }

    template<int __Axis__, typename std::enable_if<(__Axis__ == movement::Axes::DEPTH), int>::type = 0>
    void updateMotorValsMsgComponent( _MotorValsMsg & msg, int const & motor1_id, int const & motor2_id, btVector3 const & linear_vec, btVector3 const & angular_vec )
    {
        msg.motors[motor1_id] += linear_vec.z();
        msg.motors[motor2_id] += linear_vec.z();
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

                printf( "normalizing axis [%zu] [%i, %i : %i]\n", i, value1, value2, magnitude );

                if( magnitude > 100 )
                {
                    double const scale = 100.0 / (double) magnitude;
                    value1 *= scale;
                    value2 *= scale;
                }

                printf( "-> [%i, %i]\n", value1, value2 );
            }
        }
    }

    QUICKDEV_SPIN_ONCE()
    {
        auto const transform_to_target = _RobotController::getTransformToTarget();

        _MotorValsMsg motor_vals_msg;

        // calculate pose error
        btVector3 const linear_error_vec = unit::make_unit( transform_to_target.getOrigin() );
        btVector3 const angular_error_vec = unit::make_unit( transform_to_target.getRotation() );

        printf( "error [%f %f %f] [%f %f %f]\n",
            linear_error_vec.x(),
            linear_error_vec.y(),
            linear_error_vec.z(),
            angular_error_vec.x(),
            angular_error_vec.y(),
            angular_error_vec.z()
        );

        btVector3 linear_output_vec;
        btVector3 angular_output_vec;

        // update PIDs
        linear_output_vec.setX( pid_.linear_.x_.update( 0, linear_error_vec.x() ) );
        linear_output_vec.setY( pid_.linear_.y_.update( 0, linear_error_vec.y() ) );
        linear_output_vec.setZ( pid_.linear_.z_.update( 0, linear_error_vec.z() ) );

        //angular_output_vec.setX( pid_.angular_.x_.update( 0, angular_error_vec.x() ) );
        //angular_output_vec.setY( pid_.angular_.y_.update( 0, angular_error_vec.y() ) );
        angular_output_vec.setZ( pid_.angular_.z_.update( 0, angular_error_vec.z() ) );

        printf( "pid [%f %f %f] [%f %f %f]\n",
            linear_output_vec.x(),
            linear_output_vec.y(),
            linear_output_vec.z(),
            angular_output_vec.x(),
            angular_output_vec.y(),
            angular_output_vec.z()
        );

        // convert axis output values into motor values
        updateMotorValsMsg<movement::Axes::SPEED>( motor_vals_msg, linear_output_vec, angular_output_vec );
        updateMotorValsMsg<movement::Axes::STRAFE>( motor_vals_msg, linear_output_vec, angular_output_vec );
        updateMotorValsMsg<movement::Axes::DEPTH>( motor_vals_msg, linear_output_vec, angular_output_vec );
        updateMotorValsMsg<movement::Axes::YAW>( motor_vals_msg, linear_output_vec, angular_output_vec );

        // ensure all motor values are properly normalized
        normalizeMotorValsMsg( motor_vals_msg );

        _RobotController::update( motor_vals_msg );
    }
};

#endif // SEABEE3CONTROLS_SEABEE3CONTROLSNODE_H_
