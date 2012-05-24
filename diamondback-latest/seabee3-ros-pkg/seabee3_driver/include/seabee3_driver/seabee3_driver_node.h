/***************************************************************************
 *  include/seabee3_driver/seabee3_driver_node.h
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

#ifndef SEABEE3DRIVER_SEABEE3DRIVERNODE_H_
#define SEABEE3DRIVER_SEABEE3DRIVERNODE_H_

#include <quickdev/node.h>

#include <quickdev/robot_driver_policy.h>
#include <quickdev/service_server_policy.h>
#include <quickdev/reconfigure_policy.h>

#include <seabee3_driver/MotorVals.h>
#include <seabee3_driver/Depth.h>
#include <seabee3_driver/KillSwitch.h>
#include <seabee3_driver/Pressure.h>

#include <seabee3_driver/FiringDeviceAction.h>

#include <seabee3_driver/FakeSeabeeConfig.h>

#include <seabee3_driver/bee_stem3_driver.h>

typedef seabee3_driver::MotorVals _MotorValsMsg;
typedef quickdev::RobotDriverPolicy<_MotorValsMsg> _RobotDriver;

typedef seabee3_driver::FiringDeviceAction _FiringDeviceActionService;
typedef quickdev::ServiceServerPolicy<_FiringDeviceActionService, 0> _Shooter1ServiceServer;
typedef quickdev::ServiceServerPolicy<_FiringDeviceActionService, 1> _Shooter2ServiceServer;
typedef quickdev::ServiceServerPolicy<_FiringDeviceActionService, 2> _Dropper1ServiceServer;
typedef quickdev::ServiceServerPolicy<_FiringDeviceActionService, 3> _Dropper2ServiceServer;

typedef seabee3_driver::FakeSeabeeConfig _FakeSeabeeCfg;
typedef quickdev::ReconfigurePolicy<_FakeSeabeeCfg> _FakeSeabeeLiveParams;

typedef seabee3_driver::Depth _DepthMsg;
typedef seabee3_driver::KillSwitch _KillSwitchMsg;
typedef seabee3_driver::Pressure _PressureMsg;

typedef quickdev::TfTranceiverPolicy _TfTranceiverPolicy;

using namespace seabee3_common;

QUICKDEV_DECLARE_NODE( Seabee3Driver, _RobotDriver, _Shooter1ServiceServer, _Shooter2ServiceServer, _Dropper1ServiceServer, _Dropper2ServiceServer, _FakeSeabeeLiveParams )

QUICKDEV_DECLARE_NODE_CLASS( Seabee3Driver )
{
    BeeStem3Driver bee_stem3_driver_;
    std::array<int, movement::NUM_MOTOR_CONTROLLERS> motor_dirs_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( Seabee3Driver )
    {
        motor_dirs_.fill( 1 );
    }

    QUICKDEV_SPIN_FIRST()
    {
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        _RobotDriver::registerCallback( quickdev::auto_bind( &Seabee3DriverNode::motorValsCB, this ) );
        _Shooter1ServiceServer::registerCallback( quickdev::auto_bind( &Seabee3DriverNode::shooter1CB, this ) );
        _Shooter2ServiceServer::registerCallback( quickdev::auto_bind( &Seabee3DriverNode::shooter2CB, this ) );
        _Dropper1ServiceServer::registerCallback( quickdev::auto_bind( &Seabee3DriverNode::dropper1CB, this ) );
        _Dropper2ServiceServer::registerCallback( quickdev::auto_bind( &Seabee3DriverNode::dropper2CB, this ) );
        _FakeSeabeeLiveParams::registerCallback( quickdev::auto_bind( &Seabee3DriverNode::reconfigureCB, this ) );

        initPolicies
        <
            _RobotDriver,
            _Shooter1ServiceServer,
            _Shooter2ServiceServer,
            _Dropper1ServiceServer,
            _Dropper2ServiceServer
        >
        (
            "enable_key_ids", true, // enable keys with IDs appended for all policies in this group
            "robot_name_param", std::string( "seabee3" ),
            "service_name_param0", std::string( "/seabee3/shooter1" ),
            "service_name_param1", std::string( "/seabee3/shooter2" ),
            "service_name_param2", std::string( "/seabee3/dropper1" ),
            "service_name_param3", std::string( "/seabee3/dropper2" )
        );

        // initialize any remaining policies
        initPolicies<quickdev::policy::ALL>();

        multi_pub_.addPublishers
        <
            _DepthMsg,
            _KillSwitchMsg,
            _PressureMsg,
            _PressureMsg
        >
        (
            nh_rel,
            {
                "/seabee3/depth",
                "/seabee3/kill_switch",
                "/seabee3/internal_pressure",
                "/seabee3/external_pressure"
            }
        );
    }

    inline double getDepthFromPressure( int const & observed_pressure ) const
    {
        return ( observed_pressure - config_.surface_pressure ) / config_.montalbos_per_meter;
    }

    QUICKDEV_SPIN_ONCE()
    {
        _PressureMsg intl_pressure_msg;
        _PressureMsg extl_pressure_msg;
        _DepthMsg depth_msg;
        _KillSwitchMsg kill_switch_msg;

        if( config_.simulate )
        {
            intl_pressure_msg.value = config_.internal_pressure;
            extl_pressure_msg.value = config_.external_pressure;
            kill_switch_msg.is_killed = config_.is_killed;
        }
        else
        {
            bee_stem3_driver_.readPressure( intl_pressure_msg.value, extl_pressure_msg.value );
            bee_stem3_driver_.readKillSwitch( kill_switch_msg.is_killed );
        }

        auto const depth = getDepthFromPressure( extl_pressure_msg.value );

        depth_msg.value = depth;

        multi_pub_.publish(
            "/seabee3/depth", depth_msg,
            "/seabee3/kill_switch", kill_switch_msg,
            "/seabee3/internal_pressure", intl_pressure_msg,
            "/seabee3/external_pressure", extl_pressure_msg
        );

        _TfTranceiverPolicy::publishTransform( btTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( 0, 0, -depth ) ), "/world", "/seabee3/depth" );
    }

    bool executeFiringDeviceAction( seabee3_driver::FiringDeviceAction::Request &req,
                                    seabee3_driver::FiringDeviceAction::Response &res,
                                    int const & device_id )
    {
        if( !bee_stem3_driver_.connected() ) return false;

        bool device_status = bee_stem3_driver_.getDeviceStatus( device_id );
        switch ( req.action )
        {
        case seabee3_driver::FiringDeviceAction::Request::CHECK_STATUS:
            break;
        case seabee3_driver::FiringDeviceAction::Request::RESET_STATUS:
            device_status = true;
            break;
        case seabee3_driver::FiringDeviceAction::Request::FIRE:
            bee_stem3_driver_.fireDevice( device_id );
            break;
        }
        res.is_loaded = device_status;
        return true;
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( motorValsCB, _MotorValsMsg )
    {
        if( config_.simulate && config_.is_killed ) return;

        auto const & motors = msg->motors;
        auto const & mask = msg->mask;
        for( size_t i = 0; i < motors.size(); ++i )
        {
            if( config_.override_kill_switch )
            {
                bee_stem3_driver_.setThruster( i, 0 );
                continue;
            }
            if( mask[i] )
            {
                auto motor_value = motor_dirs_[i] * motors[i];
                if( motor_value != 0 && abs( motor_value ) < config_.motor_speed_floor ) motor_value = 0;

                if( config_.simulate )
                {
                    PRINT_INFO( "Setting motor id %zu to %d", i, motor_value );
                }
                else
                {
                    bee_stem3_driver_.setThruster( i, motor_value );
                }
            }
        }
    }

    QUICKDEV_DECLARE_SERVICE_CALLBACK( shooter1CB, _FiringDeviceActionService )
    {
        if( config_.simulate ) PRINT_INFO( "Firing first torpedo!" );
        else return executeFiringDeviceAction( request, response, movement::FiringDeviceIDs::SHOOTER1 );

        return true;
    }

    QUICKDEV_DECLARE_SERVICE_CALLBACK( shooter2CB, _FiringDeviceActionService )
    {
        if( config_.simulate ) PRINT_INFO( "Firing second torpedo!" );
        else return executeFiringDeviceAction( request, response, movement::FiringDeviceIDs::SHOOTER2 );

        return true;
    }

    QUICKDEV_DECLARE_SERVICE_CALLBACK( dropper1CB, _FiringDeviceActionService )
    {
        if( config_.simulate ) PRINT_INFO( "Dropping first marker!" );
        else return executeFiringDeviceAction( request, response, movement::FiringDeviceIDs::DROPPER_STAGE1 );

        return true;
    }

    QUICKDEV_DECLARE_SERVICE_CALLBACK( dropper2CB, _FiringDeviceActionService )
    {
        if( config_.simulate ) PRINT_INFO( "Dropping second marker!" );
        else return executeFiringDeviceAction( request, response, movement::FiringDeviceIDs::DROPPER_STAGE2 );

        return true;
    }

    QUICKDEV_DECLARE_RECONFIGURE_CALLBACK( reconfigureCB, _FakeSeabeeCfg )
    {
        if( !config.simulate )
        {
            bee_stem3_driver_.connect( config.port );
            bee_stem3_driver_.shooter1_params_.trigger_time_ =  config.shooter1_trigger_time;
            bee_stem3_driver_.shooter1_params_.trigger_value_ = config.shooter1_trigger_value;
            bee_stem3_driver_.shooter2_params_.trigger_time_ =  config.shooter2_trigger_time;
            bee_stem3_driver_.shooter2_params_.trigger_value_ = config.shooter2_trigger_value;
            bee_stem3_driver_.dropper1_params_.trigger_time_ =  config.dropper1_trigger_time;
            bee_stem3_driver_.dropper1_params_.trigger_value_ = config.dropper1_trigger_value;
            bee_stem3_driver_.dropper2_params_.trigger_time_ =  config.dropper2_trigger_time;
            bee_stem3_driver_.dropper2_params_.trigger_value_ = config.dropper2_trigger_value;
        }

        motor_dirs_[movement::MotorControllerIDs::FWD_RIGHT_THRUSTER] =    config.fwd_right_thruster_dir;
        motor_dirs_[movement::MotorControllerIDs::FWD_LEFT_THRUSTER] =     config.fwd_left_thruster_dir;
        motor_dirs_[movement::MotorControllerIDs::DEPTH_RIGHT_THRUSTER] =  config.depth_right_thruster_dir;
        motor_dirs_[movement::MotorControllerIDs::DEPTH_LEFT_THRUSTER] =   config.depth_left_thruster_dir;
        motor_dirs_[movement::MotorControllerIDs::STRAFE_FRONT_THRUSTER] = config.strafe_front_thruster_dir;
        motor_dirs_[movement::MotorControllerIDs::STRAFE_BACK_THRUSTER] =  config.strafe_back_thruster_dir;
    }
};

#endif // SEABEE3DRIVER_SEABEE3DRIVERNODE_H_
