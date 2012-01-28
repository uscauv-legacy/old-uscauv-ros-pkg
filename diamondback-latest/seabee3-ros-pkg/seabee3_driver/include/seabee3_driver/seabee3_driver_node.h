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

QUICKDEV_DECLARE_NODE( Seabee3Driver, _RobotDriver, _Shooter1ServiceServer, _Shooter2ServiceServer, _Dropper1ServiceServer, _Dropper2ServiceServer, _FakeSeabeeLiveParams )

QUICKDEV_DECLARE_NODE_CLASS( Seabee3Driver )
{
    boost::shared_ptr<BeeStem3Driver> bee_stem3_driver_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( Seabee3Driver )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        initPolicies<
            _RobotDriver,
            _Shooter1ServiceServer,
            _Shooter2ServiceServer,
            _Dropper1ServiceServer,
            _Dropper2ServiceServer
        >(
            "enable_key_ids", true,
            "robot_name_param", std::string( "seabee3" ),
            "service_name_param0", std::string( "/seabee3/shooter1" ),
            "service_name_param1", std::string( "/seabee3/shooter2" ),
            "service_name_param2", std::string( "/seabee3/dropper1" ),
            "service_name_param3", std::string( "/seabee3/dropper2" )
        );

        // initialize any remaining policies
        initPolicies<quickdev::policy::ALL>();

        _RobotDriver::registerCallback( quickdev::auto_bind( &Seabee3DriverNode::motorValsCB, this ) );
        _Shooter1ServiceServer::registerCallback( quickdev::auto_bind( &Seabee3DriverNode::shooter1CB, this ) );
        _Shooter2ServiceServer::registerCallback( quickdev::auto_bind( &Seabee3DriverNode::shooter2CB, this ) );
        _Dropper1ServiceServer::registerCallback( quickdev::auto_bind( &Seabee3DriverNode::dropper1CB, this ) );
        _Dropper2ServiceServer::registerCallback( quickdev::auto_bind( &Seabee3DriverNode::dropper2CB, this ) );

        multi_pub_.addPublishers<
            _DepthMsg,
            _KillSwitchMsg,
            _PressureMsg,
            _PressureMsg
        > ( nh_rel, {
            "/seabee3/depth",
            "/seabee3/kill_switch",
            "/seabee3/internal_pressure",
            "/seabee3/external_pressure"
        } );

    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( motorValsCB, _MotorValsMsg )
    {
        PRINT_INFO( "Setting motor vals..." );
    }

    QUICKDEV_DECLARE_SERVICE_CALLBACK( shooter1CB, _FiringDeviceActionService )
    {
        PRINT_INFO( "Firing first torpedo!" );
        return true;
    }

    QUICKDEV_DECLARE_SERVICE_CALLBACK( shooter2CB, _FiringDeviceActionService )
    {
        PRINT_INFO( "Firing second torpedo!" );
        return true;
    }

    QUICKDEV_DECLARE_SERVICE_CALLBACK( dropper1CB, _FiringDeviceActionService )
    {
        PRINT_INFO( "Dropping first marker!" );
        return true;
    }

    QUICKDEV_DECLARE_SERVICE_CALLBACK( dropper2CB, _FiringDeviceActionService )
    {
        PRINT_INFO( "Dropping second marker!" );
        return true;
    }
};

#endif // SEABEE3DRIVER_SEABEE3DRIVERNODE_H_
