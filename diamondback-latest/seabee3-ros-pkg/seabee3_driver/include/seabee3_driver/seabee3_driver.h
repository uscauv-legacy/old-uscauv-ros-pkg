/***************************************************************************
 *  include/seabee3_driver/seabee3_driver.h
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

#ifndef SEABEE3_DRIVER_SEABEE3_DRIVER_SEABEE3_DRIVER_H_
#define SEABEE3_DRIVER_SEABEE3_DRIVER_SEABEE3_DRIVER_H_

#include <quickdev/node.h>
#include <quickdev/robot_driver_policy.h>
#include <quickdev/service_server_policy.h>
#include <seabee3_driver/MotorVals.h>
#include <seabee3_driver/FiringDeviceAction.h>

typedef seabee3_driver::MotorVals _MotorValsMsg;
typedef quickdev::RobotDriverPolicy<_MotorValsMsg> _RobotDriver;

typedef seabee3_driver::FiringDeviceAction _FiringDeviceActionService;
typedef quickdev::ServiceServerPolicy<_FiringDeviceActionService, 0> _Shooter1ServiceServer;
typedef quickdev::ServiceServerPolicy<_FiringDeviceActionService, 1> _Shooter2ServiceServer;
typedef quickdev::ServiceServerPolicy<_FiringDeviceActionService, 2> _Dropper1ServiceServer;
typedef quickdev::ServiceServerPolicy<_FiringDeviceActionService, 3> _Dropper2ServiceServer;

QUICKDEV_DECLARE_NODE( Seabee3Driver, _RobotDriver, _Shooter1ServiceServer, _Shooter2ServiceServer, _Dropper1ServiceServer, _Dropper2ServiceServer )

QUICKDEV_DECLARE_NODE_CLASS( Seabee3Driver )
{
	QUICKDEV_DECLARE_NODE_CONSTRUCTOR( Seabee3Driver )
	{
		//
	}

	QUICKDEV_SPIN_FIRST()
	{
		auto & nh_rel = quickdev::RunablePolicy::getNodeHandle();

		nh_rel.setParam( "robot_name", "seabee3" );
		_RobotDriver::init();
		_RobotDriver::registerCallback( quickdev::auto_bind( &Seabee3DriverNode::motorValsCB, this ) );

		nh_rel.setParam( "shooter1_service_name", "/seabee3/shooter1" );
		_Shooter1ServiceServer::init( "service_name_param", std::string( "shooter1_service_name" ) );
		_Shooter1ServiceServer::registerCallback( quickdev::auto_bind( &Seabee3DriverNode::shooter1CB, this ) );

		nh_rel.setParam( "shooter2_service_name", "/seabee3/shooter2" );
		_Shooter2ServiceServer::init( "service_name_param", std::string( "shooter2_service_name" ) );
		_Shooter2ServiceServer::registerCallback( quickdev::auto_bind( &Seabee3DriverNode::shooter2CB, this ) );

		nh_rel.setParam( "dropper1_service_name", "/seabee3/dropper1" );
		_Dropper1ServiceServer::init( "service_name_param", std::string( "dropper1_service_name" ) );
		_Dropper1ServiceServer::registerCallback( quickdev::auto_bind( &Seabee3DriverNode::dropper1CB, this ) );

		nh_rel.setParam( "dropper2_service_name", "/seabee3/dropper2" );
		_Dropper2ServiceServer::init( "service_name_param", std::string( "dropper2_service_name" ) );
		_Dropper2ServiceServer::registerCallback( quickdev::auto_bind( &Seabee3DriverNode::dropper2CB, this ) );
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

#endif // SEABEE3_DRIVER_SEABEE3_DRIVER_SEABEE3_DRIVER_H_
