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

#include <base_libs/node.h>
#include <base_libs/robot_driver_policy.h>
#include <base_libs/service_server_policy.h>
#include <seabee3_driver/MotorVals.h>
#include <seabee3_driver/FiringDeviceAction.h>

typedef seabee3_driver::MotorVals _MotorValsMsg;
typedef base_libs::RobotDriverPolicy<_MotorValsMsg> _RobotDriverPolicy;

typedef seabee3_driver::FiringDeviceAction _FiringDeviceActionService;
typedef base_libs::ServiceServerPolicy<_FiringDeviceActionService, 0> _Shooter1ServiceServerPolicy;
typedef base_libs::ServiceServerPolicy<_FiringDeviceActionService, 1> _Shooter2ServiceServerPolicy;
typedef base_libs::ServiceServerPolicy<_FiringDeviceActionService, 2> _Dropper1ServiceServerPolicy;
typedef base_libs::ServiceServerPolicy<_FiringDeviceActionService, 3> _Dropper2ServiceServerPolicy;

BASE_LIBS_DECLARE_NODE( Seabee3Driver, _RobotDriverPolicy, _Shooter1ServiceServerPolicy, _Shooter2ServiceServerPolicy, _Dropper1ServiceServerPolicy, _Dropper2ServiceServerPolicy )

BASE_LIBS_DECLARE_NODE_CLASS( Seabee3Driver )
{
	BASE_LIBS_DECLARE_NODE_CONSTRUCTOR( Seabee3Driver )
	{
		//
	}
	
	void spinFirst()
	{
		auto & nh_rel = base_libs::RunablePolicy::getNodeHandle();
		
		nh_rel.setParam( "robot_name", "seabee3" );
		_RobotDriverPolicy::init();
		_RobotDriverPolicy::registerCallback( base_libs::auto_bind( &Seabee3DriverNode::motorValsCB, this ) );
		
		nh_rel.setParam( "shooter1_service_name", "/seabee3/shooter1" );
		_Shooter1ServiceServerPolicy::init( "service_name_param", std::string( "shooter1_service_name" ) );
		_Shooter1ServiceServerPolicy::registerCallback( base_libs::auto_bind( &Seabee3DriverNode::shooter1CB, this ) );
		
		nh_rel.setParam( "shooter2_service_name", "/seabee3/shooter2" );
		_Shooter2ServiceServerPolicy::init( "service_name_param", std::string( "shooter2_service_name" ) );
		_Shooter2ServiceServerPolicy::registerCallback( base_libs::auto_bind( &Seabee3DriverNode::shooter2CB, this ) );
		
		nh_rel.setParam( "dropper1_service_name", "/seabee3/dropper1" );
		_Dropper1ServiceServerPolicy::init( "service_name_param", std::string( "dropper1_service_name" ) );
		_Dropper1ServiceServerPolicy::registerCallback( base_libs::auto_bind( &Seabee3DriverNode::dropper1CB, this ) );
		
		nh_rel.setParam( "dropper2_service_name", "/seabee3/dropper2" );
		_Dropper2ServiceServerPolicy::init( "service_name_param", std::string( "dropper2_service_name" ) );
		_Dropper2ServiceServerPolicy::registerCallback( base_libs::auto_bind( &Seabee3DriverNode::dropper2CB, this ) );
	}
	
	BASE_LIBS_DECLARE_MESSAGE_CALLBACK( motorValsCB, _MotorValsMsg )
	{
		PRINT_INFO( "Setting motor vals..." );
	}
	
	BASE_LIBS_DECLARE_SERVICE_CALLBACK( shooter1CB, _FiringDeviceActionService )
	{
		PRINT_INFO( "Firing first torpedo!" );
		return true;
	}
	
	BASE_LIBS_DECLARE_SERVICE_CALLBACK( shooter2CB, _FiringDeviceActionService )
	{
		PRINT_INFO( "Firing second torpedo!" );
		return true;
	}
	
	BASE_LIBS_DECLARE_SERVICE_CALLBACK( dropper1CB, _FiringDeviceActionService )
	{
		PRINT_INFO( "Dropping first marker!" );
		return true;
	}
	
	BASE_LIBS_DECLARE_SERVICE_CALLBACK( dropper2CB, _FiringDeviceActionService )
	{
		PRINT_INFO( "Dropping second marker!" );
		return true;
	}
};

#endif // SEABEE3_DRIVER_SEABEE3_DRIVER_SEABEE3_DRIVER_H_
