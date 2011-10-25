/***************************************************************************
 *  test/service_client_policy.cpp
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

#include <base_libs/node.h>
#include <base_libs/service_client_policy.h>
#include <base_libs/TestService1.h>
#include <base_libs/TestService2.h>

typedef base_libs::TestService1 _TestService1;
typedef base_libs::ServiceClientPolicy<_TestService1> _ServiceClientPolicy1;
typedef base_libs::TestService2 _TestService2;
typedef base_libs::ServiceClientPolicy<_TestService2> _ServiceClientPolicy2;

BASE_LIBS_DECLARE_NODE( TestServiceClientPolicy, _ServiceClientPolicy1, _ServiceClientPolicy2 )

BASE_LIBS_DECLARE_NODE_CLASS( TestServiceClientPolicy )
{
private:
	bool mode_;
	
	BASE_LIBS_DECLARE_NODE_CONSTRUCTOR( TestServiceClientPolicy ),
		mode_( false )
	{}

public:
	void spinFirst()
	{
		//initAll();
		
		_ServiceClientPolicy1::init( "service_name_param", std::string( "service1_name" ) );
		_ServiceClientPolicy2::init( "service_name_param", std::string( "service2_name" ) );
	}
	
	void spinOnce()
	{
		PRINT_INFO( "Spinning..." );
		
		if( mode_ )
		{
			_TestService1 service;
			_ServiceClientPolicy1::callService( service );
		}
		else
		{
			_TestService2 service;
			_ServiceClientPolicy2::callService( service, ros::Duration( 2.0 ), 2 );
		}
		
		mode_ = !mode_;
	}
};

BASE_LIBS_INST_NODE( TestServiceClientPolicyNode, "test_service_client_policy_node" )
