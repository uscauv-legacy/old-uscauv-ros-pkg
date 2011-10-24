/***************************************************************************
 *  test/service_server_policy.cpp
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
#include <base_libs/service_server_policy.h>
#include <base_libs_prototype/TestService1.h>
#include <base_libs_prototype/TestService2.h>

typedef base_libs_prototype::TestService1 _TestService1;
typedef base_libs::ServiceServerPolicy<_TestService1> _TestService1ServerPolicy;
typedef base_libs_prototype::TestService2 _TestService2;
typedef base_libs::ServiceServerPolicy<_TestService2> _TestService2ServerPolicy;

BASE_LIBS_DECLARE_NODE( TestServiceServerPolicy, _TestService1ServerPolicy, _TestService2ServerPolicy )

BASE_LIBS_DECLARE_NODE_CLASS( TestServiceServerPolicy )
{
	BASE_LIBS_DECLARE_NODE_CONSTRUCTOR( TestServiceServerPolicy ){}

public:
	void spinFirst()
	{
		//initAll();
		
		_TestService1ServerPolicy::init( "service_name_param", std::string( "service1_name" ) );
		_TestService1ServerPolicy::registerCallback( base_libs::auto_bind( &TestServiceServerPolicyNode::service1CB, this ) );
		
		_TestService2ServerPolicy::init( "service_name_param", std::string( "service2_name" ) );
		_TestService2ServerPolicy::registerCallback( base_libs::auto_bind( &TestServiceServerPolicyNode::service2CB, this ) );
	}
	
	bool service1CB( _TestService1::Request & request, _TestService1::Response & response )
	{
		PRINT_INFO( "Got service1CB" );
		return true;
	}
	
	bool service2CB( _TestService2::Request & request, _TestService2::Response & response )
	{
		PRINT_INFO( "Got service2CB" );
		return true;
	}
};

BASE_LIBS_INST_NODE( TestServiceServerPolicyNode, "test_service_server_policy_node" )
