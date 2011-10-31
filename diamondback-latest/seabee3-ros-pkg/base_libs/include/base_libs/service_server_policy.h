/***************************************************************************
 *  include/base_libs/service_server_policy.h
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

#ifndef BASE_LIBS_BASE_LIBS_SERVICE_SERVER_POLICY_H_
#define BASE_LIBS_BASE_LIBS_SERVICE_SERVER_POLICY_H_

#include <base_libs/node_handle_policy.h>
#include <base_libs/auto_bind.h>
#include <ros/service_server.h>

namespace base_libs
{

BASE_LIBS_DECLARE_POLICY( ServiceServer, NodeHandlePolicy )

template<class __Service, unsigned int __Id__ = 0>
BASE_LIBS_DECLARE_POLICY_CLASS( ServiceServer )
{
	BASE_LIBS_MAKE_POLICY_NAME( ServiceServer )
	
protected:
	typedef typename __Service::Request _ServiceRequest;
	typedef typename __Service::Response _ServiceResponse;
	typedef ServiceServerPolicy<__Service, __Id__> _ServiceServerPolicy;
	typedef std::function<bool( _ServiceRequest &, _ServiceResponse & )> _CallbackType;
	
	ros::ServiceServer server_;
	_CallbackType external_callback_;
	
	BASE_LIBS_DECLARE_POLICY_CONSTRUCTOR( ServiceServer ),
		initialized_( false )
	{
		printPolicyActionStart( "create", this );
		printPolicyActionDone( "create", this );
	}
	
	BASE_LIBS_ENABLE_INIT
	{
		printPolicyActionStart( "initialize", this );
		
		auto & nh_rel = NodeHandlePolicy::getNodeHandle();
		
		const std::string service_name_param( getMetaParamDef<std::string>( "service_name_param", "service_name", args... ) );
		const std::string service_name( ros::ParamReader<std::string, 1>::readParam( nh_rel, service_name_param, "service" ) );
		
		ros::NodeHandle service_nh( nh_rel, service_name );
		PRINT_INFO( "Creating service server [%s] on topic [%s]", ros::service_traits::DataType<__Service>::value(), service_nh.getNamespace().c_str() );
		
		server_ = nh_rel.advertiseService( service_name, &_ServiceServerPolicy::serviceCB, this );
		
		BASE_LIBS_SET_INITIALIZED;
		
		printPolicyActionDone( "initialize", this );
	}

	void registerCallback( const _CallbackType & external_callback )
	{
		BASE_LIBS_CHECK_INITIALIZED;
		
		external_callback_ = external_callback;
	}

private:
	BASE_LIBS_DECLARE_SERVICE_CALLBACK( serviceCB, typename __Service )
	{
		if( external_callback_ ) return external_callback_( request, response );
		return false;
	}
};

}

#endif // BASE_LIBS_BASE_LIBS_SERVICE_SERVER_POLICY_H_
