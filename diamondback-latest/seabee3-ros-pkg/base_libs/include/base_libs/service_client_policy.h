/***************************************************************************
 *  include/base_libs/service_client_policy.h
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

#ifndef BASE_LIBS_BASE_LIBS_SERVICE_CLIENT_POLICY_H_
#define BASE_LIBS_BASE_LIBS_SERVICE_CLIENT_POLICY_H_

#include <base_libs/node_handle_policy.h>
#include <ros/service_client.h>

namespace base_libs
{

BASE_LIBS_DECLARE_POLICY( ServiceClient, NodeHandlePolicy )

template<class __Service, unsigned int __Id__ = 0>
BASE_LIBS_DECLARE_POLICY_CLASS( ServiceClient )
{
	BASE_LIBS_MAKE_POLICY_NAME( ServiceClient )

public:
	typedef typename __Service::Request _ServiceRequest;
	typedef typename __Service::Response _ServiceResponse;
	
private:
	ros::ServiceClient client_;
	
	std::string service_name_, service_topic_name_;
	bool is_valid_;
	
	const static double _DEF_callService_wait_time = 2.0;
	const static unsigned int _DEF_callService_attempts = 1;
	
	BASE_LIBS_DECLARE_POLICY_CONSTRUCTOR( ServiceClient ),
		service_name_( "service" ),
		service_topic_name_( "service" ),
		is_valid_( false ),
		initialized_( false )
	{
		printPolicyActionStart( "create", this );
		printPolicyActionDone( "create", this );
	}
	
	BASE_LIBS_ENABLE_INIT
	{
		printPolicyActionStart( "initialize", this );
		
		ros::NodeHandle & nh_rel = NodeHandlePolicy::getNodeHandle();
		// look up the meta-param remap of the service name param in args..., if it exists
		const std::string service_name_param = getMetaParamDef<std::string>( "service_name_param", "service_name", args... );
		// look up the ros-param rempa of the service name on the ros parameter server, if it exists
		service_name_ = ros::ParamReader<std::string, 1>::readParam( nh_rel, service_name_param, service_name_ );
		service_topic_name_ = ros::NodeHandle( nh_rel, service_name_ ).getNamespace();
		// now that we have a final value for the service name, create a service with that name
		
		BASE_LIBS_SET_INITIALIZED;
		
		connectToService( true );
		
		printPolicyActionDone( "initialize", this );
	}
	
	/*! Attempt to connect to the service
	 *  - Print warnings if the connection attempt failed
	 *  - If \param show_status_on_success and the attempt to connect succeeded, show that info
	 * 
	 *  \return is_valid_, the state of the service connection */
	bool connectToService( const bool & show_status_on_success = false )
	{
		BASE_LIBS_CHECK_INITIALIZED;
		
		ros::NodeHandle & nh_rel = NodeHandlePolicy::getNodeHandle();
		
		is_valid_ = client_.exists() && client_.isValid();
		if( !is_valid_ )
		{
			PRINT_INFO( "Attempting to connect to service [%s]...", service_topic_name_.c_str() );
			client_ = nh_rel.serviceClient<__Service>( service_name_, true );
			is_valid_ = client_.exists() && client_.isValid();
			
			if( is_valid_ ) PRINT_INFO( "Connected to service [%s].", service_topic_name_.c_str() );
			else PRINT_WARN( "Could not connect to service [%s]!", service_topic_name_.c_str() );
		}
		
		return is_valid_;
	}
	
	/*! Attempt to call the service.
	 *  - If the service is not available, attempt to connect \param attempts times before aborting, waiting \param wait_time seconds in between each attempt
	 *  - if \param attempts is 0, try indefinitely
	 *
	 *  \return false if aborted, otherwise return the result of the service call */
	
	bool callService( __Service & service, const ros::Duration & wait_time = ros::Duration( _DEF_callService_wait_time ), unsigned int attempts = _DEF_callService_attempts )
	{
		return callService( service.request, service.response, wait_time, attempts );
	}
	
	/*! Expanded version of callService that takes \param service.request and \param service.response as arguments instead of just \param service */
	bool callService( _ServiceRequest & request, _ServiceResponse & response, const ros::Duration & wait_time = ros::Duration( _DEF_callService_wait_time ), unsigned int attempts = _DEF_callService_attempts )
	{
		while( !connectToService() )
		{
			// if we're on the last attempt and we still haven't connected, abort
			if( attempts == 1 ) return false;
			
			if( attempts > 0 ) --attempts;
				
			if( attempts > 0 ) PRINT_WARN( "Will try %u more times before aborting.", attempts );
			else PRINT_WARN( "Will try indefinitely." );
			
			client_.waitForExistence( wait_time );
		}
		
		return client_.call( request, response );
	}
};

}

#endif // BASE_LIBS_BASE_LIBS_SERVICE_CLIENT_POLICY_H_
