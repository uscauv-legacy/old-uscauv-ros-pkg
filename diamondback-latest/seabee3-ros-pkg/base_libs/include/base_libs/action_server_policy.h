/***************************************************************************
 *  include/base_libs/action_server_policy.h
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

#ifndef BASE_LIBS_BASE_LIBS_ACTION_SERVER_POLICY_H_
#define BASE_LIBS_BASE_LIBS_ACTION_SERVER_POLICY_H_

#include <base_libs/node_handle_policy.h>
#include <base_libs/callback_policy.h>
#include <actionlib/server/simple_action_server.h>

BASE_LIBS_DECLARE_INTERNAL_NAMESPACE
{

template<class __Action>
class ActionServerPolicy : public GenericPolicyAdapter<NodeHandlePolicy, CallbackPolicy<void( const typename __Action::Goal::ConstPtr &, actionlib::SimpleActionServer<__Action> * const )> >
{
	BASE_LIBS_MAKE_POLICY_FUNCS( ActionServer )

private:
	typedef actionlib::SimpleActionServer<__Action> _ActionServer;
	typedef typename __Action::Goal _GoalMsg;
	typedef typename __Action::Feedback _FeedbackMsg;
	typedef typename __Action::Result _ResultMsg;
	typedef __BASE_LIBS_FUNCTION_TYPE<void ( const typename _GoalMsg::ConstPtr &, _ActionServer * const )> _Callback;
	//typedef __BASE_LIBS_FUNCTION_TYPE<void ( const typename _GoalMsg::ConstPtr & )> _InternalCallback;
	typedef decltype( CallbackPolicy_types::from_function<_Callback>::type ) _GoalMsgCallbackPolicy;
	typedef GenericPolicyAdapter<NodeHandlePolicy, _GoalMsgCallbackPolicy> _PolicyAdapter;

	_ActionServer * action_server_;
	_Callback callback_;
	std::string action_topic_name_;

	template<class... __Args>
	ActionServerPolicy( __Args&&... args )
	:
		_PolicyAdapter( args... ),
		action_server_( NULL ),
		initialized_( false )
	{
		printPolicyActionStart( "create", this );
		printPolicyActionDone( "create", this );
	}

	void postInit()
	{
		auto & nh_rel = NodeHandlePolicy::getNodeHandle();

		// we use simple_bind here to link the function required by 'server_' to the function defined by _FUNCTION_BASE_TYPE.
		// we need to pass f(x) to server_ but when server_ calls f(x), we actually want to call f(x,y)
		// so simple_bind takes f(x,y) and returns f(x); then, when f(x) is called, we automatically call f(x,y)
		action_server_( nh_rel, action_topic_name_, simple_bind( &ActionServerPolicy::executeActionCB, this ), false );
	}

	BASE_LIBS_ENABLE_INIT
	{
		printPolicyActionStart( "initialize", this );

		postInit();
		BASE_LIBS_SET_INITIALIZED();
		printPolicyActionDone( "initialize", this );
	}

	BASE_LIBS_DECLARE_MESSAGE_CALLBACK( executeActionCB, typename _GoalMsg )
	{
		BASE_LIBS_ASSERT_INITIALIZED();

		if( !action_server_ )
		{
			PRINT_ERROR( "Cannot send execute request to un-initialized server" );
			return;
		}

		_GoalMsgCallbackPolicy::invokeCallback( msg, action_server_ );
	}
};

}

#endif // BASE_LIBS_BASE_LIBS_ACTION_SERVER_POLICY_H_
