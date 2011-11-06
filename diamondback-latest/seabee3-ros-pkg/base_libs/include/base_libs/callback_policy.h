/***************************************************************************
 *  include/base_libs/callback_policy.h
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

#ifndef BASE_LIBS_BASE_LIBS_CALLBACK_POLICY_H_
#define BASE_LIBS_BASE_LIBS_CALLBACK_POLICY_H_

#include <base_libs/policy.h>

BASE_LIBS_DECLARE_INTERNAL_NAMESPACE
{

namespace adapters
{

// ########## CallbackAdapter ##################################################################################################################
template<class __Return, class... __Args>
struct CallbackAdapter
{
	typedef __BASE_LIBS_FUNCTION_TYPE<__Return( __Args&&... )> _CallbackType;

	_CallbackType external_callback_;
};

// ########## MessageCallbackAdapter for ROS messages ##########################################################################################
template<class __Message>
struct MessageCallbackAdapter : public CallbackAdapter<void, const typename __Message::ConstPtr &>{};

// ########## Specialization of MessageCallbackAdapter for empty/disabled (void) callbacks #####################################################
template<>
struct MessageCallbackAdapter<void>{};

} // adapters


// ########## Generic callback policy for any kind of return type/arg types ####################################################################
BASE_LIBS_DECLARE_POLICY( Callback, Policy )

template<class __CallbackReturn, class... __CallbackArgs>
BASE_LIBS_DECLARE_POLICY_CLASS( Callback )
{
	BASE_LIBS_MAKE_POLICY_FUNCS( Callback )

private:
	typedef adapters::CallbackAdapter<__CallbackReturn, __CallbackArgs...> _CallbackAdapter;
	_CallbackAdapter callback_adapter_;

	BASE_LIBS_DECLARE_POLICY_CONSTRUCTOR( Callback )
	{
		printPolicyActionStart( "create", this );
		printPolicyActionDone( "create", this );
	}

	void registerCallback( const typename _CallbackAdapter::_CallbackType & external_callback )
	{
		callback_adapter_.external_callback_ = external_callback;
	}

	template<class __Return>
	BASE_LIBS_ENABLE_IF_SAME( __CallbackReturn, __Return, void )
	invokeCallback_0( __CallbackArgs&&... args ) const
	{
		if( callback_adapter_.external_callback_ ) callback_adapter_.external_callback_( args... );
	}

	template<class __Return>
	BASE_LIBS_ENABLE_IF_NOT_SAME( __CallbackReturn, __Return, void )
	invokeCallback_0( __CallbackArgs&&... args ) const
	{
		const static __CallbackReturn default_return = __CallbackReturn();

		if( !callback_adapter_.external_callback_ ) return default_return;
		return callback_adapter_.external_callback_( args... );
	}

	__CallbackReturn invokeCallback( __CallbackArgs&&... args ) const
	{
		// in order to enable/disable functions with enable_if, they need to be directly dependent on some outer type
		return invokeCallback_0<__CallbackReturn>( args... );
	}
};

// ########## Base for common data in specializations of MessageCallbackPolicy #################################################################
class MessageCallbackPolicyBase : public Policy
{
	BASE_LIBS_MAKE_POLICY_NAME( MessageCallback )

	MessageCallbackPolicyBase()
	{
		printPolicyActionStart( "create", this );
		printPolicyActionDone( "create", this );
	}
};

// ########## Special callback policy for message-based callbacks ##############################################################################
template<class __Message>
class MessageCallbackPolicy : public MessageCallbackPolicyBase, public CallbackPolicy<void, const typename __Message::ConstPtr &>
{
private:
	typedef CallbackPolicy<void, const typename __Message::ConstPtr &> _CallbackPolicy;
	typedef void _EmptyMsg;

public:
	template<class... __Args>
	MessageCallbackPolicy( __Args&&... args ) : _CallbackPolicy( args... )
	{
		//
	}

	void registerCallback( const typename adapters::MessageCallbackAdapter<__Message>::_CallbackType & external_callback )
	{
		_CallbackPolicy::registerCallback( external_callback );
	}
};

// ########## Specialization of MessageCallbackPolicy for empty/disabled (void) messages #######################################################
template<>
class MessageCallbackPolicy<void> : public MessageCallbackPolicyBase, public CallbackPolicy<void>
{
public:
	template<class... __Args>
	MessageCallbackPolicy( __Args&&... args ) : CallbackPolicy<void>( args... ) {}

	void registerCallback() {}
};

} // base_libs

#endif // BASE_LIBS_BASE_LIBS_CALLBACK_POLICY_H_
