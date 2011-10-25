/***************************************************************************
 *  include/base_libs/policy.h
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

#ifndef BASE_LIBS_BASE_LIBS_POLICY_H_
#define BASE_LIBS_BASE_LIBS_POLICY_H_

#include <base_libs/console.h>
#include <base_libs/generic_policy_adapter.h>
#include <base_libs/type_utils.h>

namespace base_libs
{

class Policy
{
public:
	const static inline std::string name(){ return "Base"; }
	
	template<class... __Args>
	Policy( __Args&&... args )
	{
		// discard args... but don't fail to compile if anything is passed in
		PRINT_INFO( "########## Policy Group ##########" );
		printPolicyActionStart( "create", this );
		printPolicyActionDone( "create", this );
	}
	
	template<class __Policy>
	static void printPolicyAction( std::string action, __Policy * policy )
	{
		PRINT_INFO( "[%s] on policy: [%s]", action.c_str(), __Policy::name().c_str() );
	}
	
	template<class __Policy>
	static void printPolicyActionStart( std::string action, __Policy * policy )
	{
		printPolicyAction( ">>>>> Start: " + action, policy );
	}
	
	template<class __Policy>
	static void printPolicyActionDone( std::string action, __Policy * policy )
	{
		printPolicyAction( "<<<<< Done: " + action, policy );
	}
};

}

#endif // BASE_LIBS_BASE_LIBS_POLICY_H_
