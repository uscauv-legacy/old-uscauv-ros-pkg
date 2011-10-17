/***************************************************************************
 *  include/base_libs/generic_policy_adapter.h
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

#ifndef BASE_LIBS_BASE_LIBS_GENERIC_POLICY_ADAPTER_H_
#define BASE_LIBS_BASE_LIBS_GENERIC_POLICY_ADAPTER_H_

#include <base_libs/macros.h>
#include <base_libs/container.h>
#include <utility>

namespace base_libs
{

template<class... __Policies>
class GenericPolicyAdapter : public __Policies...
{
public:
	// Construct a policy adapter that inherits from all specified
	// policies. Note that all policies in the inheritance group take
	// the same set of args during construction.
	
	// gcc can't compile this; "invalid use of pack expansion expression"
	// see http://stackoverflow.com/questions/7694201/unpacking-parameter-pack-of-args-into-the-constructor-of-each-class-defined-in-a
	//template<class... __Args>
	//GenericPolicyAdapter( __Args... args ) : __Policies( args... )
	
	template<class __Arg>
	GenericPolicyAdapter( __Arg & arg ) : __Policies( arg )...
	{
		
	}
	
	// for all non-initable policies
	template<class __Policy, class... __Args>
	typename std::enable_if<( !__Policy::HAS_INIT_ ), void>::type
	tryInit( __Args&&... args ) {}
	
	// for all initable policies
	template<class __Policy, class... __Args>
	typename std::enable_if<( __Policy::HAS_INIT_ ), void>::type
	tryInit( __Args&&... args )
	{
		__Policy::init( args... );
	}
	
	// initialize the first policy in the subset
	// recurse through the remaining policies in the subset
	template<class __PoliciesSubset, class... __Args>
	typename std::enable_if<(__PoliciesSubset::num_types_ > 0), void>::type
	initRec( __Args&&... args )
	{
		tryInit<typename ContainerTypes<__PoliciesSubset>::_Front>( args... );
		initRec<typename ContainerTypes<__PoliciesSubset>::_Rest>( args... );
	}
	
	template<class __PoliciesSubset, class... __Args>
	typename std::enable_if<(__PoliciesSubset::num_types_ == 0), void>::type
	initRec( __Args&&... args ) {}
	
	// Do any post-construction initialization. Note that all policies
	// receive the same set of args.
	template<class... __Args>
	void initAll( __Args&&... args )
	{
		initRec<Container<__Policies...> >( args... );
	}
	
	BASE_LIBS_ENABLE_INIT{}
};

}

#endif // BASE_LIBS_BASE_LIBS_GENERIC_POLICY_ADAPTER_H_
