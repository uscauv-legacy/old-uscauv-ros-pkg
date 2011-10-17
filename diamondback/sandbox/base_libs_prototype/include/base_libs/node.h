/***************************************************************************
 *  include/base_libs/node.h
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

#ifndef BASE_LIBS_BASE_LIBS_NODE_H_
#define BASE_LIBS_BASE_LIBS_NODE_H_

#include <base_libs/runable_policy.h>

namespace base_libs
{

// wrap GenericPolicyAdapter and add RunablePolicy to the front of __Policies...
template<class... __Policies>
class Node : public GenericPolicyAdapter<RunablePolicy, __Policies...>
{
public:
	template<class... __Args>
	Node( __Args&&... args ) : GenericPolicyAdapter<RunablePolicy, __Policies...>( args... )
	{
		
	}
	
	// for all non-initable policies
	template<class __Policy, class... __Args>
	typename std::enable_if<( !__Policy::HAS_UPDATE_ ), void>::type
	tryUpdate( __Args&&... args ) {}
	
	// for all initable policies
	template<class __Policy, class... __Args>
	typename std::enable_if<( __Policy::HAS_UPDATE_ ), void>::type
	tryUpdate( __Args&&... args )
	{
		__Policy::update( args... );
	}
	
	// initialize the first policy in the subset
	// recurse through the remaining policies in the subset
	template<class __PoliciesSubset, class... __Args>
	typename std::enable_if<(__PoliciesSubset::num_types_ > 0), void>::type
	updateRec( __Args&&... args )
	{
		tryUpdate<typename ContainerTypes<__PoliciesSubset>::_Front>( args... );
		updateRec<typename ContainerTypes<__PoliciesSubset>::_Rest>( args... );
	}
	
	template<class __PoliciesSubset, class... __Args>
	typename std::enable_if<(__PoliciesSubset::num_types_ == 0), void>::type
	updateRec( __Args&&... args ) {}
	
	// Do any post-construction initialization. Note that all policies
	// receive the same set of args.
	template<class... __Args>
	void updateAll( __Args&&... args )
	{
		updateRec<Container<__Policies...> >( args... );
	}
	
	BASE_LIBS_ENABLE_UPDATE{}
};

}

#endif // BASE_LIBS_BASE_LIBS_NODE_H_
