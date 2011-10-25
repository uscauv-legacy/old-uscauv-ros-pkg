/***************************************************************************
 *  include/base_libs/type_utils.h
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

#ifndef BASE_LIBS_BASE_LIBS_TYPE_UTILS_H_
#define BASE_LIBS_BASE_LIBS_TYPE_UTILS_H_

#include <type_traits>
#include <string>
#include <stdlib.h>
#include <base_libs/console.h>
#include <boost/shared_ptr.hpp>

namespace base_libs
{
	struct TYPE_NOT_FOUND{};
	
	template<class __Desired>
	static __Desired getFirstOfType(){ return TYPE_NOT_FOUND(); }
	
	// matching value found; return it
	template<class __Desired, class __Current, class... __Rest>
	static typename std::enable_if<std::is_same<__Desired, __Current>::value, __Desired>::type
	getFirstOfType( __Current & current, __Rest&&... rest )
	{
		return current;
	}
	
	// iterate through types in __Rest until a matching type is found
	// this will fail at compile time if no matching type exists
	template<class __Desired, class __Current, class... __Rest>
	static typename std::enable_if<!std::is_same<__Desired, __Current>::value, __Desired>::type
	getFirstOfType( __Current & current, __Rest&&... rest )
	{
		return getFirstOfType<__Desired>( rest... );
	}
	
	template<class __Desired>
	__Desired getMetaParamRec( const std::string & name, const __Desired & default_value )
	{
		PRINT_WARN( "Failed to find key [%s]", name.c_str() );
		return default_value;
		// fail at runtime
		//abort();
	}
	
	// ####
	
	template<class __Desired, class __Current, class... __Rest>
	static typename std::enable_if<(!std::is_same<__Desired, __Current>::value), __Desired>::type
	getMetaParamRec( const std::string & name, const __Desired & default_value, const std::string & current_name, __Current & current, __Rest&&... rest );
	
	template<class __Desired, class __Current, class... __Rest>
	static typename std::enable_if<(std::is_same<__Desired, __Current>::value), __Desired>::type
	getMetaParamRec( const std::string & name, const __Desired & default_value, const std::string & current_name, __Current & current, __Rest&&... rest );
	
	// ####
	
	template<class __Desired, class __Current, class... __Rest>
	static typename std::enable_if<(!std::is_same<__Desired, __Current>::value), __Desired>::type
	getMetaParamRec( const std::string & name, const __Desired & default_value, const std::string & current_name, __Current & current, __Rest&&... rest )
	{
		return getMetaParamRec<__Desired>( name, default_value, rest... );
	}
	
	template<class __Desired, class __Current, class... __Rest>
	static typename std::enable_if<(std::is_same<__Desired, __Current>::value), __Desired>::type
	getMetaParamRec( const std::string & name, const __Desired & default_value, const std::string & current_name, __Current & current, __Rest&&... rest )
	{
		PRINT_INFO( "Found key [%s]", name.c_str() );
		return name == current_name ? current : getMetaParamRec<__Desired>( name, default_value, rest... );
	}
	
	// ####
	
	template<class __Desired, class... __Rest>
	static __Desired getMetaParam( const std::string & name, __Rest&&... rest )
	{
		// make sure desired type exists in list; otherwise fail at compile time
		getFirstOfType<__Desired>( rest... );
		return getMetaParamRec<__Desired>( name, __Desired(), rest... );
	}
	
	template<class __Desired, class... __Rest>
	static __Desired getMetaParamDef( const std::string & name, const __Desired & default_value, __Rest&&... rest )
	{
		return getMetaParamRec<__Desired>( name, default_value, rest... );
	}
	
	template<class __Type>
	struct makePtr
	{
		typedef boost::shared_ptr<__Type> _Shared;
		typedef boost::shared_ptr<__Type const> _Const;
	};
}

#endif // BASE_LIBS_BASE_LIBS_TYPE_UTILS_H_
