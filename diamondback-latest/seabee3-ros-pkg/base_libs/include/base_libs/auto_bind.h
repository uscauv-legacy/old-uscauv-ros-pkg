/***************************************************************************
 *  include/base_libs/auto_bind.h
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

#ifndef BASE_LIBS_BASE_LIBS_AUTO_BIND_H_
#define BASE_LIBS_BASE_LIBS_AUTO_BIND_H_

#include <functional>

/* auto_binder inspired by code written by "superbonzo" and "Chris_F" (http://www.codeguru.com/forum/showthread.php?t=512875) */

#define _AUTO_BIND_FUNCTION_TYPE std::function

namespace details
{

template<int N> struct placeholder {};
template<typename... __ArgTypes> struct container{};

}

namespace std
{

template<int N> struct is_placeholder< details::placeholder<N> > : std::integral_constant<int,N> {};

}

namespace details
{

template<class __FunctionType>
struct funct_types {};

template<class __ReturnType, class... __ArgTypes>
struct funct_types<_AUTO_BIND_FUNCTION_TYPE<__ReturnType(__ArgTypes...)> >
{
	typedef container<__ArgTypes...> _ArgTypesContainer;
	typedef __ReturnType _ReturnType;
};

template<int N>
struct auto_binder
{
	template <typename __CallerType, typename __ReturnType, typename... __ArgTypes, typename... __PlaceHolders>
	static _AUTO_BIND_FUNCTION_TYPE<__ReturnType(__ArgTypes...)> auto_bind(__ReturnType(__CallerType::*function_ptr)(__ArgTypes...), __CallerType* const caller, __PlaceHolders... placeholders )
	{
		return auto_binder<N-1>::auto_bind( function_ptr, caller, placeholder<N>(), placeholders... );
	}
	
	template <typename __ReturnType, typename... __ArgTypes, typename... __PlaceHolders>
	static _AUTO_BIND_FUNCTION_TYPE<__ReturnType(__ArgTypes...)> auto_bind(__ReturnType(*function_ptr)(__ArgTypes...), __PlaceHolders... placeholders )
	{
		return auto_binder<N-1>::auto_bind( function_ptr, placeholder<N>(), placeholders... );
	}
	
	template <typename... __OutputArgTypes, typename __ReturnType, typename... __InputArgTypes, typename... __Appends>
	static _AUTO_BIND_FUNCTION_TYPE<__ReturnType(__OutputArgTypes...)> auto_bind_append( const container<__OutputArgTypes...> & container, const _AUTO_BIND_FUNCTION_TYPE<__ReturnType(__InputArgTypes...)> & function, __Appends... appends )
	{
		return auto_binder<N-1>::auto_bind_append( container, function, placeholder<N>(), appends... );
	}
};

template<>
struct auto_binder<0>
{
	template <typename __CallerType, typename __ReturnType, typename... __ArgTypes, typename... __PlaceHolders>
	static _AUTO_BIND_FUNCTION_TYPE<__ReturnType(__ArgTypes...)> auto_bind(__ReturnType(__CallerType::*function_ptr)(__ArgTypes...), __CallerType* const caller, __PlaceHolders... placeholders )
	{
		return _AUTO_BIND_FUNCTION_TYPE<__ReturnType(__ArgTypes...)>( std::bind( function_ptr, caller, placeholders... ) );
	}
	
	template <typename __ReturnType, typename... __ArgTypes, typename... __PlaceHolders>
	static _AUTO_BIND_FUNCTION_TYPE<__ReturnType(__ArgTypes...)> auto_bind(__ReturnType(*function_ptr)(__ArgTypes...), __PlaceHolders... placeholders )
	{
		return _AUTO_BIND_FUNCTION_TYPE<__ReturnType(__ArgTypes...)>( std::bind( function_ptr, placeholders... ) );
	}
	
	template <typename... __OutputArgTypes, typename __ReturnType, typename... __InputArgTypes, typename... __Appends>
	static _AUTO_BIND_FUNCTION_TYPE<__ReturnType(__OutputArgTypes...)> auto_bind_append( const container<__OutputArgTypes...> & container, const _AUTO_BIND_FUNCTION_TYPE<__ReturnType(__InputArgTypes...)> & function, __Appends... appends )
	{
		return _AUTO_BIND_FUNCTION_TYPE<__ReturnType( __OutputArgTypes... )>( std::bind( function, appends... ) );
	}
};

}

namespace base_libs
{

template <typename __CallerType, typename __ReturnType, typename... __ArgTypes>
_AUTO_BIND_FUNCTION_TYPE<__ReturnType(__ArgTypes...)> auto_bind(__ReturnType(__CallerType::*function_ptr)(__ArgTypes...), __CallerType* const caller)
{
	return details::auto_binder<sizeof...(__ArgTypes)>::auto_bind( function_ptr, caller );
}

template <typename __ReturnType, typename... __ArgTypes>
_AUTO_BIND_FUNCTION_TYPE<__ReturnType(__ArgTypes...)> auto_bind(__ReturnType(*function_ptr)(__ArgTypes...))
{
	return details::auto_binder<sizeof...(__ArgTypes)>::auto_bind( function_ptr );
}

// this a bit ugly because the compiler was failing to pick up on the output function's args, so now the function type has to be manually specified with __OutputFunctionType and the output args can then be extracted
/* usage:
 * 	std::auto_bind<__OutputFunctionType>( @input_function, @args_to_append... )
 * 
 * example:
 * 	typedef std::function<void(int, int, int)> _FunctionType1
 * 	typedef std::function<void(int, int, int, int)> _FunctionType2
 * 	
 * 	void some_function( int arg1, int arg2, int arg3, int arg4 ){};
 * 	const int value_of_arg4 = 5;
 * 
 * 	// the usual thing to do is:
 *  _FunctionType1 other_function_boost ( boost::bind( &some_function, _1, _2, _3, value_of_arg4 ) );
 * 	other_function_boost( 4, 5, 6 ); // calls some_function( 4, 5, 6, value_of_arg4 );
 * 
 * 	// but now you can do:
 * 	_FunctionType1 other_function ( std::bind<_FunctionType1>( &some_function, value_of_arg4 ) );
 * 	other_function( 4, 5, 6 ); // calls some_function( 4, 5, 6, value_of_arg4 );
 * 
 *  // the improvement here is that placeholders are automatically calculated
 * 	// so if _FunctionType1 had N more arguments, the code for std::bind would remain the same
 * 	// whereas with boost it would be necessary to specify an additional N placeholders for those arguments
 * 
 * 	// the downside is that you can't re-order arguments like you can with boost
 * 
 */

/*template <typename __OutputFunctionType, typename __ReturnType, typename __CallerType, typename... __InputArgTypes, typename... __Appends>
auto auto_bind( __ReturnType(__CallerType::*function_ptr)(__InputArgTypes...), __CallerType* caller, __Appends... appends ) -> decltype( auto_bind( details::auto_binder<sizeof...(__InputArgTypes)>::auto_bind( function_ptr, caller ), appends... ) )
{
	return auto_bind( details::auto_binder<sizeof...(__InputArgTypes)>::auto_bind( function_ptr, caller ), appends... );
}*/

template <typename __OutputFunctionType, typename __ReturnType, typename... __InputArgTypes, typename... __Appends>
auto auto_bind( const _AUTO_BIND_FUNCTION_TYPE<__ReturnType(__InputArgTypes...)> & function, __Appends... appends ) -> decltype( auto_bind( typename details::funct_types<__OutputFunctionType>::_ArgTypesContainer(), function, appends... ) )
{
	// first, we need to find out how many args are in the return function type, since we need a placeholder for each of those args
	return auto_bind( typename details::funct_types<__OutputFunctionType>::_ArgTypesContainer(), function, appends... );
}

// we need to make a placeholder for each item of __OutputArgTypes...
template <typename... __OutputArgTypes, typename __ReturnType, typename... __InputArgTypes, typename... __Appends>
static _AUTO_BIND_FUNCTION_TYPE<__ReturnType(__OutputArgTypes...)> auto_bind( const details::container<__OutputArgTypes...> & container, const _AUTO_BIND_FUNCTION_TYPE<__ReturnType(__InputArgTypes...)> & function, __Appends... appends )
{
	return details::auto_binder<sizeof...(__OutputArgTypes)>::auto_bind_append( container, function, appends... );
}

}

#endif // BASE_LIBS_BASE_LIBS_AUTO_BIND_H_

/*###################################################################*/
