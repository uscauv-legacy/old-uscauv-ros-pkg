/***************************************************************************
 *  include/base_libs/param_reader.h
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

#include <base_libs/console.h>
#include <ros/param.h>
#include <ros/node_handle.h>
#include <type_traits>
#include <vector>
#include <sstream>

//BASE_LIBS_DECLARE_HEADER( BASE_LIBS, BASE_LIBS, PARAM_READER )

#ifndef BASE_LIBS_BASE_LIBS_PARAM_READER_H_
#define BASE_LIBS_BASE_LIBS_PARAM_READER_H_

// a class that will read up to __Dim__ paramters with the name format "prefix_|n|postfix_"
// a dim of 0 means "load all matching params that are found, without limit"
// so you can do something like:
// - ParamReader param_reader1( nh, ... ).readParams();
// - ParamReader param_reader2( nh ).readParams( ... );
// - ParamReader param_reader3( nh );
//   param_reader1.readParams( ... )
//   param_reader2.readParams( ... )
//   param_reader3.readParams( ... )

namespace ros
{

template<class __Storage, unsigned int __Dim__ = 0>
class ParamReader
{
public:
	typedef __Storage _Storage;
	typedef std::vector<__Storage> _Array;
	
	ros::NodeHandle nh_;
	std::string prefix_;
	std::string postfix_;
	unsigned int start_index_;
	_Array params_;
	
	ParamReader(){}
	
	ParamReader(
		ros::NodeHandle & nh,
		const std::string & prefix,
		const std::string & postfix = "",
		unsigned int start_index = 1 )
	:
		nh_( nh ),
		prefix_( prefix ),
		postfix_( postfix ),
		start_index_( start_index )
	{
		//
	}
	
public:
	_Array
		readParams()
	{
		params_ = readParams(
			nh_,
			prefix_,
			postfix_,
			start_index_ );
		
		return params_;
	}
	
	static _Array
		readParams(
			ros::NodeHandle & nh,
			const std::string & prefix,
			const std::string & postfix = "",
			unsigned int start_index = 1 )
	{
		_Array params;
		bool new_param_found = true;
	    unsigned int n = start_index;
	    do
	    {
			std::stringstream param_name_ss;
			param_name_ss << prefix << n << postfix;
			
			__Storage param_value;
			const std::string param_name = param_name_ss.str();
			
			if( nh.getParam(
				param_name.c_str(),
				param_value ) )
			{
				std::stringstream param_value_ss;
				param_value_ss << param_value;
				PRINT_INFO( "Loaded param [%s] with value [%s]", param_name.c_str(), param_value_ss.str().c_str() );
				params.push_back( param_value );
				++n;
			}
			else
			{
				PRINT_WARN( "Only found %i/%i parameters in array", n - start_index, __Dim__ );
				PRINT_WARN( "%s[%u:%u]%s", prefix.c_str(), start_index, start_index + __Dim__, postfix.c_str() );
				new_param_found = false;
			}
	    }
	    while( new_param_found && ( __Dim__ == 0 || n < __Dim__ + start_index ) );
		return params;
	}
};

// a ParamReader of length one can be initialized with a single
// nodehandle and readParam() can be called repeatedly to fetch params
// so you can do something like:
// - ParamReader param_reader1( nodehandle, ... ).getParam();
// - ParamReader param_reader2( nodehandle ).getParam( ... );
// - ParamReader param_reader3( nodehandle );
//   param_reader3.getParam( "param1" );
//   param_reader2.getParam( "param2" );
//   param_reader1.getParam( "param3" );

template<class __Storage>
class ParamReader<__Storage, 1>
{
public:
	ros::NodeHandle nh_;
	
	std::string param_name_;
	
	__Storage
		default_value_,
		last_value_;
	
	ParamReader(){}
	
	ParamReader(
		ros::NodeHandle & nh,
		const std::string & param_name = "",
		const __Storage & default_value = __Storage() )
	:
		nh_( nh ),
		default_value_( default_value ),
		last_value_( default_value_ )
	{
		//
	}
	
	__Storage
		readParam()
	{
		return readParam(
			nh_,
			param_name_,
			default_value_ );
	}
	
	static __Storage
		readParam(
			ros::NodeHandle & nh,
			const std::string & param_name,
			const __Storage & default_value = __Storage() )
	{
		__Storage param_value( default_value );
		
		const bool param_found( tryReadParam( nh, param_name, param_value ) );
		
		std::stringstream param_value_ss;
		param_value_ss << param_value;
		
		if( param_found )
		{
			PRINT_INFO( "> Using value [ %s ]", param_value_ss.str().c_str() );
		}
		else
		{
			PRINT_WARN( "> Defaulting to [ %s ]", param_value_ss.str().c_str() );
		}
		
		return  param_value;
	}
	
	static bool tryReadParam(
		ros::NodeHandle & nh,
		const std::string & param_name,
		__Storage & param_value )
	{
		if( nh.getParam(
			param_name.c_str(),
			param_value ) )
		{
			PRINT_INFO( "Found param [%s/%s]",
			nh.getNamespace().c_str(),
			param_name.c_str() );
			
			return true;
		}
		
		PRINT_WARN(
			"Could not find param [%s/%s]",
			nh.getNamespace().c_str(),
			param_name.c_str() );
		
		return false;
	}
};

}

#endif // BASE_LIBS_BASE_LIBS_PARAM_READER_H_
