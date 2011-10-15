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
		return readParams(
			prefix_,
			postfix_,
			start_index_ );
	}
	
	_Array
		readParams(
			const std::string & prefix,
			const std::string & postfix = "",
			unsigned int start_index = 1 )
	{
		params_.clear();
		bool new_param_found = true;
	    unsigned int n = start_index_;
	    do
	    {
			std::stringstream ss;
			ss << prefix_ << n << postfix_;
			
			__Storage param_value;
			const std::string param_name = ss.str();
			
			if( nh_.getParam(
				param_name.c_str(),
				param_value ) )
			{
				PRINT_INFO( "Loaded param [%s] with value [%s]", param_name.c_str(), param_value );
				params_.push_back( param_value );
				++n;
			}
			else new_param_found = false;
	    }
	    while( new_param_found && ( __Dim__ == 0 || n < __Dim__ + start_index_ ) );
		return params_;
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
		
		std::stringstream ss;
		ss << param_value;
		
		if( param_found )
		{
			PRINT_INFO( "> Using value [ %s ]", ss.str().c_str() );
		}
		else
		{
			PRINT_WARN( "> Defaulting to [ %s ]", ss.str().c_str() );
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
