/***************************************************************************
 *  include/uscauv_common/param_loader.h
 *  --------------------
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Dylan Foster (turtlecannon@gmail.com)
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
 *  * Neither the name of USC AUV nor the names of its
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


#ifndef USCAUV_USCAUVCOMMON_PARAMLOADER
#define USCAUV_USCAUVCOMMON_PARAMLOADER

// ROS
#include <ros/ros.h>

#include <XmlRpcValue.h>
#include <XmlRpcException.h>

// ################################################################
// ################################################################

namespace uscauv
{
  namespace param
  {
  
    typedef XmlRpc::XmlRpcValue XmlRpcValue;

    // ################################################################
    // ################################################################

    template<typename __ParamType, typename Enable = void>
      struct XmlRpcValueConverter;

    template<typename __ParamType>
      struct XmlRpcValueConverter<__ParamType, typename std::enable_if< std::is_arithmetic<__ParamType>::value, void>::type>
    {
      static __ParamType convert(XmlRpcValue & param)
      {
	if( param.getType() == XmlRpcValue::TypeBoolean ) return bool(param);
	else if ( param.getType() == XmlRpcValue::TypeInt ) return int(param);
	else if ( param.getType() == XmlRpcValue::TypeDouble ) return double(param);
	else return __ParamType(param); /// TODO: Assert or something - this shouldn't happen
      }
    };
  
    template<typename __NestedParamType>
      struct XmlRpcValueConverter< std::vector<__NestedParamType> >
      {

	static std::vector<__NestedParamType> convert(XmlRpcValue & param)
	{
	  std::vector<__NestedParamType> output;

	  for(int idx = 0; idx < param.size(); ++idx )
	    {
	      __NestedParamType p;
	      XmlRpcValue & val = param[idx];
		
	      try
		{
		  p = XmlRpcValueConverter<__NestedParamType>::convert( val );
		}
	      catch(typename XmlRpc::XmlRpcException & ex)
		{
		  ROS_WARN("Caught XmlRpc exception [ %s ] loading element at idx [ %d ]. Skipping...", ex.getMessage().c_str(), idx);
		  continue;
		}
	      catch(std::exception & ex)
		{
		  ROS_WARN("Caught exception [ %s ] loading element at idx [ %d ]. Skipping...", ex.what(), idx);
		  continue;
		}
		
	      output.push_back ( p );
	    }
	  return output;
	}
	
      };


    template<typename __NestedParamType>
      struct XmlRpcValueConverter< std::map<std::string, __NestedParamType> >
      {
	static std::map<std::string, __NestedParamType> 
	  convert(XmlRpcValue & param)
	  {
	    std::map<std::string, __NestedParamType> output;

	    for( XmlRpcValue::ValueStruct::value_type & elem : param )
	      {
		std::pair<std::string, __NestedParamType> p;

		try
		  {
		    p = std::make_pair( elem.first, XmlRpcValueConverter<__NestedParamType>::convert( elem.second ) );
		  }
		catch( typename XmlRpc::XmlRpcException & ex)
		  {
		    ROS_WARN("Caught XmlRpc exception [ %s ] loading element [ %s ]. Skipping...", ex.getMessage().c_str(), elem.first.c_str());
		    continue;
		  }
		catch(std::exception & ex)
		  {
		    ROS_WARN("Caught exception [ %s ] loading element [ %s ]. Skipping...", ex.what(), elem.first.c_str());
		    continue;
		  }
	    
		ROS_INFO("Loaded map param element [ %s ].", elem.first.c_str() );
		output.insert( p );
	      }
	
	    return output;
	  }
      };

    /// Only works if the target object has a copy constructor
    template<typename __NestedParamType>
      struct XmlRpcValueConverter< std::shared_ptr<__NestedParamType> >
      {
	static std::shared_ptr<__NestedParamType> convert(XmlRpcValue & param)
	{
	  return std::make_shared<__NestedParamType>( XmlRpcValueConverter<__NestedParamType>::convert( param ) );
	}
      };

    // ################################################################
    // ################################################################

    template <typename __ParamType>
      static typename std::enable_if< !std::is_arithmetic<__ParamType>::value, std::string>::type
      toString(__ParamType const & param)
      {
	return "Aggregate Type";
      }

    template <typename __ParamType>
      static typename std::enable_if< std::is_arithmetic<__ParamType>::value, std::string>::type
      toString(__ParamType const & param)
      {
	std::stringstream ss;
	ss << std::boolalpha << param;
	return ss.str();
      }
  
    /// Defined in param_loader.cpp to make that sure linkage works out and there is no -Wunused-function)
    std::string toString(std::string const & param);
    std::string toString(XmlRpcValue const & param);

    // ################################################################
    // ################################################################

    template<typename __ParamType>
    static __ParamType lookup( XmlRpcValue & base_param, std::string const & param_name, __ParamType default_value, bool const & quiet = false)
      {
	if( base_param.hasMember( param_name ) )
	  return XmlRpcValueConverter<__ParamType>::convert(base_param[ param_name ]);
	
	if(!quiet){
	  ROS_WARN_STREAM("Couldn't find param [ " << param_name << " ]. Using default value [ " <<
			  toString(default_value) << " ]." );
	}
	
	return default_value;
      }

    /// Intended to be wrapped in try/catch blocks
    template<typename __ParamType>
      static __ParamType lookup( XmlRpcValue & base_param, std::string const & param_name) throw( XmlRpc::XmlRpcException )
      {
	if( !base_param.hasMember( param_name ) )
	  {
	    std::stringstream ss; ss << "Couldn't find param [ " << param_name << " ].";

	    XmlRpc::XmlRpcException ex( ss.str() );
	    throw ex;
	  }
	
	return XmlRpcValueConverter<__ParamType>::convert( base_param[ param_name ] );
      }

    // ################################################################
    // ################################################################
    
    template <typename __ParamType>
      static __ParamType load( ros::NodeHandle const & nh, std::string const & name, __ParamType const & default_value)
      {
	XmlRpcValue value;
	std::string const & resolved_name = nh.resolveName(name, true);
	bool const param_found = nh.getParam( name, value );

	if( param_found )
	  {
	    __ParamType return_value;

	    try
	      {
		return_value = XmlRpcValueConverter<__ParamType>::convert(value);
	      }
	    catch( XmlRpc::XmlRpcException & ex )
	      {
		ROS_WARN("Caught XmlRpc exception [ %s ] while loading param [ %s ]. Using default [ %s ].",
			 ex.getMessage().c_str(), resolved_name.c_str(), toString(default_value).c_str());
		
		return default_value;
	      }
	    catch( std::exception & ex )
	      {
		ROS_WARN("Caught exception [ %s ] while loading param [ %s ]. Using default [ %s ].",
			 ex.what(), resolved_name.c_str(), toString(default_value).c_str());
		
		return default_value;
	      }
	    ROS_INFO("Loaded param [ %s ] with value [ %s ].", 
		     resolved_name.c_str(), toString(return_value).c_str());
	  
	    return return_value;
	  }

	ROS_WARN("Failed to load param [ %s ]. Using default [ %s ].",
		 resolved_name.c_str(), toString(default_value).c_str());

	return default_value;

      }
    
    /// Load param successfully or kill node
    template <typename __ParamType>
      static __ParamType load( ros::NodeHandle const & nh, std::string const & name)
      {
	XmlRpcValue value;
	std::string const & resolved_name = nh.resolveName(name, true);
	bool const param_found = nh.getParam( name, value );

	if( param_found )
	  {
	    __ParamType return_value;

	    try
	      {
		return_value = XmlRpcValueConverter<__ParamType>::convert(value);
	      }
	    catch( XmlRpc::XmlRpcException & ex)
	      {	
		ROS_FATAL("Caught XmlRpc exception [ %s ] loading param [ %s ].  Shutting down...",
			  ex.getMessage().c_str(), resolved_name.c_str());
		ros::shutdown();
		return __ParamType();
	      }
	    catch( std::exception & ex)
	      {	
		ROS_FATAL("Caught exception [ %s ] loading param [ %s ].  Shutting down...",
			  ex.what(), resolved_name.c_str());
		ros::shutdown();
		return __ParamType();
	      }
	    
	    ROS_INFO("Loaded param [ %s ] with value [ %s ].", 
		     resolved_name.c_str(), toString(return_value).c_str());
	  
	    return return_value;
	  }
	
	ROS_FATAL("Failed to load param [ %s ].  Shutting down...",
		  resolved_name.c_str());
	ros::shutdown();
	return __ParamType();
	
      }
  } // param
} // uscauv

// ################################################################
// ################################################################

#define USCAUV_DECLARE_PARAM_LOADER_CONVERSION(__ParamType, var_name, conversion ) \
  namespace uscauv							\
  {									\
    namespace param							\
    {									\
      template<>							\
	struct XmlRpcValueConverter<__ParamType>			\
      {									\
	static __ParamType convert( XmlRpcValue & var_name ) throw(std::exception) \
	{								\
	  conversion;							\
	}								\
      };								\
    }									\
  }									\


USCAUV_DECLARE_PARAM_LOADER_CONVERSION( XmlRpc::XmlRpcValue, param, return param; )
USCAUV_DECLARE_PARAM_LOADER_CONVERSION( std::string, param, return std::string( param ); )

#endif // USCAUV_USCAUVCOMMON_PARAMLOADER
