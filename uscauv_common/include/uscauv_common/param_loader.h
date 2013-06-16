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

namespace uscauv
{
  typedef XmlRpc::XmlRpcValue XmlRpcValue;

  // ################################################################
  // ################################################################

  template<typename __ParamType> 
    static typename std::enable_if< !std::is_arithmetic<__ParamType>::value, __ParamType>::type
    fromXmlRpcValue(XmlRpcValue & param);
  
  template <typename __ParamType>
    static typename std::enable_if< std::is_arithmetic<__ParamType>::value, __ParamType>::type
    fromXmlRpcValue(XmlRpcValue & param)
    {
      if( param.getType() == XmlRpcValue::TypeBoolean ) return bool(param);
      else if ( param.getType() == XmlRpcValue::TypeInt ) return int(param);
      else if ( param.getType() == XmlRpcValue::TypeDouble ) return double(param);
      else return __ParamType(); /// TODO: Assert or something - this shouldn't happen
    }

  template<>
    std::string fromXmlRpcValue<std::string>(XmlRpcValue & param)
    {
      return std::string(param);
    }

  template<>
    XmlRpcValue fromXmlRpcValue<XmlRpcValue>(XmlRpcValue & param)
    {
      return param;
    }
  
  // ################################################################
  // ################################################################

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

  template <typename __ParamType>
    static __ParamType loadParam( ros::NodeHandle const & nh, std::string const & name, __ParamType default_value)
    {
      XmlRpcValue value;
      std::string const & resolved_name = nh.resolveName(name, true);
      bool const param_found = nh.getParam( name, value );

      if( param_found )
	{
	  __ParamType return_value = fromXmlRpcValue<__ParamType>(value);
	  
	  ROS_INFO("Loaded param [ %s ] with value [ %s ].", 
		   resolved_name.c_str(), toString(return_value).c_str());
	  
	  return return_value;
	}
      else
	{
	  ROS_WARN("Failed to load param [ %s ]. Using default [ %s ].",
		   resolved_name.c_str(), toString(default_value).c_str());

	  return default_value;
	}
    }

  template <typename __ParamType>
    __ParamType loadParam( ros::NodeHandle const & nh, std::string const & name)
    {
      XmlRpcValue value;
      std::string const & resolved_name = nh.resolveName(name, true);
      bool const param_found = nh.getParam( name, value );

      if( param_found )
	{
	  __ParamType return_value = fromXmlRpcValue<__ParamType>(value);
	  
	  ROS_INFO("Loaded param [ %s ] with value [ %s ].", 
		   resolved_name.c_str(), toString(return_value).c_str());
	  
	  return return_value;
	}
      else
	{
	  ROS_FATAL("Failed to load param [ %s ].  Shutting down...",
		   resolved_name.c_str());
	  ros::shutdown();
	  return __ParamType();
	}
    }

} // uscauv

#endif // USCAUV_USCAUVCOMMON_PARAMLOADER
