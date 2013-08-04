/***************************************************************************
 *  include/uscauv_common/lookup_table.h
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


#ifndef USCAUV_USCAUVCOMMON_LOOKUPTABLE
#define USCAUV_USCAUVCOMMON_LOOKUPTABLE

// ROS
#include <ros/ros.h>

#include <uscauv_common/param_loader.h>

namespace uscauv
{

  template<class __KeyType, class __ValueType> 
    class LookupTable
  {
  public:
    std::vector<__KeyType> key_;
    std::vector<__ValueType> value_;

    LookupTable(){};
  LookupTable(std::vector<__KeyType> const & key,
	      std::vector<__ValueType> const & value)
    :
    key_(key),
      value_(value)
      {
	assert( key_.size() == value_.size() );
      }

    __ValueType lookupBinary(__KeyType const & key, double eps = 0.0) const
    {
      assert( key_.size() == value_.size() );
      assert( key_.size() );
    
      int left = 0;
      int right = value_.size() - 1;
      int middle;
    
      while ( right >= left )
	{
	  middle = ( left + right ) / 2;

	  __KeyType test_key = key_[middle];

	  if ( std::abs( test_key - key ) <= eps )
	    return value_[middle];
	  else if ( test_key > key )
	    right = middle - 1;
	  else if (test_key < key )
	    left = middle + 1;
	}

      /// At this point left and right are the same, so the choice of index is arbitrary
      ROS_WARN_STREAM( "Warning: Lookup for key [ " << key << " ] failed." );
      return value_[left];
    }

    __ValueType lookupClosestSlow(__KeyType const & key) const
    {
      assert( key_.size() == value_.size() );
      assert( key_.size() );
    
      double min_error;
      int ii = 0;
      int size = key_.size();
      int min_index = 0;
      min_error = std::abs( key_[ii] - key);
    
      for(; ii < size; ++ii)
	{

	  double error = std::abs( key_[ii] - key );
	  if (error < min_error )
	    {
	      min_error = error;
	      min_index = ii;
	    }
	}
    
      return value_[min_index];
    }

    int fromXmlRpc(XmlRpc::XmlRpcValue & xml_lookup, std::string const & key_name, std::string const & value_name)
    {
      
      try
	{
	  XmlRpc::XmlRpcValue xml_key   = uscauv::param::lookup<XmlRpc::XmlRpcValue>( xml_lookup, key_name );
	  XmlRpc::XmlRpcValue xml_value = uscauv::param::lookup<XmlRpc::XmlRpcValue>( xml_lookup, value_name );
	  int key_size = xml_key.size(), value_size = xml_value.size();

	  if( xml_key.size() != xml_value.size() )
	    {
	      ROS_WARN("Lookup table key size does not match value size (key: %d) != (value: %d )", key_size, value_size);
	      return -1;
	    }
      
	  key_ = uscauv::param::XmlRpcValueConverter<std::vector<double> >::convert( xml_key );
	  value_ = uscauv::param::XmlRpcValueConverter<std::vector<double> >::convert( xml_value );
	}
      catch( XmlRpc::XmlRpcException const & ex )
	{
	  ROS_WARN("Caught exception [ %s ] loading lookup table with key [ %s ], value [ %s ].",
		   ex.getMessage().c_str(), key_name.c_str(), value_name.c_str() );
	  return -1;
	}

      /// If key size isn't the same as value size at this point something has gone horribly wrong.
      /* for(int i = 0; i < key_size; ++i) */
      /* 	{ */
      /* 	  key_.push_back( xml_key[i] ); */
      /* 	  value_.push_back( xml_value[i] ); */
      /* 	} */
      return 0;
    }
  };
    
} // uscauv

#endif // USCAUV_USCAUVCOMMON_LOOKUPTABLE
