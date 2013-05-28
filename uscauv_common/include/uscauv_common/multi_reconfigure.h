/***************************************************************************
 *  include/uscauv_common/multi_reconfigure.h
 *  --------------------
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Dylan Foster
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


#ifndef USCAUV_USCAUVCOMMON_MULTIRECONFIGURE
#define USCAUV_USCAUVCOMMON_MULTIRECONFIGURE

// ROS
#include <ros/ros.h>

/// cpp11
#include <functional>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>

class BaseReconfigureStorage{};

template <class __ConfigType>
class ReconfigureStorage: public BaseReconfigureStorage
{
 public:
  typedef __ConfigType ConfigType;
  typedef dynamic_reconfigure::Server<__ConfigType> ServerType;
  typedef std::function< void( ConfigType const & ) > CallbackType;
  
 private:
  ServerType server_;
  CallbackType external_callback_;

 public:
  ConfigType config_;

 public:
  
 ReconfigureStorage( ros::NodeHandle const & nh ):
  server_( nh )
  {
    server_.setCallback( boost::bind( &ReconfigureStorage::internalCallback,
				      this, _1, _2 ) );
      
  }
  
  template <class... __BindArgs>
    ReconfigureStorage( ros::NodeHandle const & nh, __BindArgs... bind_args)
    : server_( nh )
  {
    external_callback_ = std::bind( std::forward<__BindArgs>(bind_args)...,
				    std::placeholders::_1 );
    
    server_.setCallback( boost::bind( &ReconfigureStorage::internalCallback,
				      this, _1, _2 ) );
      
  }

  /**
   * Note: server will call this once while it is being bound during the constructor
   * So config_ will be initialized to default __ConfigType values before it is available
   * externally.
   */
  void internalCallback( ConfigType const & config, uint32_t level )
  {
    config_ = config;
    
    if ( !external_callback_ ) return;
    
    try
      {
	external_callback_( config_ );
      }
    catch( std::exception const& ex )
      {
	ROS_WARN("External reconfigure callback failed [ %s ]", ex.what() );
	return;
      }
    catch(...)
      {
	ROS_WARN("External reconfigure callback failed [ unknown exception ]");
	return;
      }
  }
  
};

class MultiReconfigure
{
 private:
  typedef std::shared_ptr< BaseReconfigureStorage > _StoragePointerType;
  typedef std::map< std::string, _StoragePointerType > _NamedRCStorageMap;

  ros::NodeHandle nh_rel_;
  
  _NamedRCStorageMap reconfigure_storage_;

 public:
 MultiReconfigure():
  nh_rel_( "~" )
    {}
  
  template<class __ConfigType>
    void addReconfigureServer( std::string const & ns)
    {
      ros::NodeHandle nh_rcs( nh_rel_, ns );
      std::string const & ns_rcs = nh_rcs.getNamespace();
      
      ROS_INFO("Creating reconfigure server [ %s ]...", ns_rcs.c_str() );

      _NamedRCStorageMap::iterator rcs_it = reconfigure_storage_.find( ns_rcs );
      
      if ( rcs_it != reconfigure_storage_.end() )
	{
	  ROS_WARN("Add reconfigure server [ %s ] requested, but this server already exists. Overwriting...", ns_rcs.c_str() );
	  
	  rcs_it->second.reset();
	}
      
      reconfigure_storage_[ ns_rcs ] = 
	std::make_shared< ReconfigureStorage<__ConfigType> >( nh_rcs );


      ROS_INFO("Created reconfigure server successfully.");
      return;
    }

  template<class __ConfigType, class... __BindArgs>
    void addReconfigureServer( std::string const & ns, __BindArgs... bind_args)
    {
      ros::NodeHandle nh_rcs( nh_rel_, ns );
      std::string const & ns_rcs = nh_rcs.getNamespace();
      
      ROS_INFO("Creating reconfigure server [ %s ]...", ns_rcs.c_str() );

      _NamedRCStorageMap::iterator rcs_it = reconfigure_storage_.find( ns_rcs );
      
      if ( rcs_it != reconfigure_storage_.end() )
	{
	  ROS_WARN("Add reconfigure server [ %s ] requested, but this server already exists. Overwriting...", ns_rcs.c_str() );
	  
	  rcs_it->second.reset();
	}
      
      reconfigure_storage_[ ns_rcs ] = 
	std::make_shared< ReconfigureStorage<__ConfigType> >
	( nh_rcs, std::forward<__BindArgs>(bind_args)... );

      ROS_INFO("Created reconfigure server successfully.");
      return;
    }

  template<class __ConfigType>
    __ConfigType & getLatestConfig( std::string const & ns )
    {
      std::string const & ns_rcs = nh_rel_.resolveName( ns, true);
      
      _NamedRCStorageMap::iterator rcs_it = reconfigure_storage_.find( ns_rcs );
      
      if ( rcs_it == reconfigure_storage_.end() )
	{
	  ROS_WARN("Requested reconfigure server [ %s ] does not exist.", ns_rcs.c_str() );
	  
	  /// TODO: Figure out what the default config is and how to return it.
	  /* return; */
	}
      
      std::shared_ptr< ReconfigureStorage<__ConfigType> > rc_storage =
	std::static_pointer_cast< ReconfigureStorage<__ConfigType> >
	( rcs_it->second );
      
      return rc_storage->config_;

    }

};

#endif // USCAUV_USCAUVCOMMON_MULTIRECONFIGURE
