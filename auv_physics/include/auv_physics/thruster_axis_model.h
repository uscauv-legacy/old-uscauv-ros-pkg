/***************************************************************************
 *  include/auv_physics/thruster_axis_model.h
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


#ifndef USCAUV_AUVPHYSICS_THRUSTERAXISMODEL
#define USCAUV_AUVPHYSICS_THRUSTERAXISMODEL

// ROS
#include <ros/ros.h>

/// math
#include <tf_conversions/tf_eigen.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>

#include <uscauv_common/param_loader.h>
#include <uscauv_common/multi_reconfigure.h>
#include <uscauv_common/simple_math.h>
#include <uscauv_common/defaults.h>
#include <auv_msgs/MotorPowerArray.h>

#include <auv_physics/ThrusterModelConfig.h>

namespace uscauv
{

  class ThrusterModel
  {
    typedef XmlRpc::XmlRpcValue _XmlVal;
    /// Data members
  protected:
    tf::Vector3 cm_to_thruster_;
    tf::Vector3 thrust_dir_;
 
  public:

    ThrusterModel(){}
  
    virtual int load(std::string const & thruster_link, std::string const & cm_link = uscauv::defaults::CM_LINK)
    {
      tf::TransformListener tf_listener;
      tf::StampedTransform cm_to_thruster_tf;
      

      if( tf_listener.waitForTransform( cm_link, thruster_link, ros::Time(0),
					ros::Duration(5.0), ros::Duration(0.1)) )
	{
	  /// Get the transform from the center of mass to the thruster
	  try
	    {
	      tf_listener.lookupTransform( cm_link, thruster_link, ros::Time(0), cm_to_thruster_tf );
	    }
	  catch (tf::TransformException ex) 
	    {
	      ROS_ERROR( "%s", ex.what() );
	      return -1;
	    }
	}
      else
	{
	  ROS_WARN( "Lookup of thruster [ %s ] transform failed.", thruster_link.c_str() );
	  return -1;
	}

      /// Point where the thruster lies with respect to the auv's center of mass
      cm_to_thruster_ = cm_to_thruster_tf.getOrigin();
    
      /// Unit vector in the direction of thrust, with respect to the auv's center of mass
      /// All we are doing is applying a rotation matrix to the unit vector
      thrust_dir_ = cm_to_thruster_tf.getBasis() * tf::Vector3(1.0, 0.0, 0.0);

      ROS_INFO("Loaded thruster [ %s ] with direction ( %f, %f, %f ), location ( %f, %f, %f ),",
	       thruster_link.c_str(), thrust_dir_.getX(), thrust_dir_.getY(), thrust_dir_.getZ(),
	       cm_to_thruster_.getX(), cm_to_thruster_.getY(), cm_to_thruster_.getZ() );

      return 0;
    }

    virtual tf::Vector3 getThrustDir() const 
      {
	return thrust_dir_;
      }

    virtual tf::Vector3 getPosition() const 
      {
	return cm_to_thruster_;
      }

    virtual double applyConstraints(double const & value) const
    {
      return value;
    }
    
  };

  class ReconfigurableThrusterModel: public ThrusterModel
  {
  protected:
    auv_physics::ThrusterModelConfig config_;
    
  public:
    
    /// Same as base class, but direction can be inverted
    tf::Vector3 getThrustDir() const 
      {
	return thrust_dir_ * ( config_.invert ? -1 : 1 );
      }
   
    double applyConstraints(double const & value) const 
    {
      double total = value + config_.trim;
      if ( std::fabs(total) < config_.floor_mag ) return 0;
      if( config_.use_clamp )
	uscauv::clamp( total, config_.clamp_upper, config_.clamp_lower );
      return total;
    }

    bool getEnabled() const
    {
      return config_.enable;
    }
        
    void updateConfig( auv_physics::ThrusterModelConfig const & config)
    {
      config_ = config;
    }

  };
 
  template<class __ThrusterModel>
  class ThrusterAxisModel
  {
  protected:
    typedef XmlRpc::XmlRpcValue _XmlVal;
    typedef std::map<std::string, __ThrusterModel> _NamedThrusterMap;
  public:
    typedef Eigen::Matrix<double, 6, 1> AxisVector;
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1> ThrusterVector;
    
  protected:
    ros::NodeHandle nh_base_;

    _NamedThrusterMap all_thruster_models_;
    _NamedThrusterMap active_thruster_models_;
    
    Eigen::Matrix<double, 6, Eigen::Dynamic> thruster_to_axis_;
    
    std::string param_ns_;
    
  public:
  ThrusterAxisModel(std::string const & param_ns = "model/thrusters"): 
    param_ns_( param_ns )
      {}
    
    virtual void load(std::string const & tf_prefix = "robot/thrusters",
		      std::string const & cm_link = uscauv::defaults::CM_LINK)
    {
      loadModels( tf_prefix, cm_link );
      active_thruster_models_ = all_thruster_models_;
      computeThrusterAxisMatrix();
    }
    
    AxisVector ThrusterToAxis( ThrusterVector const & thruster_vals)
    {
      ROS_ASSERT( thruster_vals.rows() == thruster_to_axis_.cols() );

      return thruster_to_axis_ * thruster_vals;
    }
    
    /// Find a thruster combination to achieve the desired axis vals using least squares
    ThrusterVector AxisToThruster( AxisVector const & axis_vals)
    {
      ThrusterVector const thrust =  thruster_to_axis_.colPivHouseholderQr().solve(axis_vals);
      
     
      if( !(thruster_to_axis_*thrust).isApprox( axis_vals ))
	{
	  double const mse = ( thruster_to_axis_ * thrust - axis_vals ).norm() / axis_vals.norm();
	  ROS_ERROR("Requested axis values have no solution [ error %f ]!", mse );
	}
      return thrust;
    }
    
    /// This function implicitly assumes that thruster_models_ is sorted as it was when load() was called.
    auv_msgs::MotorPowerArray AxisToMotorArray( AxisVector const & axis_vals )
      {
	ThrusterVector const thruster_vals = AxisToThruster( axis_vals );

	auv_msgs::MotorPowerArray motors;
	
	int row_idx = 0;
	for(typename _NamedThrusterMap::const_iterator thruster_it = active_thruster_models_.begin();
	    thruster_it != active_thruster_models_.end(); ++thruster_it, ++row_idx)
	  {
	    auv_msgs::MotorPower mp;
	    mp.name = thruster_it->first;
	    mp.power = thruster_it->second.applyConstraints( thruster_vals(row_idx, 0) );
	    motors.motors.push_back(mp);
	  }
	return motors;
      }
    
    static AxisVector constructAxisVector(double const & x,  double const & y, 
					  double const & z,  double const & t1, 
					  double const & t2, double const & t3)
    {
      AxisVector vec;
      vec << x, y, z, t1, t2, t3;
      return vec;
    }

  protected:
    void loadModels(std::string const & tf_prefix,
		    std::string const & cm_link )
    {
      /// shutdown if we can't find the param
      _XmlVal base_node = uscauv::param::load<_XmlVal>(nh_base_, param_ns_);
      
      for(std::map<std::string, _XmlVal>::iterator thruster_it = base_node.begin(); 
	  thruster_it != base_node.end(); ++thruster_it)
	{
	  __ThrusterModel thruster;
	  std::string thruster_tf_name = tf_prefix + "/" + std::string(thruster_it->first);
	  
	  if( thruster.load(thruster_tf_name) )
	    {
	      ROS_WARN( "Failed to load thruster model [ %s ].", thruster_it->first.c_str() );
	      continue;
	    }
	  else
	    {
	      ROS_DEBUG( "Load thruster model [ %s ] success.", thruster_it->first.c_str() );
	    }	
	  
	  if ( all_thruster_models_.insert( std::pair<std::string, __ThrusterModel>(thruster_it->first, thruster) ).second == false )
	    {
	      ROS_WARN( "Thruster [ %s ] is already loaded. Ignoring...", thruster_it->first.c_str());
	      continue;
	    }
	}
      if( !all_thruster_models_.size() )
	{
	  ROS_FATAL( "No thruster models could be loaded." );
	  ros::shutdown();
	}
      return;
    }

    void computeThrusterAxisMatrix()
    {
      thruster_to_axis_.resize(6, active_thruster_models_.size() );

      int col_idx = 0;
      for(typename _NamedThrusterMap::const_iterator thruster_it = active_thruster_models_.begin();
	  thruster_it != active_thruster_models_.end(); ++thruster_it)
	{
	  AxisVector col;
	  Eigen::Vector3d thrust_dir_unit, cm_to_thruster, torque;
	  tf::vectorTFToEigen( thruster_it->second.getPosition(), cm_to_thruster );
	  tf::vectorTFToEigen( thruster_it->second.getThrustDir(), thrust_dir_unit );
	  
	  /// not actually torque unless thrust_dir_unit is some unit of force
	  /// could also be something like linear velocity -> angular velocity
	  torque = cm_to_thruster.cross(thrust_dir_unit);

	  col << thrust_dir_unit, torque;
	  ROS_DEBUG_STREAM("Thruster [ " << thruster_it->first << " ] (" << col.transpose() <<
			   ")");
	  
	  thruster_to_axis_.col( col_idx ) = col;
	  ++col_idx;
	}
      ROS_INFO_STREAM("Thruster to axis:" << std::endl << thruster_to_axis_);
    }

  };

  typedef ThrusterAxisModel<ThrusterModel> StaticThrusterAxisModel;
  
  class ReconfigurableThrusterAxisModel: 
  public ThrusterAxisModel<ReconfigurableThrusterModel>,
    public MultiReconfigure
  {
  protected:
    typedef auv_physics::ThrusterModelConfig _ThrusterModelConfig;
    
    bool ready_;
    
  public:
  ReconfigurableThrusterAxisModel(std::string const & param_ns = "model/thrusters"):
    ThrusterAxisModel<ReconfigurableThrusterModel>( param_ns ),
      MultiReconfigure( param_ns ), ready_( false ) {}
    
    virtual void load(std::string const & tf_prefix = "robot/thrusters",
		      std::string const & cm_link = uscauv::defaults::CM_LINK)
    {
      loadModels( tf_prefix, cm_link );

      for( typename _NamedThrusterMap::value_type const & thruster : all_thruster_models_ )
	{
	  addReconfigureServer<_ThrusterModelConfig>( thruster.first, std::bind( &ReconfigurableThrusterAxisModel::reconfigureCallback, this, std::placeholders::_1, thruster.first ) );
	}

      /**
       * addReconfigure calls reconfigurecallback every time a thruster is loaded, but we don't
       * want to populate the matrix until all thrusters have their servers loaded. This flag
       * indicates that all thrusters are loaded.
       */
      ready_ = true;

      updateActiveThrusters();
    }

  private:
    void reconfigureCallback( _ThrusterModelConfig const & config, std::string const & thruster_name)
    {
      typename _NamedThrusterMap::iterator thruster_it = all_thruster_models_.find( thruster_name );
      if ( thruster_it == all_thruster_models_.end() )
	{
	  ROS_WARN( "Received reconfigure request for a thruster model that is not loaded. Ignoring..." );
	  return;
	}

      thruster_it->second.updateConfig( config );

      if( !ready_ )
	return;
      
      updateActiveThrusters();
    }

    void updateActiveThrusters()
    {
      active_thruster_models_.clear();

      for( typename _NamedThrusterMap::value_type const & thruster : all_thruster_models_ )
	{
	  if( thruster.second.getEnabled() )
	    {
	      active_thruster_models_.insert( thruster );
	    }
	}
      
      computeThrusterAxisMatrix();
    }
    
  };
  
} // uscauv

#endif // USCAUV_AUVPHYSICS_THRUSTERAXISMODEL
