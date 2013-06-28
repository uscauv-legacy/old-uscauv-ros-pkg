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
#include <uscauv_common/defaults.h>

namespace uscauv
{

  class ThrusterModel
  {
    typedef XmlRpc::XmlRpcValue _XmlVal;
    /// Data members
  public:
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
      /// All we are doing is applying a rotation matrix to the unity vector
      thrust_dir_ = cm_to_thruster_tf.getBasis() * tf::Vector3(1.0, 0.0, 0.0);

      ROS_INFO("Loaded thruster [ %s ] with direction ( %f, %f, %f ), location ( %f, %f, %f ),",
	       thruster_link.c_str(), thrust_dir_.getX(), thrust_dir_.getY(), thrust_dir_.getZ(),
	       cm_to_thruster_.getX(), cm_to_thruster_.getY(), cm_to_thruster_.getZ() );

      return 0;
    }
  
  };


  class ThrusterAxisModel
  {
  private:
    typedef XmlRpc::XmlRpcValue _XmlVal;
    typedef std::map<std::string, ThrusterModel> _NamedThrusterMap;
  public:
    typedef Eigen::Matrix<double, 6, 1> AxisVector;
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1> ThrusterVector;
    
  private:
    ros::NodeHandle nh_base_;

    std::map<std::string, ThrusterModel> thruster_models_;

    Eigen::Matrix<double, 6, Eigen::Dynamic> thruster_to_axis_;
    
  public:
    ThrusterAxisModel()
      {}
    
    void load(std::string const & param_ns = "model/thrusters",
	      std::string const & tf_prefix = "robot/thrusters",
	      std::string const & cm_link = uscauv::defaults::CM_LINK)
    {
      /// shutdown if we can't find the param
      _XmlVal base_node = uscauv::loadParam<_XmlVal>(nh_base_, param_ns);

      for(std::map<std::string, _XmlVal>::iterator thruster_it = base_node.begin(); 
	  thruster_it != base_node.end(); ++thruster_it)
	{
	  ThrusterModel thruster;
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
	
	  if ( thruster_models_.insert( std::pair<std::string, ThrusterModel>(thruster_it->first, thruster) ).second == false )
	    {
	      ROS_WARN( "Thruster [ %s ] is already loaded. Ignoring...", thruster_it->first.c_str());
	      continue;
	    }
	}
      if( !thruster_models_.size() )
	{
	  ROS_FATAL( "No thruster models could be loaded." );
	  ros::shutdown();
	  return;
	}

      thruster_to_axis_.resize(6, thruster_models_.size() );

      int col_idx = 0;
      for(_NamedThrusterMap::const_iterator thruster_it = thruster_models_.begin();
	  thruster_it != thruster_models_.end(); ++thruster_it)
	{
	  AxisVector col;
	  Eigen::Vector3d thrust_dir_unit, cm_to_thruster, torque;
	  tf::vectorTFToEigen( thruster_it->second.cm_to_thruster_, cm_to_thruster );
	  tf::vectorTFToEigen( thruster_it->second.thrust_dir_, thrust_dir_unit );
	  
	  /// not actually torque unless thrust_dir_unit is some unit of force
	  /// could also be something like linear velocity -> angular velocity
	  torque = cm_to_thruster.cross(thrust_dir_unit);

	  col << thrust_dir_unit, torque;
	  ROS_DEBUG_STREAM("Thruster [ " << thruster_it->first << " ] (" << col.transpose() <<
			  ")");
	  
	  thruster_to_axis_.col( col_idx) = col;
	  ++col_idx;
	}
      ROS_INFO_STREAM("Thruster to axis:" << std::endl << thruster_to_axis_);

    }
    
    AxisVector ThrusterToAxis( ThrusterVector const & thruster_vals)
    {
      ROS_ASSERT( thruster_vals.rows() == thruster_to_axis_.cols() );

      return thruster_to_axis_ * thruster_vals;
    }
    
    /// Find a thruster combination to achieve the desired axis vals using least squares
    ThrusterVector AxisToThruster( AxisVector const & axis_vals)
    {
      return thruster_to_axis_.colPivHouseholderQr().solve(axis_vals);
    }

    static uscauv::ThrusterAxisModel::AxisVector constructAxisVector(double x, double y, double z,
							    double t1, double t2, double t3)
    {
      uscauv::ThrusterAxisModel::AxisVector vec;
      vec << x, y, z, t1, t2, t3;
      return vec;
    }

  };
  
} // uscauv

#endif // USCAUV_AUVPHYSICS_THRUSTERAXISMODEL
