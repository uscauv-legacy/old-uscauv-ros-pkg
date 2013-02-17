/***************************************************************************
 *  include/physics_simulator/physics_simulator_node.h
 *  --------------------
 *
 *  Copyright (c) 2012, Dylan Foster, Francesca Nannizzi
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

/// follows team -> package -> source file convention
#ifndef USCAUV_AUVPHYSICS_PHYSICSSIMULATORNODE_H
#define USCAUV_AUVPHYSICS_PHYSICSSIMULATORNODE_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>

/// tf
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>

/// kinematics and dynamics library
#include <kdl/frames.hpp>

/// Simulation Commands
#include <auv_physics/SimulationInstruction.h>
#include <auv_physics/SimulationState.h>
#include <auv_physics/SimulationCommand.h>

typedef KDL::Vector _Vector;
typedef KDL::Rotation _Rotation;
typedef KDL::Frame _Frame;
typedef KDL::Twist _Twist;
typedef KDL::Wrench _Wrench;


typedef auv_physics::SimulationInstruction _SimulationInstructionMsg;
typedef auv_physics::SimulationState _SimulationStateMsg;
typedef auv_physics::SimulationCommand _SimulationCommandSrv;

template<class __KeyType, class __ValueType>
struct LookupTable
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

  __ValueType lookupBinary(__KeyType const & key, double eps = 0.0)
  {
    assert( key_.size() == value_.size() );
    
    int left = 0;
    int right = value_.size() - 1;
    int middle;
    
    while ( right >= left )
      {
	middle = ( left + right ) / 2;

        __KeyType test_key = key_[middle];

        if ( std::abs( test_key - key ) < eps )
          return value_[middle];
        else if ( test_key > key )
          right = middle - 1;
        else if (test_key < key )
          left = middle + 1;
      }

    /// At this point left and right are the same, so the choice of index is arbitrary
    ROS_WARN_STREAM( "Warning: Lookup for [ " << key << " ] failed." );
    return value_[left];
  }

  __ValueType lookupClosest(__KeyType const & key)
  {
    assert( key_.size() == value_.size() );
    
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
};

class SimpleAUVPhysicsSimulatorNode 
{
 private:

  /// Publishers and subscribers
  ros::Subscriber motor_cmd_sub_;
  ros::Subscriber water_temp_sub_;
  tf::Transform transform_;

  /// Services
  ros::ServiceServer simulation_cmd_server_;
  
  /// tf
  tf::TransformBroadcaster pose_br_;

  /// Simulation data
  bool sim_running_;
  ros::Time last_update_time_;
  std::string parent_frame_name_;
  
  /// Robot model
  
  
  /// Robot state
  _Frame  current_pose_;
  _Twist  current_velocity_;
  _Wrench current_force_;
  
  /// Parameters
  double gravity_;
  LookupTable<double, double> water_density_lookup_;
  double water_density_;
  

  /// Constructor and destructor ------------------------------------
 public:
 SimpleAUVPhysicsSimulatorNode()
   :
  sim_running_( false ),
    current_pose_ ( _Frame::Identity() ),
    current_velocity_( _Twist::Zero() ),
    current_force_( _Wrench::Zero() )
      {
      };   

  ~SimpleAUVPhysicsSimulatorNode(){}; // Destructor
    
  /// Methods for flow control 
 public:

  /// Running spin() will cause this function to be called before the node begins looping the spingOnce() function.
  void spinFirst()
  {
    ros::NodeHandle nh_rel("~");
    
    getParameters();

    /// Subscribe to topics ------------------------------------
    water_temp_sub_ = nh_rel.subscribe("water_temp", 1, &SimpleAUVPhysicsSimulatorNode::waterTempCallback, this);
    
    /// Begin service servers ------------------------------------
    simulation_cmd_server_ = nh_rel.advertiseService("simulation_cmd", &SimpleAUVPhysicsSimulatorNode::simulationCommandCallback, this);

    ROS_INFO( "Finished spinning up." );
    return;
  }

  /// Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
  {
   
    /* broadcaster_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "world", "physics_simulation_pose")); */
    
    /// Run physics simulation
    if ( sim_running_ )
      simulateAndPublish();
    
    return;
  }
  
  void spin()
  {
    /// nodehandle will resolve namespaces relative to this node's name
    ros::NodeHandle nh_rel("~");

    double loop_rate_hz;
    
    if( !nh_rel.getParam("loop_rate", loop_rate_hz) )
      {
	ROS_WARN("Parameter [loop_rate] not found. Using default.");
	loop_rate_hz = 10.0;
      }

    ros::Rate loop_rate( loop_rate_hz );

    ROS_INFO( "Spinning up Physics Simulator..." );
    spinFirst();

    ROS_INFO( "Physics Simulator is spinning at %.2f Hz.", loop_rate_hz ); 
    
    while( ros::ok() )
      {
	spinOnce();
	ros::spinOnce();
	loop_rate.sleep();
      }
    
    return;
  }
  
  /// Parameters
 private:
  void getParameters()
  {
    ros::NodeHandle nh;
    
    /// Get gravity ------------------------------------
    if (! nh.getParam( "environment/constants/gravity", gravity_ ) )
      {
	ROS_WARN( "Parameter [gravity] not found. Using default.");
	gravity_ = 9.8;
      }
    
    /// Get water density lookup ------------------------------------
    XmlRpc::XmlRpcValue wtd_map;
    if( !nh.getParam("environment/maps/water_temp_density", wtd_map) )
      {
	ROS_ERROR( "Couldn't find water density map." );
	ros::shutdown();
      }
    
    /// initialize our lookup table with the ugliest syntax possible (thanks Cinco!)
    std::vector<double> temperature, density;
    XmlRpc::XmlRpcValue & xml_temp = wtd_map["temp"];
    XmlRpc::XmlRpcValue & xml_density = wtd_map["density"];

    ROS_ASSERT( xml_temp.size() == xml_density.size() );
    
    for(int i = 0; i < xml_temp.size(); ++i)
      {
	water_density_lookup_.key_.push_back(xml_temp[i]);
	water_density_lookup_.value_.push_back(xml_density[i]);
      }
        
    /// get density at room temperature
    water_density_ = water_density_lookup_.lookupClosest( 20.0 );
    
    return;
  }


  /// Physics simulation implementation
 private:
  void simulateAndPublish()
  {
    /// timing
    ros::Time now = ros::Time::now();
    double dt = (now - last_update_time_).toSec();
    

    /// Integrate velocity
    current_pose_.Integrate( current_velocity_, 1.0/dt );

    /// TODO: Integrate acceleration here.
    
    
    /// TODO: Apply forces to get new acceleration and velocity here.
    
    /// Publish current odometry estimate ------------------------------------
    
    /// Have to convert our KDL frame representation of pose to a tf stamped transform
    tf::Transform pose_tf;
    tf::poseKDLToTF(current_pose_, pose_tf);

    tf::StampedTransform pose_stamped_tf( pose_tf, now, parent_frame_name_, "simulated_pose" );

    pose_br_.sendTransform( pose_stamped_tf );
    
    /// update our timekeeping
    last_update_time_ = now;

    ROS_INFO("dt was %f", dt);
    
    return;
  }

  /// Control functions
 private:
  bool stopSimulation()
  {
    sim_running_ = false;

    return true;
  }

  bool startSimulation(_SimulationCommandSrv::Request & request)
  {
    ros::Time now = ros::Time::now();

    const geometry_msgs::Point & p = request.command.initial_pose.position;
    const geometry_msgs::Quaternion & q = request.command.initial_pose.orientation;
    const geometry_msgs::Vector3 & t1 = request.command.initial_velocity.linear;
    const geometry_msgs::Vector3 & t2 = request.command.initial_velocity.angular;
    
    parent_frame_name_ = request.command.header.frame_id;
    
    /// Convert from geometry_msgs to KDL
    current_pose_     = _Frame( _Rotation::Quaternion(q.x, q.y, q.z, q.w), _Vector(p.x, p.y, p.z) );
    current_velocity_ = _Twist( _Vector(t1.x, t1.y, t1.z), _Vector(t2.x, t2.y, t2.z) );

    /// Integrate velocity over the time between now and timestamp in request
    double dt = (now - request.command.header.stamp).toSec();
    
    /// Second argument is sample frequency for 1st order integration approximation
    current_pose_.Integrate(current_velocity_, 1.0/dt);
    
    /// update simulator state data
    last_update_time_ = now;
    sim_running_ = true;
    
    return true;
  }

  /// Message callbacks
 public:

  void waterTempCallback(std_msgs::Float64::ConstPtr msg)
  {
    /// Get the water density at this temperature
    water_density_ = water_density_lookup_.lookupClosest( msg->data );
    
    /// Simulate
    if ( sim_running_ )
      simulateAndPublish();
  }
  
  /// Service callbacks
 public:
  bool simulationCommandCallback(_SimulationCommandSrv::Request & request, _SimulationCommandSrv::Response & response)
  {
    /// Print info
    if( request.command.type == _SimulationInstructionMsg::START )
      {
	if ( sim_running_ )
	  {
	    ROS_INFO( "Received a start request, but physics simulation is already running." );
	  }
	else
	  {
	    const geometry_msgs::Point & p = request.command.initial_pose.position;
	    const geometry_msgs::Quaternion & q = request.command.initial_pose.orientation;
 	    const geometry_msgs::Vector3 & t1 = request.command.initial_velocity.linear;
	    const geometry_msgs::Vector3 & t2 = request.command.initial_velocity.angular;
	    
	    ROS_INFO( "Starting physics simulation..." );
	    ROS_INFO( "Using pose with position: (%.2f, %.2f, %.2f), rotation: (%.2f, %.2f, %.2f, %.2f)",
		      p.x, p.y, p.z, q.x, q.y, q.z, q.w );
	    ROS_INFO( "Using velocity with linear: (%.2f, %.2f, %.2f), angular: (%.2f, %.2f, %.2f)",
		      t1.x, t1.y, t1.z, t2.x, t2.y, t2.z);
	    ROS_INFO( "Using parent frame [ %s ]", request.command.header.frame_id.c_str() );
	    
	    if( startSimulation( request ) )
	      {
		ROS_INFO( "Started physics simulation successfully." );
	      }
	    else
	      {
		ROS_WARN( "Failed to start physics simulation." );
		return false;
	      }
	  }
      }
    else if( request.command.type == _SimulationInstructionMsg::RESTART )
      {
	
	const geometry_msgs::Point & p = request.command.initial_pose.position;
	const geometry_msgs::Quaternion & q = request.command.initial_pose.orientation;
	const geometry_msgs::Vector3 & t1 = request.command.initial_velocity.linear;
	const geometry_msgs::Vector3 & t2 = request.command.initial_velocity.angular;
	

	if ( !sim_running_ )
	  {
	    ROS_INFO( "Received a stop request, but physics simulation is already stopped." );
	  }
	else
	  {
	    ROS_INFO( "Halting physics simulation..." );
	    
	    if( stopSimulation() )
	      {
		ROS_INFO( "Halted physics simulation successfully." );
	      }
	    else
	      {
		ROS_WARN( "Failed to halt physics simulation." );
		return false;
	      }
	  }
	
	ROS_INFO( "Using pose with position: (%.2f, %.2f, %.2f), rotation: (%.2f, %.2f, %.2f, %.2f)",
		  p.x, p.y, p.z, q.x, q.y, q.z, q.w );
	ROS_INFO( "Using velocity with linear: (%.2f, %.2f, %.2f), angular: (%.2f, %.2f, %.2f)",
		  t1.x, t1.y, t1.z, t2.x, t2.y, t2.z);
	ROS_INFO( "Using parent frame [ %s ]", request.command.header.frame_id.c_str() );

	ROS_INFO( "Restarting simulation..." );
	if( startSimulation( request ) )
	  {
	    ROS_INFO( "Restarted physics simulation successfully." );
	  }
	else
	  {
	    ROS_WARN( "Failed to halt physics simulation." );
	    return false;
	  }
	
      }
    else if( request.command.type == _SimulationInstructionMsg::STOP )
      {
	if ( !sim_running_ )
	  {
	    ROS_INFO( "Received a stop request, but physics simulation is already stopped." );
	  }
	else
	  {
	    ROS_INFO( "Stopping physics simulation..." );
	
	    if( stopSimulation() )
	      {
		ROS_INFO( "Stopped physics simulation successfully." );
	      }
	    else
	      {
		ROS_WARN( "Failed to stop physics simulation." );
		return false;
	      }
	  }
      }

    /// Have to cast these enums to ints to avoid compiler warnings. This is stupid
    response.state.state = ( sim_running_ ) ? int(_SimulationStateMsg::RUNNING) : int(_SimulationStateMsg::STOPPED);

    return true;
  }

};

#endif //USCAUV_AUVPHYSICS_PHYSICSSIMULATORNODE_H
