/***************************************************************************
 *  include/auv_physics/physics_simulator_node.h
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
#include <tf/transform_listener.h>

/// dynamics
#include <ode/ode.h>

/// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <auv_physics/PhysicsSimulatorConfig.h>

/// Simulation Commands
#include <auv_physics/SimulationInstruction.h>
#include <auv_physics/SimulationState.h>
#include <auv_physics/SimulationCommand.h>

/// AUV messages
#include <auv_msgs/MotorPower.h>
#include <auv_msgs/MotorPowerArray.h>

typedef auv_msgs::MotorPower _MotorPowerMsg;
typedef auv_msgs::MotorPowerArray _MotorPowerArrayMsg;

typedef auv_physics::SimulationInstruction _SimulationInstructionMsg;
typedef auv_physics::SimulationState _SimulationStateMsg;
typedef auv_physics::SimulationCommand _SimulationCommandSrv;



class ThrusterModel
{
  /// Data members
 public:
  dVector3 cm_to_thruster_;
  dVector3 thrust_dir_;
  
  double current_force_;
  
  LookupTable<double, double> power_to_force_;
  
 public:

 ThrusterModel() :
  current_force_( 0.0 )
    {}
  
  /**
   * Given an XmlRpcValue from the parameter server, populate the fields of the thruster model.
   *
   * @param xml_thruster The XmlRpcValue containing the thruster data
   *
   * @return Zero of successful, non-zero otherwise.
   */
  int fromXmlRpc(std::string const & name, XmlRpc::XmlRpcValue & xml_thruster)
  {
    if( power_to_force_.fromXmlRpc( xml_thruster, "power", "force" ) )
      {
	ROS_WARN( "Failed to load power-force map for thruster [ %s ].", name.c_str() );
	return -1;
      }

    tf::TransformListener tf_listener;
    tf::StampedTransform cm_to_thruster_tf;

    std::stringstream tf_name;
    tf_name << name << "_link";

    if( tf_listener.waitForTransform( "cm_link", tf_name.str(), ros::Time(0),
				      ros::Duration(5.0), ros::Duration(0.1)) )
      {
    
	/// Get the transform from the center of mass to the thruster
	try
	  {
	    tf_listener.lookupTransform( "cm_link", tf_name.str(), ros::Time(0), cm_to_thruster_tf );
	  }
	catch (tf::TransformException ex) 
	  {
	    ROS_ERROR( "%s", ex.what() );
	    return -1;
	  }
      }
    else
      {
	ROS_WARN( "Lookup of thruster [ %s ] transform failed.", name.c_str() );
	return -1;
      }

    /// Point where the thruster lies with respect to the auv's center of mass
    tf::Vector3 const & cm_to_thruster_vec = cm_to_thruster_tf.getOrigin();
    
    /// Unit vector in the direction of thrust, with respect to the auv's center of mass
    tf::Vector3 const thrust_dir_unit = cm_to_thruster_tf * tf::Vector3(1.0, 0.0, 0.0);
    
    cm_to_thruster_[0] = cm_to_thruster_vec.x();
    cm_to_thruster_[1] = cm_to_thruster_vec.y();
    cm_to_thruster_[2] = cm_to_thruster_vec.z();
    
    thrust_dir_[0] = thrust_dir_unit.x();
    thrust_dir_[1] = thrust_dir_unit.y();
    thrust_dir_[2] = thrust_dir_unit.z();

    return 0;
  }
  
  void updateForce(double const & power)
  {
    current_force_ = power_to_force_.lookupClosest( power );
  }
  
  void applyForce( dBodyID const & auv_body ) const
  {
    dBodyAddRelForceAtRelPos( auv_body, current_force_ * thrust_dir_[0], current_force_ * thrust_dir_[1],
			      current_force_  * thrust_dir_[2], cm_to_thruster_[0], 
			      cm_to_thruster_[1], cm_to_thruster_[2] );
    
  }

};

class AUVDynamicsModel
{
 public:
  double volume_;
  /// Incorporates mass at CM and inertial tensor
  dMass mass_;
  
  dVector3 cm_to_cv_;
  
  int fromXmlRpc(XmlRpc::XmlRpcValue & xml_model)
  {
    /// Get dynamics parameters (already retrieved from parameter server) ------------------------------------
    
    XmlRpc::XmlRpcValue & xml_tensor = xml_model["inertial_tensor"];

    volume_ = xml_model["volume"];
    
    /// Last six arguments are the non-redundant elements of the inertial tensor 
    dMassSetParameters( &mass_, xml_model["mass"],
			0.0, 0.0, 0.0,
			xml_tensor[0], xml_tensor[4], xml_tensor[8],
			xml_tensor[1], xml_tensor[2], xml_tensor[5] );
    
    /// Look up the transform to the center of volume ------------------------------------
    tf::TransformListener tf_listener;
    tf::StampedTransform cm_to_cv_tf;
    
    /// TODO: Verify that this gets the transform expressing 
    if( tf_listener.waitForTransform( "cm_link", "cv_link", ros::Time(0),
				      ros::Duration(5.0), ros::Duration(0.1)) )
      {
	try
	  {
	    tf_listener.lookupTransform( "cm_link", "cv_link" , ros::Time(0), cm_to_cv_tf );
	  }
	catch (tf::TransformException ex) 
	  {
	    ROS_ERROR( "%s", ex.what() );
	    return -1;
	  }
      }
    else
      {
	ROS_WARN( "Lookup of [cv_link] failed." );
	return -1;
      }

    tf::Vector3 const & cm_to_cv_vec = cm_to_cv_tf.getOrigin();
    
    cm_to_cv_[0] = cm_to_cv_vec.x();
    cm_to_cv_[1] = cm_to_cv_vec.y();
    cm_to_cv_[2] = cm_to_cv_vec.z();

    ROS_INFO("Loaded center-of-volume transform: ( %f, %f, %f )", cm_to_cv_[0], cm_to_cv_[1], cm_to_cv_[2] );
    
    return 0;
  }
  
};

class SimpleAUVPhysicsSimulatorNode 
{
 private:

  /// Publishers and subscribers
  ros::Subscriber motor_power_sub_;
  ros::Subscriber water_temp_sub_;
  tf::Transform transform_;

  /// Services
  ros::ServiceServer simulation_cmd_server_;
  
  /// tf
  tf::TransformBroadcaster pose_br_;

  /// Simulation data
  bool sim_running_;
  double loop_rate_hz_;
  ros::Time last_update_time_;
  std::string parent_frame_name_;
  
  /// Robot model
  std::map<std::string, ThrusterModel> thruster_models_;
  
  /// Parameters
  double gravity_;
  LookupTable<double, double> water_density_lookup_;
  double water_density_;
  bool force_neutral_buoyancy_;
  
  /// physics world
  double simulation_delta_;
  
  dWorldID auv_world_;
  dBodyID  auv_body_;
  
  AUVDynamicsModel auv_dynamics_;
  
  /// Constructor and destructor ------------------------------------
 public:
 SimpleAUVPhysicsSimulatorNode()
   :
  sim_running_( false ),
  force_neutral_buoyancy_( false )
    {
    }
  
  ~SimpleAUVPhysicsSimulatorNode()
    {
      /// destroy dynamics world. What happens if this is called without calling dWorldCreate()?
      dWorldDestroy( auv_world_ );
      dBodyDestroy( auv_body_ );
      
    } // Destructor
  
  /// Methods for flow control 
 public:

  /// Running spin() will cause this function to be called before the node begins looping the spingOnce() function.
  void spinFirst()
  {
    /// TODO: Figure out why this is called here. Maybe so that parameters are available?
    ros::spinOnce();
   
    /// Set up physics world ------------------------------------
    auv_world_ = dWorldCreate();
    auv_body_  = dBodyCreate(auv_world_);

    /// Get ROS ready ------------------------------------
    ros::NodeHandle nh_rel("~");
    
    getParameters();

    /// Subscribe to topics ------------------------------------
    water_temp_sub_ = nh_rel.subscribe("water_temp", 1, &SimpleAUVPhysicsSimulatorNode::waterTempCallback, this);
    motor_power_sub_ = nh_rel.subscribe("motor_power", 1, &SimpleAUVPhysicsSimulatorNode::motorPowerCallback, this);
    
    /// Begin service servers ------------------------------------
    simulation_cmd_server_ = nh_rel.advertiseService("simulation_cmd", &SimpleAUVPhysicsSimulatorNode::simulationCommandCallback, this);
        
    /// Print ODE info ------------------------------------
    ROS_INFO("Launching ODE simulation with parameters:");
    ROS_INFO("ERP: %f", dWorldGetERP(auv_world_) );
    ROS_INFO("CFM: %f", dWorldGetCFM(auv_world_) );
    ROS_INFO("AutoDisableFlag: %d", dWorldGetAutoDisableFlag(auv_world_) );
    ROS_INFO("AutoDisableLinearThreshold: %f", dWorldGetAutoDisableLinearThreshold(auv_world_) );
    ROS_INFO("AutoDisableAngularThreshold: %f", dWorldGetAutoDisableAngularThreshold(auv_world_) );
    ROS_INFO("AutoDisableSteps: %d", dWorldGetAutoDisableSteps(auv_world_) );
    ROS_INFO("AutoDisableTime: %f", dWorldGetAutoDisableTime(auv_world_) );
    
    ROS_INFO( "Finished spinning up." );
    return;
  }

  /// Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
  {
    /// Run physics simulation
    if ( sim_running_ )
      simulateAndPublish();
    
    return;
  }
  
  void spin()
  {
    /// nodehandle will resolve namespaces relative to this node's name
    ros::NodeHandle nh_rel("~");

    if( !nh_rel.getParam("loop_rate", loop_rate_hz_) )
      {
	ROS_WARN("Parameter [loop_rate] not found. Using default.");
	loop_rate_hz_ = 10.0;
      }

    ros::Rate loop_rate( loop_rate_hz_ );

    ROS_INFO( "Spinning up Physics Simulator..." );
    spinFirst();

    ROS_INFO( "Physics Simulator is spinning at %.2f Hz.", loop_rate_hz_ ); 

    simulation_delta_ = 1.0 / loop_rate_hz_;
    
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
	gravity_ = -9.8;
      }
    
    /// Set gravity in the z-direction
    dWorldSetGravity(auv_world_, 0.0, 0.0, gravity_);

    /// Get water density lookup ------------------------------------
    XmlRpc::XmlRpcValue wtd_map;
    if( !nh.getParam("environment/maps/water_temp_density", wtd_map) )
      {
	ROS_ERROR( "Couldn't find water density map." );
	ros::shutdown();
      }
    
    if( water_density_lookup_.fromXmlRpc( wtd_map, "temp", "density" ) )
      {
	ROS_ERROR( "Failed to build water-temperature-density map." );
	ros::shutdown();
	return;
      }
        
    /// get density at room temperature
    water_density_ = water_density_lookup_.lookupClosest( 20.0 );

    /// Get thruster models ------------------------------------
    XmlRpc::XmlRpcValue thrusters_xml;
    if (! nh.getParam( "model/thrusters", thrusters_xml ) )
      {
	ROS_WARN( "No thrusters parameter found. Thruster force will not be simulated." );
      }
    else
      {
	for(std::map<std::string, XmlRpc::XmlRpcValue>::iterator thruster_it = thrusters_xml.begin(); 
	    thruster_it != thrusters_xml.end(); ++thruster_it)
	  {
	    ThrusterModel thruster;
	
	    if( thruster.fromXmlRpc(thruster_it->first, thruster_it->second) )
	      {
		ROS_WARN( "Failed to load thruster model [ %s ].", thruster_it->first.c_str() );
		continue;
	      }
	    else
	      {
		ROS_INFO( "Loaded thruster model [ %s ].", thruster_it->first.c_str() );
	      }	
	
	    if ( thruster_models_.insert( std::pair<std::string, ThrusterModel>(thruster_it->first, thruster) ).second == false )
	      {
		ROS_WARN( "Thruster [ %s ] is already loaded. Ignoring...", thruster_it->first.c_str());
		continue;
	      }
	  }
    
	if( !thruster_models_.size() )
	  ROS_WARN( "No thruster models were loaded. Thruster force will not be simulated." );
      }

    XmlRpc::XmlRpcValue dynamics_xml;
    if (! nh.getParam( "model/dynamics", dynamics_xml ) )
      {
	ROS_ERROR( "Failed to load AUV dynamics model." );
	ros::shutdown();
	return;
      }
    
    auv_dynamics_.fromXmlRpc(dynamics_xml);
    
    dBodySetMass( auv_body_, &auv_dynamics_.mass_ );

    dMass test_mass;
    dBodyGetMass(auv_body_, &test_mass);
    
    dVector3 const & test_cm = test_mass.c;
    dMatrix3 const & test_it = test_mass.I;
    
    ROS_INFO("Loaded dynamics model with parameters:" );
    ROS_INFO("Mass: %f, Volume: %f", test_mass.mass, auv_dynamics_.volume_ );
    ROS_INFO("Center of Mass: ( %f, %f, %f )", test_cm[0], test_cm[1], test_cm[2]);
    ROS_INFO("Inertial Tensor: [ %f, %f, %f; %f, %f, %f; %f, %f, %f]",
	     test_it[0], test_it[1], test_it[2], 
	     test_it[4], test_it[5], test_it[6], 
	     test_it[8], test_it[9], test_it[10]);
    	     
    return;
  }

  /// Physics simulation implementation
 private:
  void simulateAndPublish()
  {
    /// timing
    ros::Time now = ros::Time::now();

    /// Apply forces on the body ------------------------------------
    simulateBuoyancy();
    simulateThrusters();
    
    /// Step simulation and publish the results ------------------------------------
    
    /**
     * Step the physics simulation. This is a little non-physical because it assumes that this function is called at exactly 1/loop_rate
     * Not using a fixed step size will cause instability in simulation
     */
    dWorldStep( auv_world_, simulation_delta_ );
    
    const dReal * world_to_auv_vec = dBodyGetPosition( auv_body_ );
    const dReal * world_to_auv_quat = dBodyGetQuaternion( auv_body_ );
    
    tf::Transform world_to_auv ( tf::Quaternion( world_to_auv_quat[1],
						 world_to_auv_quat[2],
						 world_to_auv_quat[3],
						 world_to_auv_quat[0]),

				 tf::Vector3( world_to_auv_vec[0],
					      world_to_auv_vec[1],
					      world_to_auv_vec[2]) );
    
    tf::StampedTransform world_to_auv_stamped( world_to_auv, now, parent_frame_name_, "simulated_pose" );

    pose_br_.sendTransform( world_to_auv_stamped );

    /// update our timekeeping
    last_update_time_ = now;

    return;
  }

  /// Add force opposing the gravity vector at the auv's volume centroid
  void simulateBuoyancy()
  {

    if( force_neutral_buoyancy_ )
      {
	dBodyAddForceAtRelPos( auv_body_, 0.0,
			       0.0, -gravity_ * auv_dynamics_.mass_.mass,
			       auv_dynamics_.cm_to_cv_[0], auv_dynamics_.cm_to_cv_[1], 
			       auv_dynamics_.cm_to_cv_[2] ); 
      }
    else
      {
	dBodyAddForceAtRelPos( auv_body_, 0.0,
			       0.0, -gravity_* water_density_ * auv_dynamics_.volume_,
			       auv_dynamics_.cm_to_cv_[0], auv_dynamics_.cm_to_cv_[1], 
			       auv_dynamics_.cm_to_cv_[2] ); 
      }

    return;
  }
  
  void simulateThrusters()
  {
    for(std::map<std::string, ThrusterModel>::iterator model_it = thruster_models_.begin();
	model_it != thruster_models_.end(); ++model_it )
      {
	model_it->second.applyForce( auv_body_ );
      }
    
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
    
    dQuaternion world_to_auv_quat;
    world_to_auv_quat[0] = q.w;
    world_to_auv_quat[1] = q.x;
    world_to_auv_quat[2] = q.y;
    world_to_auv_quat[3] = q.z;

    dBodySetPosition( auv_body_, p.x, p.y, p.z );
    dBodySetQuaternion( auv_body_, world_to_auv_quat );
    dBodySetLinearVel( auv_body_, t1.x, t1.y, t2.z );
    dBodySetAngularVel( auv_body_, t2.x, t2.y, t2.z );
    
    /// TODO: Integrate velocity over time elapsed since message was published
    
    /// update simulator state data
    last_update_time_ = now;
    sim_running_ = true;
    
    return true;
  }

  /// Message callbacks ------------------------------------
 public:

  void waterTempCallback(std_msgs::Float64::ConstPtr msg)
  {
    /// TODO: Step simulation before applying water density change. 
    
    /// Get the water density at this temperature
    water_density_ = water_density_lookup_.lookupClosest( msg->data );
    
    return;
  }

  void motorPowerCallback(_MotorPowerArrayMsg::ConstPtr msg)
  {
    /// TODO: Step simulation before applying new motor forces    
    if ( !sim_running_ )
      return;

    for(std::vector<_MotorPowerMsg>::const_iterator power_it = msg->motors.begin(); power_it != msg->motors.end(); ++power_it)
      {
    	std::map<std::string, ThrusterModel>::iterator model_it = thruster_models_.find( power_it->name ) ;
    	if ( model_it == thruster_models_.end() )
    	  {
    	    ROS_WARN( "Received thruster power, but no model is loaded. [ %s ]", power_it->name.c_str() );
    	    continue;
    	  }
	
    	model_it->second.updateForce( power_it->power );
      }
    
    return;
  }

  void reconfigureCallback(auv_physics::PhysicsSimulatorConfig & config, uint32_t level)
  {
    force_neutral_buoyancy_ = config.force_neutral_buoyancy;
  }

  /// Service callbacks ------------------------------------
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
	    const geometry_msgs::Point      & p = request.command.initial_pose.position;
	    const geometry_msgs::Quaternion & q = request.command.initial_pose.orientation;
 	    const geometry_msgs::Vector3    & t1 = request.command.initial_velocity.linear;
	    const geometry_msgs::Vector3    & t2 = request.command.initial_velocity.angular;
	    
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
