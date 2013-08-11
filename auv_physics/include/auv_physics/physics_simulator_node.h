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
#include <geometry_msgs/Wrench.h>

/// tf
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

/// dynamics
#include <ode/ode.h>
#include <auv_physics/ode_conversions.h>

/// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <auv_physics/PhysicsSimulatorConfig.h>
#include <auv_physics/DragConfig.h>

/// Simulation Commands
#include <auv_physics/SimulationInstruction.h>
#include <auv_physics/SimulationState.h>
#include <auv_physics/SimulationCommand.h>

/// AUV messages
#include <auv_msgs/MotorPower.h>
#include <auv_msgs/MotorPowerArray.h>

#include <uscauv_common/multi_reconfigure.h>
#include <uscauv_common/param_loader.h>
#include <uscauv_common/lookup_table.h>
#include <uscauv_common/defaults.h>
#include <uscauv_common/transform_utils.h>
#include <uscauv_common/base_node.h>

typedef auv_msgs::MotorPower _MotorPowerMsg;
typedef auv_msgs::MotorPowerArray _MotorPowerArrayMsg;

typedef auv_physics::SimulationInstruction _SimulationInstructionMsg;
typedef auv_physics::SimulationState _SimulationStateMsg;
typedef auv_physics::SimulationCommand _SimulationCommandSrv;

typedef auv_physics::PhysicsSimulatorConfig _PhysicsSimulatorConfig;
typedef auv_physics::DragConfig _DragConfig;

typedef geometry_msgs::Wrench _WrenchMsg;

#define dDouble

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
    
    if( tf_listener.waitForTransform( uscauv::defaults::CM_LINK, uscauv::defaults::CV_LINK, ros::Time(0),
				      ros::Duration(5.0), ros::Duration(0.1)) )
      {
	try
	  {
	    tf_listener.lookupTransform( uscauv::defaults::CM_LINK, uscauv::defaults::CV_LINK , ros::Time(0), cm_to_cv_tf );
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

class PhysicsSimulatorNode: public BaseNode, public MultiReconfigure
{
 private:

  /// Publishers and subscribers
  ros::Subscriber water_temp_sub_;
  ros::Subscriber thruster_wrench_sub_;
  tf::Transform transform_;

  /// Services
  ros::ServiceServer simulation_cmd_server_;
  
  /// tf
  tf::TransformBroadcaster pose_br_;

  /// Simulation data
  bool sim_running_;
  double loop_rate_hz_;
  
  /// msg
  _WrenchMsg last_wrench_msg_;

  /// Parameters
  double gravity_;
  uscauv::LookupTable<double, double> water_density_lookup_;
  double water_density_;
  
  _PhysicsSimulatorConfig config_;
  _DragConfig * linear_drag_config_, * angular_drag_config_;
  
  
  /// physics world
  double simulation_delta_;
  
  dWorldID auv_world_;
  dBodyID  auv_body_;
  
  AUVDynamicsModel auv_dynamics_;
  
  /// Constructor and destructor ------------------------------------
 public:
 PhysicsSimulatorNode(): BaseNode("PhysicsSimulator"),
    sim_running_( false )
    {
    }
  
  ~PhysicsSimulatorNode()
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
    /// Set up physics world ------------------------------------
    auv_world_ = dWorldCreate();
    auv_body_  = dBodyCreate(auv_world_);

    dWorldSetCFM(auv_world_, 1e-3);
    dWorldSetERP(auv_world_, 0.8);

    /// Get ready ------------------------------------
    ros::NodeHandle nh_rel("~");
    
    addReconfigureServer<_PhysicsSimulatorConfig>("simulation", &PhysicsSimulatorNode::reconfigureCallback, this );
    addReconfigureServer<_DragConfig>("drag/linear"); 
    addReconfigureServer<_DragConfig>("drag/angular");
    linear_drag_config_ = &getLatestConfig<_DragConfig>("drag/linear");
    angular_drag_config_ = &getLatestConfig<_DragConfig>("drag/angular");
    
    simulation_delta_ = 1.0 / getLoopRate();
    
    getParameters();

    /// Subscribe to topics ------------------------------------
    water_temp_sub_ = nh_rel.subscribe("water_temp", 10, &PhysicsSimulatorNode::waterTempCallback, this);
    thruster_wrench_sub_ = nh_rel.subscribe("thruster_wrench", 10, &PhysicsSimulatorNode::thrusterWrenchCallback, this );
    
    /// Begin service servers ------------------------------------
    simulation_cmd_server_ = nh_rel.advertiseService("simulation_cmd", &PhysicsSimulatorNode::simulationCommandCallback, this);
        
    /// Print ODE info ------------------------------------
    ROS_INFO("Launching ODE simulation with parameters:");
    ROS_INFO("ERP: %f", dWorldGetERP(auv_world_) );
    ROS_INFO("CFM: %f", dWorldGetCFM(auv_world_) );
    ROS_INFO("AutoDisableFlag: %d", dWorldGetAutoDisableFlag(auv_world_) );
    ROS_INFO("AutoDisableLinearThreshold: %f", dWorldGetAutoDisableLinearThreshold(auv_world_) );
    ROS_INFO("AutoDisableAngularThreshold: %f", dWorldGetAutoDisableAngularThreshold(auv_world_) );
    ROS_INFO("AutoDisableSteps: %d", dWorldGetAutoDisableSteps(auv_world_) );
    ROS_INFO("AutoDisableTime: %f", dWorldGetAutoDisableTime(auv_world_) );

    if( config_.auto_start )
      autoStart();
    
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

    /// get water density lookup ------------------------------------
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
    water_density_ = water_density_lookup_.lookupClosestSlow( 20.0 );

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
    ros::Time now = ros::Time::now();

    /// Apply forces to the body ------------------------------------
    simulateBuoyancy();
    simulateThrusters();
    simulateDrag();
    
    /// Step simulation and publish the results ------------------------------------
    
    /**
     * Step the physics simulation. This is a little non-physical because it assumes that this function is called at exactly 1/loop_rate
     * Not using a fixed step size will cause instability in simulation
     */
    dWorldStep( auv_world_, simulation_delta_ );

    tf::Vector3 world_to_auv_vec; tf::Quaternion world_to_auv_quat;
    getBodyPose( world_to_auv_vec, world_to_auv_quat );

    tf::Transform world_to_auv ( world_to_auv_quat, world_to_auv_vec );
    
    if( !uscauv::isValid( world_to_auv ) )
      {
	ROS_ERROR("ODE exploded. Halting simulation...");
	stopSimulation();
	return;
      }

    tf::Transform world_to_imu( world_to_auv_quat, tf::Vector3(0, 0, 0) );
    
    tf::Transform world_to_depth( tf::Quaternion::getIdentity(), tf::Vector3(0, 0, world_to_auv_vec.getZ() ) );
    
    tf::StampedTransform world_to_auv_stamped( world_to_auv, now, uscauv::defaults::WORLD_LINK, "simulated_pose" );
    tf::StampedTransform world_to_imu_stamped( world_to_imu, now, uscauv::defaults::WORLD_LINK, uscauv::defaults::IMU_LINK );
    tf::StampedTransform world_to_depth_stamped( world_to_depth, now, uscauv::defaults::WORLD_LINK, uscauv::defaults::DEPTH_LINK );

    std::vector<tf::StampedTransform> outgoing_transforms; 
    outgoing_transforms.push_back( world_to_auv_stamped );
    outgoing_transforms.push_back( world_to_imu_stamped );
    outgoing_transforms.push_back( world_to_depth_stamped );

    pose_br_.sendTransform( outgoing_transforms );

    return;
  }

  /// Add force opposing the gravity vector at the auv's volume centroid
  void simulateBuoyancy()
  {

    if( config_.force_neutral_buoyancy )
      {
	/// Force of exactly the same magnitude as gravity in the opposite direction, but applied at center of volume
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
    /// apply force + torque relative to body's own frame at CM

    dBodyAddRelForce( auv_body_, last_wrench_msg_.force.x,
		      last_wrench_msg_.force.y, last_wrench_msg_.force.z );

    dBodyAddRelTorque( auv_body_, last_wrench_msg_.torque.x,
		       last_wrench_msg_.torque.y, last_wrench_msg_.torque.z );
    
  }

 /// Not physical at all. 
 void simulateDrag()
 {
   /// Velocities returned by ODE are expressed in world coordinates, so we must transform them into the auv's body coordinate system
   tf::Transform const auv_to_world = tf::Transform( uscauv::QuaternionODEToTF( dBodyGetQuaternion( auv_body_ ) ) ).inverse();
   
   tf::Vector3 const linear_vel  = auv_to_world * uscauv::Vector3ODEToTF( dBodyGetLinearVel( auv_body_ ) );
   tf::Vector3 const angular_vel = auv_to_world * uscauv::Vector3ODEToTF( dBodyGetAngularVel( auv_body_ ) );

   /// first part of drag eqn
   double const f = 0.5 * water_density_;

   /// vector in which each element is proportional to velocity^2, with the opposite sign of the original velocity vector
   tf::Vector3 linear_drag = -linear_vel*linear_vel.absolute() * f;
   tf::Vector3 angular_drag = -angular_vel*angular_vel.absolute() * f;
   
   
   dBodyAddRelForce( auv_body_, linear_drag.getX() * linear_drag_config_->x,
   		     linear_drag.getY() * linear_drag_config_->y, linear_drag.getZ() * linear_drag_config_->z );

   dBodyAddRelTorque( auv_body_, angular_drag.getX() * angular_drag_config_->x,
   		      angular_drag.getY() * angular_drag_config_->y, angular_drag.getZ() * angular_drag_config_->z );
 }
  

 private:
 void getBodyPose( tf::Vector3 & vec, tf::Quaternion & quat )
 {
   const dReal * world_to_auv_vec = dBodyGetPosition( auv_body_ );
   const dReal * world_to_auv_quat = dBodyGetQuaternion( auv_body_ );

   quat = tf::Quaternion( world_to_auv_quat[1],
			  world_to_auv_quat[2],
			  world_to_auv_quat[3],
			  world_to_auv_quat[0]);

   vec = tf::Vector3( world_to_auv_vec[0],
		      world_to_auv_vec[1],
		      world_to_auv_vec[2]);

 }

  bool stopSimulation()
  {
    sim_running_ = false;

    return true;
  }

  void clearState()
  {
    last_wrench_msg_ = _WrenchMsg();
  }

  void autoStart()
  {
    _SimulationCommandSrv::Request request;
    _SimulationCommandSrv::Response response;
    
    /// All fields are initialized to zero, so we just need to make the quaternion valid
    request.command.initial_pose.orientation.w = 1;
    request.command.type = _SimulationInstructionMsg::START;
    
    simulationCommandCallback( request, response );
  }

  bool startSimulation(_SimulationCommandSrv::Request & request)
  {
    const geometry_msgs::Point & p = request.command.initial_pose.position;
    const geometry_msgs::Quaternion & q = request.command.initial_pose.orientation;
    const geometry_msgs::Vector3 & t1 = request.command.initial_velocity.linear;
    const geometry_msgs::Vector3 & t2 = request.command.initial_velocity.angular;
    
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
    sim_running_ = true;

    clearState();

    return true;
  }

  /// Message callbacks ------------------------------------
 public:

  /// Note: The sub doesn't currently have temperature sensors
  void waterTempCallback(std_msgs::Float64::ConstPtr msg)
  {
    /// TODO: Step simulation before applying water density change. 
    
    /// Get the water density at this temperature
    water_density_ = water_density_lookup_.lookupClosestSlow( msg->data );
    
    return;
  }

  void thrusterWrenchCallback( _WrenchMsg::ConstPtr const & msg )
  {
    last_wrench_msg_ = *msg;
  }

  void reconfigureCallback(auv_physics::PhysicsSimulatorConfig const & config)
  {
    config_ = config;
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
