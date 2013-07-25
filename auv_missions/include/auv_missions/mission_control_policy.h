/***************************************************************************
 *  include/auv_missions/mission_control_policy.h
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


#ifndef USCAUV_AUVMISSIONS_MISSIONCONTROLPOLICY
#define USCAUV_AUVMISSIONS_MISSIONCONTROLPOLICY

// ROS
#include <ros/ros.h>

#include <tf/LinearMath/Transform.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

/// cpp11
#include <functional>
#include <thread>

#include <boost/bind/protect.hpp>

/// seabee
#include <seabee3_msgs/Depth.h>
#include <seabee3_msgs/KillSwitch.h>

/// uscauv
#include <uscauv_common/param_loader.h>
#include <uscauv_common/multi_reconfigure.h>
#include <uscauv_common/action_token.h>
#include <uscauv_common/simple_math.h>

#include <auv_msgs/MaskedTwist.h>
#include <auv_missions/MissionControlConfig.h>

namespace uscauv
{

  class MissionControlPolicy
  {
  private:
    typedef seabee3_msgs::Depth _DepthMsg;
    typedef seabee3_msgs::KillSwitch _KillswitchMsg;
    typedef auv_msgs::MaskedTwist _MaskedTwistMsg;

    typedef auv_missions::MissionControlConfig _MissionControlConfig;
     
  private:
   
    /// ROS
    ros::Subscriber depth_sub_, killswitch_sub_;
    ros::NodeHandle nh_rel_;
    tf::TransformBroadcaster control_tf_broadcaster_;
    /// TODO: Make sure it's alright to use this from multiple threads without synchronizing. 
    /// Should be alright since lookupTransform and canTransform are const
    tf::TransformListener tf_listener_;

    ros::Publisher axis_command_pub_;
     
    /// Sensing
    _DepthMsg::ConstPtr last_depth_msg_;
    _KillswitchMsg::ConstPtr last_killswitch_msg_;
    tf::Transform world_to_desired_tf_, world_to_measurement_tf_;

    /// threading
    std::mutex last_depth_msg_mutex_;
    std::mutex last_killswitch_msg_mutex_;
    std::mutex control_tf_mutex_;

    std::mutex killswitch_enabled_mutex_;
    std::condition_variable killswitch_enabled_condition_;
    
    std::mutex killswitch_disabled_mutex_;
    std::condition_variable killswitch_disabled_condition_;

    /// actions
    std::vector< quickdev::SimpleActionToken > active_tokens_;

    /// other
    double action_loop_rate_hz_;
    double control_loop_rate_hz_;
    bool auto_start_;
     
  public:
  MissionControlPolicy(): nh_rel_( "~" ), 
      world_to_desired_tf_( tf::Transform::getIdentity() ),
      world_to_measurement_tf_( tf::Transform::getIdentity() )
	{}
     
    /// The program will exit when this thread returns
    template<class... __BoundArgs>
      void startMissionControl( __BoundArgs&&... bound_args )
      {
	ros::NodeHandle nh;

	action_loop_rate_hz_ = uscauv::param::load<double>( nh_rel_, "action_loop_rate", 60 );
	control_loop_rate_hz_ = uscauv::param::load<double>( nh_rel_, "control_loop_rate", 60 );

	/**
	 * Note: Killswitch condition logic gets a little messed up when this param is used. 
	 * Don't set while using the real robot
	 */
	auto_start_ = uscauv::param::load<bool>( nh_rel_, "auto_start", false );
	
	/// Start ros interfaces
	depth_sub_ = nh.subscribe("robot/sensors/depth", 10,
				  &MissionControlPolicy::depthCallback, this );

	killswitch_sub_ = nh.subscribe("robot/sensors/kill_switch", 10,
				       &MissionControlPolicy::killswitchCallback, this );

	axis_command_pub_ = nh.advertise<_MaskedTwistMsg>( "control_server/axis_cmd", 10 );

	/// Start external main loop thread
	boost::thread main_loop_thread( &MissionControlPolicy::missionControlThread, this, boost::protect( boost::bind( std::forward<__BoundArgs>( bound_args )...) ) );
	main_loop_thread.detach();

	boost::thread control_loop_thread( &MissionControlPolicy::updateTransformsThread, this );
	control_loop_thread.detach();
	
	return;
      }
    
    // ################################################################
    // Main API #######################################################
    // ################################################################

  public:
    
    /// Assumes positive Z axis pointing toward the bottom of the pool
    quickdev::SimpleActionToken diveTo( double const & depth )
      {
	quickdev::SimpleActionToken token;
	token.start( &MissionControlPolicy::diveTo_impl, this, token, depth );
	active_tokens_.push_back( token );
	return token;
      }
    
    quickdev::SimpleActionToken faceTo( double const & heading )
      {
	quickdev::SimpleActionToken token;
	token.start( &MissionControlPolicy::faceTo_impl, this, token, heading );
	active_tokens_.push_back( token );
	return token;
      }

    quickdev::SimpleActionToken zeroPitchRoll()
      {
	quickdev::SimpleActionToken token;
	token.start( &MissionControlPolicy::zeroPitchRoll_impl, this, token );	
	active_tokens_.push_back( token );
	return token;
      }

    quickdev::SimpleActionToken maintainHeading()
      {
	quickdev::SimpleActionToken token;
	token.start( &MissionControlPolicy::maintainHeading_impl, this, token );
	active_tokens_.push_back( token );	
	return token;
      }

    /// Move in the direction of a unit vector expressed relative to the robot's CM frame
    quickdev::SimpleActionToken moveToward( double const & x, double const & y, double const & scale = 1 )
      {
	quickdev::SimpleActionToken token;
	token.start( &MissionControlPolicy::moveToward_impl, this, token, x, y, scale );
	active_tokens_.push_back( token );	
	return token;	
      }

    // ################################################################
    // Main API function implementation ###############################
    // ################################################################

  private:
    
    void diveTo_impl( quickdev::SimpleActionToken token, double const & depth )
    {
      ros::Rate loop_rate( action_loop_rate_hz_ );

      while( ros::ok() && token() )
	{
	  
	  /// Lock the depth message
	  {
	    std::unique_lock< std::mutex > depth_lock( last_depth_msg_mutex_ );

	    if( !last_depth_msg_ ) continue;
	    

	    /// Lock the transforms
	    {
	      std::unique_lock<std::mutex> tf_lock( control_tf_mutex_ );

	      /// Depth outside this function is expressed with Z axis pointing out of pool
	      world_to_desired_tf_.getOrigin().setZ( -1.0 * depth );
	      world_to_measurement_tf_.getOrigin().setZ( last_depth_msg_->value );
	      
	      /// TODO: Change depth delta to something that's not a guess
	      if( std::abs( world_to_desired_tf_.getOrigin().getZ() -
			    world_to_measurement_tf_.getOrigin().getZ() ) < 0.05 && !token.success() )
		{
		  ROS_INFO("Reached target depth [ %f ]. ", depth );
		  token.succeed();		  
		}
	    }
	    
	  }
	  
	  loop_rate.sleep();
	}
      
      /// Cancel depth trajectory
      {
	std::unique_lock<std::mutex> lock( control_tf_mutex_ );
	world_to_desired_tf_.getOrigin().setZ( 0 );
	world_to_measurement_tf_.getOrigin().setZ( 0 );
      }
      
      token.complete();
    }
    
    void faceTo_impl( quickdev::SimpleActionToken token, double const & heading )
    {
      ros::Rate loop_rate( action_loop_rate_hz_ );
      
      double const normalized_heading = uscauv::algebraic_mod( heading, uscauv::TWO_PI );

      while( ros::ok() && token() )
	{
	  tf::StampedTransform world_to_imu_tf;

	  if( getTransform( "robot/sensors/imu", world_to_imu_tf ) )
	    continue;

	  double roll, pitch, yaw;
	  world_to_imu_tf.getBasis().getRPY( roll, pitch, yaw );
	  
	  {
	    std::unique_lock<std::mutex> lock( control_tf_mutex_ );
	    
	    setTransformYaw( world_to_desired_tf_, normalized_heading );
	    setTransformYaw( world_to_measurement_tf_, yaw );
	    
	    /// TODO: criteria from param
	    if( uscauv::ring_distance( yaw, normalized_heading, uscauv::TWO_PI ) < uscauv::PI / 18.0 &&
		!token.success() )
	      {
		ROS_INFO("Reached target heading [ %f ].", normalized_heading );
		token.succeed();
	      }   

	  }
	  
	  loop_rate.sleep();
	}

      /// Cancel yaw trajectory
      {
	std::unique_lock<std::mutex> lock( control_tf_mutex_ );
	setRotationMask( world_to_desired_tf_, 0, 0, 0, true, false, false );
	setRotationMask( world_to_measurement_tf_, 0, 0, 0, true, false, false );
      }
      
      token.complete();
    }

    void zeroPitchRoll_impl( quickdev::SimpleActionToken token )
      {
	ros::Rate loop_rate( action_loop_rate_hz_ );
      
	while( ros::ok() && token() )
	  {
	    tf::StampedTransform world_to_imu_tf;

	    if( getTransform( "robot/sensors/imu", world_to_imu_tf ) )
	      continue;

	    double roll, pitch, yaw;
	    world_to_imu_tf.getBasis().getRPY( roll, pitch, yaw );
	  
	    {
	      std::unique_lock<std::mutex> lock( control_tf_mutex_ );
	      
	      /// Set desired pitch and roll to zero, leave yaw alone
	      setRotationMask( world_to_desired_tf_, 0, 0, 0, false, true, true );
	      /// Set measured pitch and roll to IMU, leave yaw alone
	      setRotationMask( world_to_measurement_tf_, 0, pitch, roll, false, true, true );

	      if(  uscauv::ring_distance( pitch, 0.0, uscauv::TWO_PI ) < uscauv::PI / 18.0 &&
		   uscauv::ring_distance( roll,  0.0, uscauv::TWO_PI ) < uscauv::PI / 18.0 &&
		   !token.success() )
		{
		  ROS_INFO("Zeroed out pitch and roll." );
		  token.succeed();
		}
	    }
	      
	    loop_rate.sleep();
	  }

      /// Cancel trajectory
      {
	std::unique_lock<std::mutex> lock( control_tf_mutex_ );
	setRotationMask( world_to_desired_tf_, 0, 0, 0, false, true, true );
	setRotationMask( world_to_measurement_tf_, 0, 0, 0, false, true, true );
      }
	
	token.complete();
      }

    void maintainHeading_impl( quickdev::SimpleActionToken token )
    {
      ros::Rate loop_rate( action_loop_rate_hz_ );
      
      double target_heading;
      
      while( ros::ok() && token() )
	{
	  double roll, pitch;
	  
	  tf::StampedTransform world_to_imu_tf;
	  
	  /// if we were able to look up the transform
	  if( !getTransform( "robot/sensors/imu", world_to_imu_tf ) )
	    {
	      world_to_imu_tf.getBasis().getRPY( roll, pitch, target_heading );
	      break;
	    }
	}
      
      while( ros::ok() && token() )
	{
	  tf::StampedTransform world_to_imu_tf;

	  if( getTransform( "robot/sensors/imu", world_to_imu_tf ) )
	    continue;

	  double roll, pitch, yaw;
	  world_to_imu_tf.getBasis().getRPY( roll, pitch, yaw );
	  
	  {
	    std::unique_lock<std::mutex> lock( control_tf_mutex_ );
	    
	    setTransformYaw( world_to_desired_tf_, target_heading );
	    setTransformYaw( world_to_measurement_tf_, yaw );
	    
	    /// TODO: criteria from param
	    if( uscauv::ring_distance( yaw, target_heading, uscauv::TWO_PI ) < uscauv::PI / 18.0 &&
		!token.success() )
	      {
		ROS_INFO("Maintained target heading [ %f ].", target_heading );
		token.succeed();
	      }   
	  }
	  
	  loop_rate.sleep();
	}

      /// Cancel yaw trajectory
      {
	std::unique_lock<std::mutex> lock( control_tf_mutex_ );
	setRotationMask( world_to_desired_tf_, 0, 0, 0, true, false, false );
	setRotationMask( world_to_measurement_tf_, 0, 0, 0, true, false, false );
      }
      
      token.complete();
    }

    void moveToward_impl( quickdev::SimpleActionToken token, double const & x, double const & y, double const & scale )
    {
      	ros::Rate loop_rate( action_loop_rate_hz_ );
      
	double const mag = sqrt( pow(x, 2) + pow(y, 2) );

	_MaskedTwistMsg axis_command;
	axis_command.mask.linear.x  = true;
	axis_command.mask.linear.y  = true;
	axis_command.mask.linear.z  = false;
	axis_command.mask.angular.x = false;
	axis_command.mask.angular.y = false;
	axis_command.mask.angular.z = false;
	axis_command.twist.linear.x = x / mag * scale;
	axis_command.twist.linear.y = y / mag * scale;
	
	while( ros::ok() && token() )
	  {
	    axis_command_pub_.publish( axis_command );
	      
	    loop_rate.sleep();
	  }
	
	/// Cancel trajectory
	ROS_INFO("Canceling axis command");
	{
	  axis_command.twist.linear.x = 0;
	  axis_command.twist.linear.y = 0;
	  
	  axis_command_pub_.publish( axis_command );
	}
	ROS_INFO("Cancelled");
	
	token.complete();
	
	ROS_INFO("returning");
    }
  
    // ################################################################
    // Callback functions #############################################
    // ################################################################

  private:

    void depthCallback( _DepthMsg::ConstPtr const & msg )
    {
      std::unique_lock< std::mutex > lock( last_depth_msg_mutex_ );

      last_depth_msg_ = msg;
    }

    void killswitchCallback( _KillswitchMsg::ConstPtr const & msg )
    {
      std::unique_lock< std::mutex > lock( last_killswitch_msg_mutex_ );

      if( last_killswitch_msg_ )
	{
	  if( !last_killswitch_msg_->is_killed && msg->is_killed )
	    killswitch_disabled_condition_.notify_all();
	  else if( last_killswitch_msg_->is_killed && !msg->is_killed )
	    killswitch_enabled_condition_.notify_all();
	}
      last_killswitch_msg_ = msg;
    }
    
    // ################################################################
    // Misc. ##########################################################
    // ################################################################

  private:
    void missionControlThread( std::function< void() > const & mission_plan_function )
    {
      /// TODO: Wait for dervied class's spinFirst to complete
      
      ROS_INFO( "Welcome to USC AUV Mission Control." );
      
      while( ros::ok() )
	{
	  /// Wait until killswitch is enabled
	  if( !auto_start_ )
	    {
	      std::unique_lock<std::mutex> killswitch_enabled_lock( killswitch_enabled_mutex_ );
	    
	      ROS_INFO("Waiting for killswitch enable.");

	      killswitch_enabled_condition_.wait( killswitch_enabled_lock );
	    }
	  else
	    auto_start_ = false;
	  
	  ROS_INFO("Killswitch enabled. Launching Mission Plan thread..." );
	  
	  boost::thread external_loop_thread( &MissionControlPolicy::missionPlanThread, this, mission_plan_function );
	  
	  /// Wait for killswitch to be disabled
	  {
	    std::unique_lock<std::mutex> killswitch_disabled_lock( killswitch_disabled_mutex_ );
	    
	    killswitch_disabled_condition_.wait( killswitch_disabled_lock );
	  }
	  
	  ROS_INFO("Killswitch disabled. Cancelling all active action tokens...");
	  
	  /// Cancel all tokens
	  /// TODO: Find a way to ensure that all threads return before clear() is called, so that trajectories are guaranteed to be cancelled
	  for( quickdev::SimpleActionToken & action : active_tokens_ )
	    {
	      action.complete();
	    }
	  active_tokens_.clear();
	  
	  ROS_DEBUG("Killing Mission Plan thread...");
	  external_loop_thread.interrupt();
	  
	  ROS_DEBUG("Restarting Mission Control loop...");
	}
      
      ros::shutdown();
      return;
    }

    void missionPlanThread( std::function< void() > const & mission_plan_function )
    {
      try
	{
	  mission_plan_function();
	}
      catch( std::exception const & ex)
	{
	  ROS_ERROR("Caught exception [ %s ] while trying to run Mission Plan",
		    ex.what() );
	}
      ROS_INFO("Mission Plan finished cleanly.");

      /// Hang around until the parent thread interrupts or the program is killed
      while(1){ boost::this_thread::interruption_point(); }

      ROS_INFO("was interrupted");
      
      return;
    }

    /// Broadcast the current value of world_to_desired and world_to_measurement
    void updateTransformsThread()
    {
      ros::Rate loop_rate( control_loop_rate_hz_ );

      while( ros::ok() )
	{
	  /// Try to lock the coordinate frame transforms
	  {
	    std::unique_lock< std::mutex > lock( control_tf_mutex_, std::try_to_lock );
	    if( lock )
	      {
		ros::Time now = ros::Time::now();
	      
		std::vector< tf::StampedTransform > controls;
		
		tf::StampedTransform desired( world_to_desired_tf_, now,
					      "/world", "robot/controls/desired" );
		tf::StampedTransform measurement( world_to_measurement_tf_, now,
						  "/world", "robot/controls/measurement" );
		
		controls.push_back( desired ); controls.push_back( measurement );
		
		control_tf_broadcaster_.sendTransform( controls );
	      }
	  }
	  
	  loop_rate.sleep();
	}
    }
    
    // ################################################################
    // Misc. ##########################################################
    // ################################################################
    
    int getTransform( std::string const & target_frame, tf::StampedTransform & output )
    {
      if( tf_listener_.canTransform( "/world", target_frame, ros::Time(0) ))
	{
	  try
	    {
	      tf_listener_.lookupTransform( "/world", target_frame, ros::Time(0), output );

	    }
	  catch(tf::TransformException & ex)
	    {
	      ROS_ERROR( "Caught exception [ %s ] looking up transform", ex.what() );
	      return -1;
	    }
	  return 0;
	}
      else 
	return -1;
    }
    
    void setTransformYaw( tf::Transform & transform, double const & yaw )
    {
      double roll, pitch, old_yaw;
      transform.getBasis().getRPY( roll, pitch, old_yaw );
      transform.getBasis().setRPY( roll, pitch, yaw );
      return;
    }

    void setRotationMask( tf::Transform & transform, double const & yaw,  double const & pitch,  double const & roll,
			  bool const & yaw_mask = false,  bool const & pitch_mask = false,  bool const & roll_mask = false )
    {
      double old_roll, old_pitch, old_yaw;
      transform.getBasis().getRPY( old_roll, old_pitch, old_yaw );
      unsigned char mask = ( yaw_mask << 2 ) + ( pitch_mask << 1) + roll_mask;

      switch (mask)
	{
	case 0:
	  break;
	case 1:
	  {
	    transform.getBasis().setRPY( roll, old_pitch, old_yaw );
	    break;
	  }
	case 2:
	  {
	    transform.getBasis().setRPY( old_roll, pitch, old_yaw );
	    break;
	  }
	case 3:
	  {
	    transform.getBasis().setRPY( roll, pitch, old_yaw );
	    break;
	  }
	case 4:
	  {
	    transform.getBasis().setRPY( old_roll, old_pitch, yaw );
	    break;
	  }
	case 5:
	  {
	    transform.getBasis().setRPY( roll, old_pitch, yaw );
	    break;
	  }
	case 6:
	  {
	    transform.getBasis().setRPY( old_roll, pitch, yaw );
	    break;
	  }
	case 7:
	  {
	    transform.getBasis().setRPY( roll, pitch, yaw );
	    break;
	  }
	}

      return;
    }
    
  };
    
} // uscauv

#endif // USCAUV_AUVMISSIONS_MISSIONCONTROLPOLICY
