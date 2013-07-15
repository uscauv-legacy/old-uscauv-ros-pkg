/***************************************************************************
 *  include/auv_controls/control_server_node.h
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


#ifndef USCAUV_AUVCONTROLS_CONTROLSERVER
#define USCAUV_AUVCONTROLS_CONTROLSERVER

// ROS
#include <ros/ros.h>

// uscauv
#include <uscauv_common/base_node.h>

#include <auv_controls/controller.h>
#include <auv_physics/thruster_axis_model.h>

#include <auv_msgs/MaskedTwist.h>
#include <auv_msgs/MotorPowerArray.h>

#include <eigen_conversions/eigen_msg.h>

class ControlServerNode: public BaseNode, public uscauv::PID6D
{
 public:
  typedef Eigen::Matrix<double, 6, 1> AxisValueVector;
  typedef Eigen::Matrix<bool, 6, 1> AxisMaskVector;
 private:
  typedef auv_msgs::MaskedTwist _MaskedTwistMsg;
  
 private:
  /// 
  uscauv::ReconfigurableThrusterAxisModel thruster_axis_model_;
  
  AxisValueVector axis_command_value_;
  AxisMaskVector axis_command_mask_;

  /// ROS
  ros::NodeHandle nh_rel_;
  ros::Subscriber axis_command_sub_;
  ros::Publisher motor_pub_;
  
 public:
 ControlServerNode(): BaseNode("ControlServer"), thruster_axis_model_("model/thrusters"), 
    axis_command_value_( AxisValueVector::Zero()), axis_command_mask_( AxisMaskVector::Zero() ),
    nh_rel_("~")
   {
   }

 private:

  // Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  void spinFirst()
     {
       axis_command_sub_ = nh_rel_.subscribe( "axis_cmd", 10, &ControlServerNode::axisCommandCallback, this );

       motor_pub_ = nh_rel_.advertise<auv_msgs::MotorPowerArray>( "motor_levels", 10 );
       
       /// Load thruster models
       thruster_axis_model_.load("robot/thrusters");

       /// Load PIDs
       loadController();

       setObserved<PID6D::Axes::SURGE> (0.0f);
       setObserved<PID6D::Axes::SWAY>  (0.0f);
       setObserved<PID6D::Axes::HEAVE> (0.0f);
       setObserved<PID6D::Axes::ROLL>  (0.0f);
       setObserved<PID6D::Axes::PITCH> (0.0f);
       setObserved<PID6D::Axes::YAW>   (0.0f);
       
     }  

  // Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
  {
    AxisValueVector axis_control = updateAllPID();

    auv_msgs::MotorPowerArray motor_control = thruster_axis_model_.AxisToMotorArray( axis_control );
    
    motor_pub_.publish( motor_control );
  }
  
 private:
  
  void axisCommandCallback(_MaskedTwistMsg::ConstPtr const & msg)
  {
    /// kinda hack-y, msg.mask's numeric variables are floats, when they should be bools
    AxisValueVector input_value, input_mask_float;
    AxisMaskVector input_mask_bool;
    
    tf::twistMsgToEigen( msg->twist, input_value );
    tf::twistMsgToEigen( msg->mask, input_mask_float );
    input_mask_bool = input_mask_float.cast<bool>();

    /// TODO: Check for collisions with current pose command
    for(unsigned int idx = 0; idx < 6; ++idx)
      {
	if( input_mask_bool( idx ) )
	  {
	    axis_command_mask_( idx ) = true;
	    axis_command_value_( idx ) = input_value( idx );
	  }
      }
    
    updateSetpoints();
  }

  void updateSetpoints()
  {
    if( axis_command_mask_(0) )
      setSetpoint<PID6D::Axes::SURGE>( axis_command_value_(0) );
    if( axis_command_mask_(1) )
      setSetpoint<PID6D::Axes::SWAY>( axis_command_value_(1) );
    if( axis_command_mask_(2) )
      setSetpoint<PID6D::Axes::HEAVE>( axis_command_value_(2) );
    if( axis_command_mask_(3) )
      setSetpoint<PID6D::Axes::ROLL>( axis_command_value_(3) );
    if( axis_command_mask_(4) )
      setSetpoint<PID6D::Axes::PITCH>( axis_command_value_(4) );
    if( axis_command_mask_(5) )
      setSetpoint<PID6D::Axes::YAW>( axis_command_value_(5) );
  }

};

#endif // USCAUV_AUVCONTROLS_CONTROLSERVER
