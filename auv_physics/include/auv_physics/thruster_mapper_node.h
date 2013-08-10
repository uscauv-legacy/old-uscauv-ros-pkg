/***************************************************************************
 *  include/auv_physics/thruster_mapper_node.h
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


#ifndef USCAUV_AUVPHYSICS_THRUSTERMAPPER
#define USCAUV_AUVPHYSICS_THRUSTERMAPPER

// ROS
#include <ros/ros.h>

// uscauv
#include <uscauv_common/base_node.h>
#include <auv_physics/thruster_axis_model.h>

#include <auv_msgs/MotorPowerArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>

typedef auv_msgs::MotorPower _MotorPowerMsg;
typedef auv_msgs::MotorPowerArray _MotorPowerArrayMsg;

typedef uscauv::ReconfigurableThrusterAxisModel<uscauv::ThrusterModelSimpleLookup> _ThrusterAxisModel;

static const double MAX_MOTOR_VAL = 100;

class ThrusterMapperNode: public BaseNode
{
 private:

  _ThrusterAxisModel thruster_axis_model_;

  /// ros
  ros::NodeHandle nh_rel_;
  ros::Publisher motor_pub_, wrench_pub_;
  ros::Subscriber axis_sub_;
  
 public:
 ThrusterMapperNode(): BaseNode("ThrusterMapper"), thruster_axis_model_("model/thrusters"),
    nh_rel_("~")
      {
      }

 private:

  // Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  void spinFirst()
  {
    axis_sub_ = nh_rel_.subscribe( "axis_in", 10, &ThrusterMapperNode::axisCallback, this );

    motor_pub_ = nh_rel_.advertise<_MotorPowerArrayMsg>("motor_levels", 10);
    wrench_pub_ = nh_rel_.advertise<geometry_msgs::Wrench>("thruster_wrench", 10);
    
    thruster_axis_model_.load("robot/thrusters");
  }  

  // Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
  {

  }

  void axisCallback( geometry_msgs::Twist::ConstPtr const & msg )
  {
    _ThrusterAxisModel::AxisVector desired_axis;
    desired_axis <<
      msg->linear.x,
      msg->linear.y,
      msg->linear.z,
      msg->angular.x,
      msg->angular.y,
      msg->angular.z;

    _MotorPowerArrayMsg motor_levels = thruster_axis_model_.AxisToMotorArray( desired_axis );

    /**
     * Normalize thrusters on the same axis to have maximum possible motor value
     * Future versions will do this without explicitly mapping axes to thrusters
     */
    normalizeAxes( motor_levels );
    
    motor_pub_.publish( motor_levels );

    /// Get predicted wrench on auv body due to firing thrusters 
    geometry_msgs::Wrench wrench_on_body = thruster_axis_model_.MotorArrayToWrench( motor_levels );
    wrench_pub_.publish( wrench_on_body );
    
  }
 private:
  
  void normalizeAxes( _MotorPowerArrayMsg & msg )
  {
    if( normalizeMotorPair( msg, "thruster1", "thruster2" ) ||
	normalizeMotorPair( msg, "thruster3", "thruster4" ) ||
	normalizeMotorPair( msg, "thruster5", "thruster6" ) )
      ROS_ERROR("Failed to normalize motor vals.");
  }

  int normalizeMotorPair( _MotorPowerArrayMsg & msg, std::string const & motor1, std::string const & motor2 )
  {
    int motor1_idx = getMotorIndex(msg, motor1), motor2_idx = getMotorIndex(msg, motor2);
    if( motor1_idx < 0 || motor2_idx < 0)
      return -1;

    double & power1 = msg.motors[ motor1_idx ].power;
    double & power2 = msg.motors[ motor2_idx ].power;

    double max = std::max( fabs( power1), fabs( power2 ) );

    if(max <= MAX_MOTOR_VAL )
      return 0;
    
    power1 *= MAX_MOTOR_VAL/max;
    power2 *= MAX_MOTOR_VAL/max;
    
    return 0;
  }

  int getMotorIndex( _MotorPowerArrayMsg & msg, std::string const & name )
  {
    unsigned int idx = 0;
    
    for( _MotorPowerMsg & motor : msg.motors )
      {
	if( motor.name == name )
	  return idx;
	++idx;
      }
    
    return -1;
  }

};

#endif // USCAUV_AUVPHYSICS_THRUSTERMAPPER
