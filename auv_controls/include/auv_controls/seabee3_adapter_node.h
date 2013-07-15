/***************************************************************************
 *  include/auv_controls/seabee3_adapter_node.h
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


#ifndef USCAUV_AUVCONTROLS_SEABEE3ADAPTER
#define USCAUV_AUVCONTROLS_SEABEE3ADAPTER

// ROS
#include <ros/ros.h>

// uscauv
#include <uscauv_common/base_node.h>

#include <auv_msgs/MotorPowerArray.h>
#include <seabee3_msgs/MotorVals.h>

#include <seabee3_common/movement.h>

typedef seabee3_msgs::MotorVals _MotorValsMsg;
typedef auv_msgs::MotorPower _MotorPowerMsg;
typedef auv_msgs::MotorPowerArray _MotorPowerArrayMsg;

static const unsigned int MAX_MOTOR_VAL = 100;

static const std::map< std::string, int> thruster_map =
  {
    {"thruster1", seabee3_common::movement::MotorControllerIDs::DEPTH_BACK_THRUSTER },
    {"thruster2", seabee3_common::movement::MotorControllerIDs::DEPTH_FRONT_THRUSTER },
    {"thruster3", seabee3_common::movement::MotorControllerIDs::STRAFE_TOP_THRUSTER },
    {"thruster4", seabee3_common::movement::MotorControllerIDs::STRAFE_BOTTOM_THRUSTER },
    {"thruster5", seabee3_common::movement::MotorControllerIDs::FWD_LEFT_THRUSTER },
    {"thruster6", seabee3_common::movement::MotorControllerIDs::FWD_RIGHT_THRUSTER },
  };

class Seabee3AdapterNode: public BaseNode
{
  /// ROS
  ros::NodeHandle nh_rel_;
  ros::Subscriber motor_levels_sub_;
  ros::Publisher motor_vals_pub_;

 public:
 Seabee3AdapterNode(): BaseNode("Seabee3Adapter"), nh_rel_("~")
   {
   }

 private:

  // Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  void spinFirst()
  {
    motor_vals_pub_ = nh_rel_.advertise<_MotorValsMsg>("motor_vals", 10);
    motor_levels_sub_ = nh_rel_.subscribe( "motor_levels", 10,
					   &Seabee3AdapterNode::motorPowerArrayCallback, this );
    
  }  

  // Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
  {
    
  }
  
  /// Map thrusters and publish result
  void motorPowerArrayCallback( _MotorPowerArrayMsg::ConstPtr const & msg )
  {
    _MotorValsMsg motor_vals;
    
    for( _MotorPowerMsg const & motor : msg->motors )
      {
	int motor_id;

	try
	  {
	    motor_id = thruster_map.at( motor.name );
	  }
	catch( std::exception const & ex )
	  {
	    ROS_WARN( "Caught exception [ %s ] looking up ID for thruster [ %s ].", ex.what(), motor.name.c_str() );
	    continue;
	  }

	motor_vals.mask[ motor_id ] = true;
	motor_vals.motors[ motor_id ] = motor.power;
		
      }

    normalizeMotors( motor_vals );

    motor_vals_pub_.publish( motor_vals );
  }

 private:
  
  void normalizeMotors(_MotorValsMsg & msg )
  {
    normalizeAxis(msg, seabee3_common::movement::Axes::SPEED );
    normalizeAxis(msg, seabee3_common::movement::Axes::STRAFE);
    normalizeAxis(msg, seabee3_common::movement::Axes::DEPTH );
  }
  
  void normalizeAxis(_MotorValsMsg & msg, int const & axis)
  {
    int motor1_id = seabee3_common::movement::ThrusterPairs::values[axis][0];
    int motor2_id = seabee3_common::movement::ThrusterPairs::values[axis][1];

    double max = std::max( abs(msg.motors[motor1_id]), abs(msg.motors[motor2_id]) );
    
    if(max <= MAX_MOTOR_VAL )
      return;
    
    msg.motors[motor1_id] *= MAX_MOTOR_VAL/max;
    msg.motors[motor2_id] *= MAX_MOTOR_VAL/max;
  }

};

#endif // USCAUV_AUVCONTROLS_SEABEE3ADAPTER
