/***************************************************************************
 *  include/uscauv_joystick/joystick_client_node.h
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


#ifndef USCAUV_USCAUVJOYSTICK_JOYSTICKCLIENT
#define USCAUV_USCAUVJOYSTICK_JOYSTICKCLIENT

// ROS
#include <ros/ros.h>

// uscauv
#include <uscauv_common/base_node.h>

/// joystick
#include <uscauv_joystick/joystick_policy.h>

/// msgs
#include <auv_msgs/MaskedTwist.h>

typedef auv_msgs::MaskedTwist _MaskedTwistMsg;

class JoystickClientNode: public BaseNode, public JoystickPolicy
{
  ros::NodeHandle nh_rel_;
  ros::Publisher axis_command_pub_;
  
 public:
 JoystickClientNode(): BaseNode("JoystickClient"), nh_rel_("~")
    {
    }

 private:

  // Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  void spinFirst()
  {
    ros::NodeHandle nh;
    
    axis_command_pub_ = nh.advertise<_MaskedTwistMsg>("control_server/axis_cmd", 10);
    
    /// joystickCallback() will be called everytime we get a new joystick message
    initJoystick( &JoystickClientNode::joystickCallback, this );
  }  

  // Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
  {
    if( getButton("enable") )
      {
	_MaskedTwistMsg axis_command;
	
	/// The ROS API for populating messages is great!
	axis_command.mask.linear.x = true;
	axis_command.mask.linear.y = true;
	axis_command.mask.linear.z = false;
	axis_command.mask.angular.x = false;
	axis_command.mask.angular.y = false;
	axis_command.mask.angular.z = true;
	
	axis_command.twist.linear.x = getAxis("linear.x");
	axis_command.twist.linear.y = getAxis("linear.y");
	axis_command.twist.angular.z = getAxis("angular.z");
	
	axis_command_pub_.publish( axis_command );
      }
  }
  
  /// Use to ensure that time-dependent joystick inputs are debounced
  void joystickCallback()
  {
    if( getButtonReleased("enable") )
      {
	/// messages are initialized to zero
	_MaskedTwistMsg axis_command;

	axis_command.mask.linear.x = true;
	axis_command.mask.linear.y = true;
	axis_command.mask.linear.z = false;
	axis_command.mask.angular.x = false;
	axis_command.mask.angular.y = false;
	axis_command.mask.angular.z = true;
	
	axis_command_pub_.publish( axis_command );
      }
  }

};

#endif // USCAUV_USCAUVJOYSTICK_JOYSTICKCLIENT
