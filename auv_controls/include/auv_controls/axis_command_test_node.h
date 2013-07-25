/***************************************************************************
 *  include/auv_controls/axis_command_test_node.h
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


#ifndef USCAUV_AUVCONTROLS_AXISCOMMANDTEST
#define USCAUV_AUVCONTROLS_AXISCOMMANDTEST

// ROS
#include <ros/ros.h>

// uscauv
#include <uscauv_common/base_node.h>
#include <uscauv_common/param_loader.h>
#include <uscauv_joystick/joystick_policy.h>
#include <auv_physics/thruster_axis_model.h>
#include <auv_msgs/MaskedTwist.h>

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

class AxisCommandTestNode: public BaseNode, public JoystickPolicy
{
  typedef std::map<std::string, uscauv::StaticThrusterAxisModel::AxisVector, std::less<std::string>, 
    Eigen::aligned_allocator< std::pair<const std::string, uscauv::StaticThrusterAxisModel::AxisVector> > > _NamedScrewMap;
  typedef auv_msgs::MaskedTwist _MaskedTwistMsg;
  _NamedScrewMap test_input_;
  _NamedScrewMap::iterator current_input_;

  ros::NodeHandle nh_rel_;
  ros::Publisher axis_command_pub_;
  double scale_;
 
 public:
 AxisCommandTestNode(): BaseNode("AxisCommandTest"), nh_rel_("~")
    {
      test_input_ = 
	{
	  {"surge+", uscauv::StaticThrusterAxisModel::constructAxisVector(1,0,0,0,0,0)},
	  {"surge-", uscauv::StaticThrusterAxisModel::constructAxisVector(-1,0,0,0,0,0)},
	  {"sway+", uscauv::StaticThrusterAxisModel::constructAxisVector(0,1,0,0,0,0)},
	  {"sway-", uscauv::StaticThrusterAxisModel::constructAxisVector(0,-1,0,0,0,0)},
	  {"heave+", uscauv::StaticThrusterAxisModel::constructAxisVector(0,0,1,0,0,0)},
	  {"heave-", uscauv::StaticThrusterAxisModel::constructAxisVector(0,0,-1,0,0,0)},
	  {"yaw+", uscauv::StaticThrusterAxisModel::constructAxisVector(0,0,0,0,0,1)},
	  {"yaw-", uscauv::StaticThrusterAxisModel::constructAxisVector(0,0,0,0,0,-1)},
	  {"roll+", uscauv::StaticThrusterAxisModel::constructAxisVector(0,0,0,1,0,0)},
	  {"roll-", uscauv::StaticThrusterAxisModel::constructAxisVector(0,0,0,-1,0,0)},
	  {"pitch+", uscauv::StaticThrusterAxisModel::constructAxisVector(0,0,0,0,1,0)},
	  {"pitch-", uscauv::StaticThrusterAxisModel::constructAxisVector(0,0,0,0,-1,0)}
      };

    }

 private:

  // Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  void spinFirst()
  {
    ros::NodeHandle nh;
    
    axis_command_pub_ = nh.advertise<_MaskedTwistMsg>("control_server/axis_cmd", 10);
    
    initJoystick( &AxisCommandTestNode::joystickCallback, this );
    
    scale_ = uscauv::param::load<double>( nh_rel_, "scale", 100 );
    
    current_input_ = test_input_.begin();
    
    ROS_INFO_STREAM( "Test input is [ " << current_input_->first << 
		     " ], ( " << current_input_->second.transpose() << ")." );
  }  
  
  // Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
  {
    _MaskedTwistMsg axis_command;
    
    applyAxisVector( current_input_->second * scale_, axis_command );
    
    axis_command_pub_.publish( axis_command );
  }
  
  void joystickCallback()
  {
    if( getButtonAcquired("next") )
      {
	++current_input_;
	if( current_input_ == test_input_.end() )
	  current_input_ = test_input_.begin();
	
	ROS_INFO_STREAM("Switched test input to [ " << current_input_->first << 
			" ], ( " << current_input_->second.transpose() << ")." );
	
      }
  }

  void  applyAxisVector( uscauv::StaticThrusterAxisModel::AxisVector const &  vec, 
			 _MaskedTwistMsg & cmd )
  {
    cmd.mask.linear.x  = true;
    cmd.mask.linear.y  = true;
    cmd.mask.linear.z  = true;
    cmd.mask.angular.x = true;
    cmd.mask.angular.y = true;
    cmd.mask.angular.z = true;

    cmd.twist.linear.x = vec(0);
    cmd.twist.linear.y = vec(1);
    cmd.twist.linear.z = vec(2);
    cmd.twist.angular.x = vec(3);
    cmd.twist.angular.y = vec(4);
    cmd.twist.angular.z = vec(5);
  }
  
  
};

#endif // USCAUV_AUVCONTROLS_AXISCOMMANDTEST
