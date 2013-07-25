/***************************************************************************
 *  include/auv_controls/pose_command_test_node.h
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


#ifndef USCAUV_AUVCONTROLS_POSECOMMANDTEST
#define USCAUV_AUVCONTROLS_POSECOMMANDTEST

// ROS
#include <ros/ros.h>

// uscauv
#include <uscauv_common/base_node.h>
#include <uscauv_common/param_loader.h>
#include <uscauv_joystick/joystick_policy.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

struct TransformPair
{
  tf::Transform desired, measurement;

  static TransformPair makeLinearTrajectory( tf::Vector3 vec )
  {
    TransformPair pair = 
      { 
	tf::Transform( tf::Quaternion::getIdentity(), vec ),
	tf::Transform( tf::Quaternion::getIdentity(), tf::Vector3( 0, 0, 0 ) )
      };

    return pair;
  }
  
};

class PoseCommandTestNode: public BaseNode, public JoystickPolicy
{
  ros::NodeHandle nh_rel_;
  tf::TransformBroadcaster control_tf_broadcaster_;
  
  typedef std::map< std::string, TransformPair> _TransformPairMap;

  _TransformPairMap test_input_;
  _TransformPairMap::iterator current_input_;
 
 public:
 PoseCommandTestNode(): BaseNode("PoseCommandTest")
  {
    test_input_ =
      {
	{"surge+", TransformPair::makeLinearTrajectory( tf::Vector3( 1, 0, 0 ) )},
	{"surge-", TransformPair::makeLinearTrajectory( tf::Vector3( -1, 0, 0 ) )},
	{"sway+", TransformPair::makeLinearTrajectory( tf::Vector3( 0, 1, 0 ) )},
	{"sway-", TransformPair::makeLinearTrajectory( tf::Vector3( 0, -1, 0 ) )},
	{"heave+", TransformPair::makeLinearTrajectory( tf::Vector3( 0, 0, 1 ) )},
	{"heave-", TransformPair::makeLinearTrajectory( tf::Vector3( 0, 0, -1 ) )}
	/* {"yaw+", uscauv::StaticThrusterAxisModel::constructAxisVector(0,0,0,0,0,1)}, */
	/* {"yaw-", uscauv::StaticThrusterAxisModel::constructAxisVector(0,0,0,0,0,-1)}, */
	/* {"roll+", uscauv::StaticThrusterAxisModel::constructAxisVector(0,0,0,1,0,0)}, */
	/* {"roll-", uscauv::StaticThrusterAxisModel::constructAxisVector(0,0,0,-1,0,0)}, */
	/* {"pitch+", uscauv::StaticThrusterAxisModel::constructAxisVector(0,0,0,0,1,0)}, */
	/* {"pitch-", uscauv::StaticThrusterAxisModel::constructAxisVector(0,0,0,0,-1,0) */
      };
  }
  
 private:

  // Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  void spinFirst()
  {
    initJoystick( &PoseCommandTestNode::joystickCallback, this );

    current_input_ = test_input_.begin();
    ROS_INFO("Started test input with [ %s ].", current_input_->first.c_str() );
  }  

  // Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
  {
    ros::Time now = ros::Time::now();
       
    std::vector< tf::StampedTransform > controls;
       
    tf::StampedTransform desired( current_input_->second.desired, now,
				  "/world", "robot/controls/desired" );
    tf::StampedTransform measurement( current_input_->second.measurement, now,
				      "/world", "robot/controls/measurement" );
       
    controls.push_back( desired ); controls.push_back( measurement );
       
    control_tf_broadcaster_.sendTransform( controls );
  }

  void joystickCallback()
  {
    if( getButtonAcquired("next") )
      {
	++current_input_;
	if( current_input_ == test_input_.end() )
	  current_input_ = test_input_.begin();
	
	ROS_INFO_STREAM("Switched test input to [ " << current_input_->first << 
			" ]." );
      }
  }

};

#endif // USCAUV_AUVCONTROLS_POSECOMMANDTEST
