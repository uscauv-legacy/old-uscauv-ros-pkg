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

/* New physics simulator			|
____________________________________|*/

#ifndef USCAUV_PHYSICSSIMULATORNODE_H
#define USCAUV_PHYSICSSIMULATORNODE_H

#include <ros/ros.h>
#include <std_msgs/String.h> //what to use for msgs?
#include <tf/transform_broadcaster.h>

/// didn't #include any seabee3_common stuff
/* using namespace seabee3_common; */

class SimpleAUVPhysicsSimulatorNode {
 public:
  SimpleAUVPhysicsSimulatorNode(){}; // Constructor, what params?
  ~SimpleAUVPhysicsSimulatorNode(){}; // Destructor

  ros::Subscriber sub;
  static tf::TransformBroadcaster broadcaster;
  tf::Transform transform;
  
  /// Methods for flow control 
 public:
  /// Running spin() will cause this function to be called before the node begins looping the spingOnce() function.
  void spinFirst()
  {
	  sub = nh_rel.subscribe("motor commands", 100, );

    ROS_INFO( "Finished spinning up." );
    return;
  }

  /// Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
  {
    	//transform.setOrigin()
    	//transform.setRotation()
    	//do other things to transform?
    	broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "physics_simulation_pose"));
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
	loop_rate.sleep();
      }
    
    return;
  }

  /// Methods for handling services
 private:
  
  
};

#endif //USCAUV_PHYSICSSIMULATORNODE_H
