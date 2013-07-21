/***************************************************************************
 *  include/auv_missions/mission_test_node.h
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


#ifndef USCAUV_AUVMISSIONS_MISSIONTEST
#define USCAUV_AUVMISSIONS_MISSIONTEST

// ROS
#include <ros/ros.h>

// uscauv
#include <uscauv_common/base_node.h>
#include <auv_missions/mission_control_policy.h>

#include <boost/thread/thread.hpp>

using namespace quickdev;

class MissionTestNode: public BaseNode, public uscauv::MissionControlPolicy
{
  
 public:
  MissionTestNode(): BaseNode("MissionTest")
   {
   }

 private:

  // Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  void spinFirst()
     {
       startMissionControl( &MissionTestNode::missionPlan, this );
     }  
  
  void missionPlan()
  {
    
    SimpleActionToken dive_token = diveTo( 0.5 );
    
    dive_token.wait();

    /// Good to know:
    /* quickdev::SimpleActionToken bad_token; */
    /* bad_token.start( [&](){ return; } ); */
    /* if( bad_token.wait(5) ) */
    /*   ROS_INFO("Great job!"); */
    /* else */
    /*   ROS_INFO("You fucked up"); */
    
    /* ROS_INFO("Worker thread is entering sleep."); */
    /* boost::this_thread::sleep( boost::posix_time::milliseconds(100000) ); */
    /* ROS_INFO("Worker thread finished sleeping.."); */
			       
  }

  // Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
     {

     }

};

#endif // USCAUV_AUVMISSIONS_MISSIONTEST
