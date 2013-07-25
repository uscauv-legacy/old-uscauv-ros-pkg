/***************************************************************************
 *  include/auv_missions/bowling_mission_node.h
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


#ifndef USCAUV_AUVMISSIONS_BOWLINGMISSION
#define USCAUV_AUVMISSIONS_BOWLINGMISSION

// ROS
#include <ros/ros.h>

// uscauv
#include <uscauv_common/base_node.h>

#include <auv_missions/mission_control_policy.h>

using namespace quickdev;

class BowlingMissionNode: public BaseNode, public uscauv::MissionControlPolicy
{
  
 public:
 BowlingMissionNode(): BaseNode("BowlingMission")
    {
    }

 private:

  // Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  void spinFirst()
  {
    startMissionControl( &BowlingMissionNode::missionPlan, this );
  }  

  void missionPlan()
  {
    SimpleActionToken ori_token = zeroPitchRoll();
    SimpleActionToken heading_token = maintainHeading();

    /* Dive to 1 meter */
    SimpleActionToken dive_token = diveTo( 1 );
    ROS_INFO("Diving...");
    dive_token.wait( 5.0 );
   
    
    /// Go forward
    SimpleActionToken motion_token = moveToward( 1, 0 );
    ROS_INFO("Bowling...");
    motion_token.wait();
    /* motion_token.wait(5); */
    /* motion_token.complete(); */
    /* while(1){ boost::this_thread::interruption_point();} */
  }
  
  // Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
  {

  }

};

#endif // USCAUV_AUVMISSIONS_BOWLINGMISSION
