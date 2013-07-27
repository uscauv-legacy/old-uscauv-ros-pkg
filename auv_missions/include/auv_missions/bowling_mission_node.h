
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
  
  double init_depth_, buoy_depth_;
  double gate_time_;
  std::string object_;
  
 public:
 BowlingMissionNode(): BaseNode("BowlingMission")
    {
    }

 private:

  // Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  void spinFirst()
  {
    ros::NodeHandle nh_rel("~");

    init_depth_ = uscauv::param::load<double>( nh_rel, "init_depth", 0.5); 
    buoy_depth_ = uscauv::param::load<double>( nh_rel, "buoy_depth", 3); 
    gate_time_ = uscauv::param::load<double>( nh_rel, "gate_time", 30); 
    object_ = uscauv::param::load<std::string>( nh_rel, "object", "buoy"); 
    startMissionControl( &BowlingMissionNode::missionPlan, this );
  }  

  void missionPlan()
  {
    SimpleActionToken ori_token = zeroPitchRoll();
    ori_token.wait(5);
    
    SimpleActionToken heading_token = maintainHeading();
    heading_token.wait(1);

    SimpleActionToken dive_token = diveTo( init_depth_ );
    ROS_INFO("Diving...");
    dive_token.wait(5);
    
    /* /// Go forward */
    SimpleActionToken motion_token = moveToward( 1, 0 );
    ROS_INFO("Bowling...");
    motion_token.wait(45.0);
    motion_token.complete();
    dive_token.complete();

    ROS_INFO("Diving to buoy depth");
    SimpleActionToken dive_token2 = diveTo( 2 );
    dive_token2.wait(15);

    ROS_INFO("Searching for object...");
    SimpleActionToken find_object_token = findObject( object_ );
    SimpleActionToken motion_token2 = moveToward( 1, 0, 0.5 );
    motion_token2.wait(1);
    find_object_token.wait();
    motion_token2.complete();
    heading_token.complete();
    
    ROS_INFO("Moving to object...");
    SimpleActionToken moveto_token = moveToObject( object_, 0 );
    
    while(1){}
    
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
