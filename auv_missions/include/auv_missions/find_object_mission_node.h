
/***************************************************************************
 *  include/auv_missions/find_object_mission_node.h
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


#ifndef USCAUV_AUVMISSIONS_FINDOBJECTMISSION
#define USCAUV_AUVMISSIONS_FINDOBJECTMISSION

// ROS
#include <ros/ros.h>

// uscauv
#include <uscauv_common/base_node.h>
#include <auv_missions/mission_control_policy.h>
#include <auv_missions/MissionConfig.h>

using namespace quickdev;
using namespace auv_missions;

class FindObjectMissionNode: public BaseNode, public uscauv::MissionControlPolicy,
  public MultiReconfigure
{

  MissionConfig * config_;
  
 public:
  FindObjectMissionNode(): BaseNode("FindObjectMission")
    {
      addReconfigureServer<MissionConfig>("mission");
      config_ = &getLatestConfig<MissionConfig>("mission");
    }

 private:

  // Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  void spinFirst()
     {
       ros::NodeHandle nh_rel("~");
       
       startMissionControl( &FindObjectMissionNode::missionPlan, this );
     }  

  void missionPlan()
  {
    SimpleActionToken ori_token = zeroPitchRoll();
    ori_token.wait(2.0);

    SimpleActionToken heading_token = maintainHeading();
    heading_token.wait(1);
    
    SimpleActionToken dive_token = diveTo( config_->target1_depth );
    ROS_INFO("Diving to target1 depth...");
    dive_token.wait(5);       
    dive_token.complete();

    ROS_INFO("Searching for target1...");
    SimpleActionToken find_object_token = findObject( config_->target1 );
    SimpleActionToken motion_token = moveToward( 1, 0, 0.5 );
    motion_token.wait(1);
    find_object_token.wait();
    dive_token.complete();
    motion_token.complete();
    heading_token.complete();

    ROS_INFO("Moving to target1...");
    SimpleActionToken moveto_token = moveToObject( config_->target1, config_->target1_distance );
    moveto_token.wait();

    ROS_INFO("Reached safe target1 distance.");
    std::this_thread::sleep_for( std::chrono::seconds(int(config_->target1_hold_time)));    
    moveto_token.complete();
    ROS_INFO("Going in for the kill...");
    SimpleActionToken moveto_token2 = moveToObject( config_->target1, 0 );
    std::this_thread::sleep_for( std::chrono::seconds(int(config_->target1_attack_time)));    
    moveto_token2.complete();
    ROS_INFO("Retreating...");
    SimpleActionToken heading_token3 = maintainHeading();
    SimpleActionToken dive_token4 = diveTo( config_->target1_depth );
    heading_token3.wait(2);
    SimpleActionToken motion_token3 = moveToward( -1, 0, config_->retreat_vel );
    motion_token3.wait(config_->retreat_time);
    dive_token4.complete();
    motion_token3.complete();
    heading_token3.complete();
    ROS_INFO("Retreated. Searching for traffic light");
    
    
    /* SimpleActionToken find_object_token = findObject( object_name_ ); */
    
    /* SimpleActionToken motion_token = moveToward( 1, 0, 0.5, action_token::make_term_criteria( find_object_token ) ); */
    /* motion_token.wait(); */
    /* ROS_INFO("Found object."); */
    /* dive_token.complete(); */

    /* ROS_INFO("Facing to object..."); */
    /* SimpleActionToken faceto_token = moveToObject( object_name_, 1 ); */
    /* faceto_token.wait( ); */
    /* ROS_INFO("Moving to object..."); */
    /* SimpleActionToken moveto_token = moveToObject( object_name_, 1.0 ); */
    /* moveto_token.wait(); */
    
    
  }

  // Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
     {

     }

};

#endif // USCAUV_AUVMISSIONS_FINDOBJECTMISSION
