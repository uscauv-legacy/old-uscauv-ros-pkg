/***************************************************************************
 *  include/auv_physics/thruster_axis_model_test_node.h
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


#ifndef USCAUV_AUVPHYSICS_THRUSTERAXISMODELTEST
#define USCAUV_AUVPHYSICS_THRUSTERAXISMODELTEST

// ROS
#include <ros/ros.h>

// uscauv
#include <uscauv_common/base_node.h>
#include <auv_physics/thruster_axis_model.h>


class ThrusterAxisModelTestNode: public BaseNode
{
  
 public:
  ThrusterAxisModelTestNode(): BaseNode("ThrusterAxisModelTest")
   {
   }

 private:
  uscauv::ThrusterAxisModel::AxisVector constructAxisVector(double x, double y, double z,
							    double t1, double t2, double t3)
    {
      uscauv::ThrusterAxisModel::AxisVector vec;
      vec << x, y, z, t1, t2, t3;
      return vec;
    }

  // Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  void spinFirst()
     {
       typedef std::map<std::string, uscauv::ThrusterAxisModel::AxisVector> _NamedScrewMap;
       
       uscauv::ThrusterAxisModel model;

       model.load("model/thrusters", "robot/thrusters");

       _NamedScrewMap test_input = {
	 {"forward", constructAxisVector(1,0,0,0,0,0)},
	 {"backward", constructAxisVector(-1,0,0,0,0,0)},
	 {"left", constructAxisVector(0,1,0,0,0,0)},
	 {"right", constructAxisVector(0,-1,0,0,0,0)},
	 {"up", constructAxisVector(0,0,1,0,0,0)},
	 {"down", constructAxisVector(0,0,-1,0,0,0)},
	 {"yaw+", constructAxisVector(0,0,0,0,0,1)},
	 {"yaw-", constructAxisVector(0,0,0,0,0,-1)},
	 {"roll+", constructAxisVector(0,0,0,1,0,0)},
	 {"roll-", constructAxisVector(0,0,0,-1,0,0)},
	 {"pitch+", constructAxisVector(0,0,0,0,1,0)},
	 {"pitch-", constructAxisVector(0,0,0,0,-1,0)}
       };

       for( _NamedScrewMap::const_iterator screw_it = test_input.begin();
	    screw_it != test_input.end(); ++screw_it)
	 {
	   ROS_INFO_STREAM("[ " << screw_it->first << " ] axis input: " << screw_it->second.transpose());
	   ROS_INFO_STREAM("[ " << screw_it->first << " ] thrust output: " <<
			   model.AxisToThruster( screw_it->second ).transpose() );
	 }
       
       
     }  

  // Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
     {

     }

};

#endif // USCAUV_AUVPHYSICS_THRUSTERAXISMODELTEST
