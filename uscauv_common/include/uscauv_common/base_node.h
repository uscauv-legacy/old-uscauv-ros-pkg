/***************************************************************************
 *  include/uscauv_common/base_node.h
 *  --------------------
 *
 *  Copyright (c) 2013, Dylan Foster
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

#ifndef USCAUV_USCAUVCOMMON_BASENODE
#define USCAUV_USCAUVCOMMON_BASENODE

#include <ros/ros.h>
#include <uscauv_common/param_loader.h>
#include <uscauv_common/defaults.h>

class BaseNode
{
 private:
  /// ROS interfaces
  ros::NodeHandle nh_rel_;

  const std::string node_name_;
  
  double loop_rate_hz_;

 protected:

  /// Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  virtual void spinFirst() = 0;

  /// Running spin() will cause this function to get called at the loop rate until this node is killed.
  virtual void spinOnce() = 0;

 public:

 BaseNode(std::string const & node_name):
  nh_rel_("~"),
  node_name_(node_name)
  {}

  void spin()
  {
    ROS_INFO( "Spinning up %s...", node_name_.c_str() );
    
    loop_rate_hz_ = uscauv::loadParam<double>( nh_rel_, "loop_rate", double(10) );

    ros::Rate loop_rate( loop_rate_hz_ );

    spinFirst();

    ROS_INFO( "%s is spinning at %.2f Hz.", node_name_.c_str(), loop_rate_hz_ ); 

    while( ros::ok() )
      {
	spinOnce();
	ros::spinOnce();
	loop_rate.sleep();
      }
    
    return;
  }

  std::string const & getNodeName()
    {
      return node_name_;
    }

  double const & getLoopRate()
  {
    return loop_rate_hz_;
  }

};

#endif // USCAUV_USCAUVCOMMON_BASENODE
