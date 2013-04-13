/***************************************************************************
 *  include/xsens_driver/xsens_driver_node.h
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

#ifndef USCAUV_XSENSDRIVER_XSENSDRIVERNODE_H
#define USCAUV_XSENSDRIVER_XSENSDRIVERNODE_H

/// C++11 std::shared_ptr
#include <memory>

/// ROS
#include <ros/ros.h>

/// Xsens Driver class
#include <xsens_driver/xsens_driver.h>

class XsensDriverNode
{
 private:
  /// publishers and subscribers
  ros::NodeHandle nh_rel_;
  
  /// parameters
  double loop_rate_hz_;
  std::string port_;

  /// driver
  std::shared_ptr<XsensDriver> xsens_driver_;
  
 public:

  /** 
   * Default Constructor
   * 
   */
 XsensDriverNode()
    :
    nh_rel_("~")
      {}
    
 private:
    /// Running spin() will cause this function to be called before the node begins looping the spingOnce() function.
    void spinFirst()
    {
      if( !nh_rel_.getParam("port", port_) )
	{
	  port_ = "/dev/seabee/imu";
	  ROS_WARN("Parameter [port] not found. Using default [ %s ].", port_.c_str() );
	}

      /// Create driver on the given port
      xsens_driver_ = std::make_shared<XsensDriver>(port_);

      xsens_driver_->connect();

      /// TODO: Warn if node loop rate is lower than MTi output rate
      
      
      return;
    }

  /// Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
  {
       
    return;
  }

 public:
  
  void spin()
  {
    if( !nh_rel_.getParam("loop_rate", loop_rate_hz_) )
      {
	ROS_WARN("Parameter [loop_rate] not found. Using default.");
	loop_rate_hz_ = 10.0;
      }

    ros::Rate loop_rate( loop_rate_hz_ );

    ROS_INFO( "Spinning up Xsens Driver..." );
    spinFirst();

    ROS_INFO( "Xsens Driver is spinning at %.2f Hz.", loop_rate_hz_ ); 

    while( ros::ok() )
      {
	spinOnce();
	ros::spinOnce();
	loop_rate.sleep();
      }
    
    return;
  }

};

#endif // USCAUV_XSENSDRIVER_XSENSDRIVERNODE_H
