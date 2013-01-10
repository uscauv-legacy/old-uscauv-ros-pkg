/***************************************************************************
 *  include/seabee3_teleop/trajectory_setter.h
 *  --------------------
 *
 *  Copyright (c) 2012, Dylan Foster (turtlecannon@gmail.com)
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
 *  * Neither the name of seabee3-ros-pkg nor the names of its
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

#ifndef SEABEE3TELEOP_TRAJECTORYSETTER_H_
#define SEABEE3TELEOP_TRAJECTORYSETTER_H_

#include <quickdev/node.h>
#include <quickdev/multi_publisher.h>

// policies
#include <quickdev/joystick_policy.h>

// messages
#include <visualization_msgs/MarkerArray.h>

// cursor
#include <seabee3_common/markers.h>


typedef quickdev::JoystickPolicy _JoystickPolicy;
typedef seabee::TrajectoryCursor _TrajectoryCursor;
typedef visualization_msgs::MarkerArray _MarkerArrayMsg;

QUICKDEV_DECLARE_NODE( TrajectorySetter, _JoystickPolicy )

QUICKDEV_DECLARE_NODE_CLASS( TrajectorySetter )
{
public:
  /**
   * Default template argument for MultiPublisher<class __Publisher> is ros::Publisher,
   * so this object contains multiple ros::Publishers
   */
  ros::MultiPublisher<> multi_pub_;
  
  _TrajectoryCursor cursor_;
  
  QUICKDEV_DECLARE_NODE_CONSTRUCTOR( TrajectorySetter )
    {

    }


    QUICKDEV_DECLARE_MESSAGE_CALLBACK( joystickCB, _JoystickPolicy::_JoystickMsg )
      {
	if( _JoystickPolicy::isEnabled() )
	  {
	    cursor_.setVelocity( *(_JoystickPolicy::getVelocityMsg()) );
	  }
      }
    
    QUICKDEV_SPIN_FIRST()
    {
      QUICKDEV_GET_RUNABLE_NODEHANDLE(nh_rel);    
      
      /**
       * Topic that we will be publishing visualization data such as the cursor on.
       * Created on node_name/visualization_marker_array because nh_rel is initialized to "~" 
       */
      multi_pub_.addPublishers<_MarkerArrayMsg>(nh_rel, {"visualization_marker_array"});

      /// Set the frame of the cursor marker
      /// TODO:
      cursor_.setFrameID("/trajectory_setter/cursor");
      
      // initialize any remaining policies
      initPolicies<quickdev::policy::ALL>();
    }

    QUICKDEV_SPIN_ONCE()
    {
      /// Update, but don't automatically publish velocity to cmd_vel
      _JoystickPolicy::update(false);

      if( JoystickPolicy::isEnabled() )
	{
	  /// update pose
	  cursor_.updatePose();
	  /// publish the message. TrajectoryCursor is cast to a MarkerArray message
	  multi_pub_.publish(std::string("visualization_marker_array"), _MarkerArrayMsg(cursor_));
	}
      else
	{
	  /// set velocity field to zero
	  cursor_.setVelocity(geometry_msgs::Twist());
	  cursor_.resetPose();
	}
    }
    
}; // TrajectorySetterNode

#endif // SEABEE3TELEOP_TRAJECTORYSETTER_H_
