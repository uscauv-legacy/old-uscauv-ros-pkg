/***************************************************************************
 *  include/uscauv_common/pose_integrator.h
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

#ifndef USCAUV_USCAUVCOMMON_POSEINTEGRATOR
#define USCAUV_USCAUVCOMMON_POSEINTEGRATOR

#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

namespace uscauv
{

  /// TODO: Remove Twist message entirely and use tf instead
  class PoseIntegrator
  {

  protected:
    typedef geometry_msgs::Twist _TwistMsg;
    
    _TwistMsg velocity_;
    tf::Pose pose_;
    ros::Time last_velocity_update_time_;

  public:
  PoseIntegrator()
    : 
    pose_( tf::Pose( tf::Quaternion(0.0, 0.0, 0.0, 1.0) ) ),
      last_velocity_update_time_( ros::Time::now() )
	{
	  velocity_ = initTwist();
	}
 
  PoseIntegrator(_TwistMsg const & velocity)
    : 
    velocity_(velocity),
      last_velocity_update_time_(ros::Time::now())
	{}

    operator tf::Pose() const
      {
	return pose_;
      }

    tf::Pose getPose() const
      {
	return pose_;
      }
  
    void updatePose()
    {
      /// Get the amount of time since we last updated the pose
      ros::Time now = ros::Time::now();
      double dt = (now - last_velocity_update_time_).toSec();

      /// Integrate linear velocity
      tf::Vector3 linear_velocity;
      tf::vector3MsgToTF(velocity_.linear, linear_velocity);
      tf::Vector3 odom_vector = linear_velocity * dt;
    
      /// Integrate angular velocity 
      tf::Quaternion odom_quat = tf::createQuaternionFromRPY(velocity_.angular.x *dt,
							     velocity_.angular.y *dt,
							     velocity_.angular.z *dt);
      /// Combine into a single transform
      tf::Pose odom_pose = tf::Pose(odom_quat, odom_vector);
    
      /**
       * Compose the two frames. This is equivalent adding the vector of odom_pose 
       * rotated by the quaternion of pose_ to the vector of pose_ and composing the  
       * quaternions of pose_ and odom_pose.
       */
      pose_ *= odom_pose;

      /**
       * Same deal as above, but this doesn't the rotation of the cursor pose to the linear odom vector before adding it
       * The result is that direction controls will make the cursor travel along the X/Y/Z axes of the world frame 
       * instead of the X/Y/Z axes of its own rotated frame
       */
      /* /// Integrate linear velocity and add to current pose */
      /* tf::Vector3 linear_velocity; */
      /* tf::vector3MsgToTF(velocity_.linear, linear_velocity); */
      /* pose_.setOrigin(pose_.getOrigin() + (linear_velocity * dt)); */

      /* /// Integrate angular velocity and add to current pose */
      /* tf::Quaternion odom_quat = tf::createQuaternionFromRPY(velocity_.angular.x *dt, */
      /* 		     velocity_.angular.y *dt, */
      /* 		     velocity_.angular.z *dt); */
      /* tf::Quaternion cursor_pose_quat = pose_.getRotation(); */
      /* /\* pose_.setRotation(cursor_pose_quat * odom_quat); *\/ */
      /* pose_.setRotation(odom_quat * cursor_pose_quat); */

      last_velocity_update_time_ = now;
    }

    void setVelocity(_TwistMsg const & velocity)
    {
      updatePose();
      velocity_ = velocity;
      return;
    }
    
  private:
    _TwistMsg const initTwist(double const & linear_x = 0.0, double const & linear_y = 0.0, 
			      double const & linear_z = 0.0, double const & angular_x = 0.0, 
			      double const & angular_y = 0.0, double const & angular_z = 0.0)
    {
      _TwistMsg twist;
      twist.linear.x = linear_x;
      twist.linear.y = linear_y;
      twist.linear.z = linear_z;
      twist.angular.x = angular_x;
      twist.angular.y = angular_y;
      twist.angular.z = angular_z;
      return twist;
    }
  };

}

#endif // USCAUV_USCAUVCOMMON_POSEINTEGRATOR
