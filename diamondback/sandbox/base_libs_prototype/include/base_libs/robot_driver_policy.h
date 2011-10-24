/***************************************************************************
 *  include/base_libs/robot_driver_policy.h
 *  --------------------
 * 
 *  Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
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

#ifndef BASE_LIBS_BASE_LIBS_ROBOT_DRIVER_POLICY_H_
#define BASE_LIBS_BASE_LIBS_ROBOT_DRIVER_POLICY_H_

#include <base_libs/tf_tranceiver_policy.h>
#include <base_libs/multi_subscriber.h>
#include <base_libs/multi_publisher.h>
#include <base_libs/timed_policy.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread/mutex.hpp>

template<class __Output>
__Output convert( const geometry_msgs::Vector3 & vec )
{
	return __Output();
}

template<>
btVector3 convert<btVector3>( const geometry_msgs::Vector3 & vec )
{
	return btVector3( vec.x, vec.y, vec.z );
}

template<>
btQuaternion convert<btQuaternion>( const geometry_msgs::Vector3 & vec )
{
	return btQuaternion( vec.z, vec.y, vec.x );
}

geometry_msgs::Vector3 convert( const btVector3 & vec )
{
	geometry_msgs::Vector3 result;
	result.x = vec.getX();
	result.y = vec.getY();
	result.z = vec.getZ();
	return result;
}

geometry_msgs::Vector3 convert( const btQuaternion & quat )
{
	geometry_msgs::Vector3 result;
	
	const btMatrix3x3 rot_mat( quat );
	rot_mat.getEulerYPR( result.z, result.y, result.x );
	
	return result;
}

geometry_msgs::Twist convert( const btTransform & transform )
{
	geometry_msgs::Twist result;
	result.angular = convert( transform.getRotation() );
	result.linear = convert( transform.getOrigin() );
	return result;
}

btTransform convert( const geometry_msgs::Twist & twist )
{
	const btQuaternion quat( convert<btQuaternion>( twist.angular ).normalized() );
	const btVector3 vec( convert<btVector3>( twist.linear ) );
		
	return btTransform( quat, vec );
}

void operator*=( btTransform & transform, const double & scale )
{
	btVector3 angle_ypr = convert<btVector3>( convert( transform.getRotation() ) );
	angle_ypr *= scale;
	transform.setRotation( convert<btQuaternion>( convert( angle_ypr ) ) );
	transform.getOrigin() *= scale;
}

namespace base_libs
{

BASE_LIBS_DECLARE_POLICY( RobotDriver, TfTranceiverPolicy, TimedPolicy )

BASE_LIBS_DECLARE_POLICY_CLASS( RobotDriver )
{
	BASE_LIBS_MAKE_POLICY_NAME( RobotDriver )
	
protected:
	boost::mutex cmd_vel_cache_mutex_;
	geometry_msgs::Twist::ConstPtr cmd_vel_cache_;
	ros::MultiSubscriber<> multi_sub_;
	
	std::string
		robot_name_,
		world_frame_name_,
		robot_frame_name_,
		target_frame_name_;
	
	BASE_LIBS_DECLARE_POLICY_CONSTRUCTOR( RobotDriver ),
		initialized_( false )
	{
		printPolicyActionStart( "create", this );
		printPolicyActionDone( "create", this );
	}
	
	BASE_LIBS_ENABLE_INIT
	{
		printPolicyActionStart( "initialize", this );
		
		ros::NodeHandle & nh_rel = NodeHandlePolicy::getNodeHandle();
		
		multi_sub_.addSubscriber( nh_rel, getMetaParamDef<std::string>( "cmd_vel_topic_name", "cmd_vel", args... ), &RobotDriverPolicy::cmdVelCB, this );
		
		robot_name_ = getMetaParamDef<std::string>( "robot_name", "", args... );
		world_frame_name_ = getMetaParamDef<std::string>( "world_frame_name", "/world", args... );
		robot_frame_name_ = getMetaParamDef<std::string>( "robot_frame_name", "base_link", args... );
		target_frame_name_ = getMetaParamDef<std::string>( "target_frame_name", "desired_pose", args... );
		
		target_frame_name_ = robot_name_ + "/" + target_frame_name_;
		robot_frame_name_ = robot_name_ + "/" + robot_frame_name_;
		
		BASE_LIBS_SET_INITIALIZED;
		
		printPolicyActionDone( "initialize", this );
	}
	
	void cmdVelCB( const geometry_msgs::Twist::ConstPtr & msg )
	{
		BASE_LIBS_CHECK_INITIALIZED;
		
		if( !cmd_vel_cache_mutex_.try_lock() ) return;
		
		cmd_vel_cache_ = msg;
		TimedPolicy::update();
		
		PRINT_INFO( "dt: %f", dt_ );
		
		btTransform velocity_tf( convert( *msg ) );
		velocity_tf *= dt_;
			
		auto world_frame_to_target_frame_last = lookupTransform( world_frame_name_, target_frame_name_, last_time_ );
		world_frame_to_target_frame_last *= velocity_tf;
		
		// update world_frame_to_target_frame in tf now that we've added velocity*dt to it
		publishTransform( world_frame_to_target_frame_last, now_ );
		
		cmd_vel_cache_mutex_.unlock();
	}
	
	tf::StampedTransform getTransformToTarget()
	{
		BASE_LIBS_CHECK_INITIALIZED;
		
		return lookupTransform( robot_frame_name_, target_frame_name_, now_ );
	}
};

}

#endif // BASE_LIBS_BASE_LIBS_ROBOT_DRIVER_POLICY_H_
