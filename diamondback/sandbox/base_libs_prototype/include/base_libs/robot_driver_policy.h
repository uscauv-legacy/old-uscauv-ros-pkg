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
	
	BASE_LIBS_DECLARE_POLICY_CONSTRUCTOR( RobotDriver )
	{
		printPolicyActionStart( "create", this );
		printPolicyActionDone( "create", this );
	}
	
	BASE_LIBS_ENABLE_INIT
	{
		multi_sub_.addSubscriber( nh_rel_, getMetaParamDef<std::string>( "cmd_vel_topic_name", "cmd_vel", args... ), &RobotDriverPolicy::cmdVelCB, this );
		
		robot_name_ = getMetaParamDef<std::string>( "robot_name", "", args... );
		world_frame_name_ = getMetaParamDef<std::string>( "world_frame_name", "/world", args... );
		robot_frame_name_ = getMetaParamDef<std::string>( "robot_frame_name", "base_link", args... );
		target_frame_name_ = getMetaParamDef<std::string>( "target_frame_name", "desired_pose", args... );
		
		target_frame_name_ = robot_name_ + "/" + target_frame_name_;
		robot_frame_name_ = robot_name_ + "/" + robot_frame_name_;
	}
	
	void cmdVelCB( const geometry_msgs::Twist::ConstPtr & msg )
	{
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
		return lookupTransform( robot_frame_name_, target_frame_name_, now_ );
	}
};

}

#endif // BASE_LIBS_BASE_LIBS_ROBOT_DRIVER_POLICY_H_
