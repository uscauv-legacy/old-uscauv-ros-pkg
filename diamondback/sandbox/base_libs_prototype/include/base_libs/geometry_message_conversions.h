#include <geometry_msgs/Twist.h>
#include <LinearMath/btTransform.h>

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
