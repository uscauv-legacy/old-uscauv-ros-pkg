/***************************************************************************
 *  src/geometry_message_conversions.cpp
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

#include <base_libs/geometry_message_conversions.h>

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
