/***************************************************************************
 *  include/base_libs/tf_tranceiver_policy.h
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

#ifndef BASE_LIBS_BASE_LIBS_TF_TRANCEIVER_POLICY_H_
#define BASE_LIBS_BASE_LIBS_TF_TRANCEIVER_POLICY_H_

#include <base_libs/node_handle_policy.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace base_libs
{

#define DEFAULT_LOOKUP_TIME ros::Time( 0 )

BASE_LIBS_DECLARE_POLICY( TfTranceiver, NodeHandlePolicy )

BASE_LIBS_DECLARE_POLICY_CLASS( TfTranceiver )
{
	BASE_LIBS_MAKE_POLICY_NAME( TfTranceiver )
	
protected:
	tf::TransformBroadcaster tf_publisher_;
	tf::TransformListener tf_listener_;
	
public:
	typedef std::string _TfFrameId;
	
	BASE_LIBS_DECLARE_POLICY_CONSTRUCTOR( TfTranceiver )
	{
		printPolicyActionStart( "create", this );
		printPolicyActionDone( "create", this );
	}
	
	void publishTransform( const tf::StampedTransform & transform, ros::Time frame_time )
	{
		auto new_transform( transform );
		new_transform.stamp_ = frame_time;
		publishTransform( new_transform );
	}
	
	void publishTransform( const tf::StampedTransform & transform )
	{
		if( transform.frame_id_.size() == 0 || transform.child_frame_id_.size() == 0 )
		{
			PRINT_WARN( "Cannot publish StampedTransform with empty source frame or target frame id:\n[ %s -> %s ] : %f", transform.frame_id_.c_str(), transform.child_frame_id_.c_str(), transform.stamp_.toSec() );
			return;
		}
		PRINT_DEBUG( "Publishing %s -> %s [%f]", transform.frame_id_.c_str(), transform.child_frame_id_.c_str(), transform.stamp_.toSec() );
		tf_publisher_.sendTransform( transform );
	}
	
	void publishTransform(
		const tf::Transform & transform,
		const _TfFrameId & from_frame_id,
		const _TfFrameId & to_frame_id )
	{
		publishTransform(
			transform,
			from_frame_id,
			to_frame_id,
			ros::Time::now() );
	}
	
	void publishTransform(
		const tf::Transform & transform,
		const _TfFrameId & from_frame_id,
		const _TfFrameId & to_frame_id,
		const ros::Time & frame_time )
	{
		publishTransform(
			tf::StampedTransform(
				transform,
				frame_time,
				from_frame_id,
				to_frame_id ) );
	}
	
	tf::StampedTransform lookupTransform(
		const _TfFrameId & from_frame_id,
		const ros::Time & from_frame_time,
		const _TfFrameId & to_frame_id,
		const ros::Time & to_frame_time,
		const _TfFrameId & fixed_frame_id,
		const double & wait_time = 0.05,
		const bool & default_to_latest = true )
	{
		return lookupTransform(
			from_frame_id,
			from_frame_time,
			to_frame_id,
			to_frame_time,
			fixed_frame_id,
			ros::Duration( wait_time ),
			default_to_latest );
	}
	
	tf::StampedTransform lookupTransform(
		const _TfFrameId & from_frame_id,
		const ros::Time & from_frame_time,
		const _TfFrameId & to_frame_id,
		const ros::Time & to_frame_time,
		const _TfFrameId & fixed_frame_id,
		const ros::Duration & wait_time,
		const bool & default_to_latest = true )
	{
		PRINT_DEBUG( "Looking up transform:\n [ %s-> %s ]\n( %f -> %f )...", from_frame_id.c_str(), to_frame_id.c_str(), from_frame_time.toSec(), to_frame_time.toSec() );
		tf::StampedTransform transform( btTransform( tf::createIdentityQuaternion() ), ros::Time::now(), from_frame_id, to_frame_id );
		try
		{
			if( transformExists(
				from_frame_id,
				from_frame_time,
				to_frame_id,
				to_frame_time,
				fixed_frame_id ) )
			{
				tf_listener_.waitForTransform(
					from_frame_id,
					from_frame_time,
					to_frame_id,
					to_frame_time,
					fixed_frame_id,
					wait_time );
				
				tf_listener_.lookupTransform(
					from_frame_id,
					from_frame_time,
					to_frame_id,
					to_frame_time,
					fixed_frame_id,
					transform );
					
				PRINT_DEBUG( "OK\n" );
			}
			else
			{
				PRINT_WARN(
					"Cannot find transform from %s to %s via %s at the given times",
					from_frame_id.c_str(),
					to_frame_id.c_str(),
					fixed_frame_id.c_str() );
				
				if( default_to_latest )
				{
					PRINT_WARN( "Attempting to look up latest transform..." );
					return lookupTransform( from_frame_id, to_frame_id, ros::Time( 0 ), wait_time, false );
				}
				else
				{
					PRINT_WARN(
						"Lookup of  %s -> %s via %s failed",
						from_frame_id.c_str(),
						to_frame_id.c_str(),
						fixed_frame_id.c_str() );
				}
			}
		}
		catch ( const tf::TransformException & ex )
		{
			PRINT_ERROR(
				"%s",
				ex.what() );
		}
		transform.setRotation( transform.getRotation().normalized() );
		return transform;
	}
	
	tf::StampedTransform lookupTransform(
		const _TfFrameId & from_frame_id,
		const _TfFrameId & to_frame_id,
		const double & wait_time = 0.05,
		const bool & default_to_latest = true )
	{
		return lookupTransform(
			from_frame_id,
			to_frame_id,
			DEFAULT_LOOKUP_TIME,
			wait_time );
	}
	
	tf::StampedTransform lookupTransform(
		const _TfFrameId & from_frame_id,
		const _TfFrameId & to_frame_id,
		const ros::Time & frame_time,
		const double & wait_time = 0.05,
		const bool & default_to_latest = true )
	{
		return lookupTransform(
			from_frame_id,
			to_frame_id,
			frame_time,
			ros::Duration( wait_time ) );
	}
	
	tf::StampedTransform lookupTransform(
		const _TfFrameId & from_frame_id,
		const _TfFrameId & to_frame_id,
		const ros::Time & frame_time,
		const ros::Duration & wait_time,
		const bool & default_to_latest = true )
	{
		PRINT_DEBUG( "Looking up transform:\n [ %s-> %s ]\n( %f )...", from_frame_id.c_str(), to_frame_id.c_str(), frame_time.toSec() );
		tf::StampedTransform transform( btTransform( tf::createIdentityQuaternion() ), ros::Time::now(), from_frame_id, to_frame_id );
		try
		{
			if( transformExists(
				from_frame_id,
				to_frame_id,
				frame_time ) )
			{
				tf_listener_.waitForTransform(
					from_frame_id,
					to_frame_id,
					frame_time,
					wait_time );
				
				tf_listener_.lookupTransform(
					from_frame_id,
					to_frame_id,
					frame_time,
					transform );
					
				PRINT_DEBUG( "OK" );
			}
			else
			{
				PRINT_WARN(
					"Cannot find transform from %s to %s at the given time",
					from_frame_id.c_str(),
					to_frame_id.c_str() );
				
				if( default_to_latest )
				{
					PRINT_WARN( "Attempting to look up latest transform..." );
					return lookupTransform( from_frame_id, to_frame_id, ros::Time( 0 ), wait_time, false );
				}
				else
				{
					PRINT_WARN(
						"Lookup of %s -> %s failed",
						from_frame_id.c_str(),
						to_frame_id.c_str() );
				}
			}
		}
		catch ( const tf::TransformException & ex )
		{
			PRINT_ERROR(
				"%s",
				ex.what() );
		}
		transform.setRotation( transform.getRotation().normalized() );
		return transform;
	}
	
	bool transformExists(
		const _TfFrameId & from_frame_id,
		const _TfFrameId & to_frame_id )
	{
		return transformExists(
			from_frame_id,
			to_frame_id,
			DEFAULT_LOOKUP_TIME );
	}
	
	bool transformExists(
		const _TfFrameId & from_frame_id,
		const _TfFrameId & to_frame_id,
		const ros::Time & frame_time )
	{
		// to_frame and from_frame are flipped in the tf api here
		return tf_listener_.canTransform(
			to_frame_id,
			from_frame_id,
			frame_time );
	}
	
	bool transformExists(
		const _TfFrameId & from_frame_id,
		const ros::Time & from_frame_time,
		const _TfFrameId & to_frame_id,
		const ros::Time & to_frame_time,
		const _TfFrameId & fixed_frame_id )
	{
		// to_frame and from_frame are flipped in the tf api here
		return tf_listener_.canTransform(
			to_frame_id,
			to_frame_time,
			from_frame_id,
			from_frame_time,
			fixed_frame_id );
	}
			
};

#undef DEFAULT_LOOKUP_TIME

}

#endif // BASE_LIBS_BASE_LIBS_TF_TRANCEIVER_POLICY_H_
