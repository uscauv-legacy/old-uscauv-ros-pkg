/***************************************************************************
 *  include/base_libs/tf_manager_policy.h
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

#ifndef BASE_LIBS_BASE_LIBS_TF_MANAGER_POLICY_H_
#define BASE_LIBS_BASE_LIBS_TF_MANAGER_POLICY_H_

#include <base_libs/node_handle_policy.h>
#include <base_libs/tf_tranceiver_policy.h>
#include <base_libs/timed_policy.h>
#include <base_libs/tf_manager.h>
#include <base_libs/multi_subscriber.h>
#include <base_libs/geometry_message_conversions.h>
#include <base_libs/threading.h>
#include <geometry_msgs/Twist.h>

/*! A policy to store and update multiple tf frames
  - subscribes to a command velocity
  - owns a TfManager
  - publishes all owned frames on update
  - allows updating of frames based on command velocity or direct update */

namespace base_libs
{

BASE_LIBS_DECLARE_POLICY_NAMESPACE( TfManager )
{
typedef TimedPolicy<0> _UpdateTimer;
typedef TimedPolicy<1> _CallbackTimer;
}

BASE_LIBS_DECLARE_POLICY( TfManager, NodeHandlePolicy, TfTranceiverPolicy, _UpdateTimer, _CallbackTimer )

BASE_LIBS_DECLARE_POLICY_CLASS( TfManager )
{
	BASE_LIBS_MAKE_POLICY_FUNCS( TfManager )

public:
	typedef geometry_msgs::Twist _VelocityMsg;

	MessageCache<_VelocityMsg> velocity_msg_cache_;

	std::string cmd_vel_topic_name_;
	TfManager tf_manager_;
	ros::MultiSubscriber<> multi_sub_;


	BASE_LIBS_DECLARE_POLICY_CONSTRUCTOR( TfManager ),
		initialized_( false )
	{
		printPolicyActionStart( "create", this );
		printPolicyActionDone( "create", this );
	}

	void postInit()
	{
		auto & nh_rel = NodeHandlePolicy::getNodeHandle();

		multi_sub_.addSubscriber( nh_rel, cmd_vel_topic_name_, &TfManagerPolicy::cmdVelCB, this );
	}

	BASE_LIBS_ENABLE_INIT
	{
		printPolicyActionStart( "initialize", this );

		auto & nh_rel = NodeHandlePolicy::getNodeHandle();

		cmd_vel_topic_name_ = getMetaParamDef<std::string>( "cmd_vel_topic_name_param", "cmd_vel", args... );
		const auto frame_pairs = ros::ParamReader<std::string, 0>::readParams( nh_rel, "frame_pair", "", 0 );

		for( auto frame_pair = frame_pairs.begin(); frame_pair != frame_pairs.end(); ++frame_pair )
		{
			const auto separator_pos = frame_pair->find_first_of( "," );
			if( separator_pos == std::string::npos )
			{
				PRINT_WARN( "Frame pair not properly formatted; format is: <from_frame>,<to_frame>" );
				continue;
			}
			const auto & from_frame = frame_pair->substr( 0, separator_pos );
			const auto & to_frame = frame_pair->substr( separator_pos + 1 );
			registerFrames( from_frame, to_frame );
		}

		postInit();

		BASE_LIBS_SET_INITIALIZED;

		printPolicyActionDone( "initialize", this );
	}

	BASE_LIBS_DECLARE_MESSAGE_CALLBACK( cmdVelCB, _VelocityMsg )
	{
		BASE_LIBS_CHECK_INITIALIZED;

		auto lock = velocity_msg_cache_.tryLockAndUpdate( msg );
		BASE_LIBS_TRY_LOCK_OR_RETURN( lock, "Dropping Twist message." );

		BASE_LIBS_GET_POLICY_NAMESPACE( TfManager )::_CallbackTimer::update();

		updateFrames( msg );
	}

	/*! Publish all transforms stored in our TfManager */
	void update()
	{
		auto & update_timer = BASE_LIBS_GET_POLICY_NAMESPACE( TfManager )::_UpdateTimer::getInstance();
		update_timer.update();

		const auto & transforms = tf_manager_.getTransforms();
		PRINT_DEBUG( "Publishing %zu frames", transforms.size() );

		for( auto transform = transforms.begin(); transform != transforms.end(); ++transform )
		{
			TfTranceiverPolicy::publishTransform( transform->second, update_timer.now() );
		}
	}

	void updateFrames( const _VelocityMsg::ConstPtr & msg )
	{
		const auto & transforms = tf_manager_.getTransforms();
		for( auto transform = transforms.begin(); transform != transforms.end(); ++transform )
		{
			updateFrames( transform->first, msg );
		}
	}

	/*! Update a set of frames (recursively) given a mixed variadic template
	  @param frame_id : the frame to update
	  @param msg : the velocity of the frame
	  @param rest : the rest of the parameters in the variadic template */
	template<class... __Rest>
	void updateFrames( const TfManager::_TfFrameId & frame_id, const _VelocityMsg::ConstPtr & msg, __Rest&&... rest )
	{
		// since no duration is specified, get the duration from our TimedPolicy and pass everything to the next updateFrames function
		const auto & dt = BASE_LIBS_GET_POLICY_NAMESPACE( TfManager )::_CallbackTimer::dt();
		updateFrames( frame_id, msg, dt, rest... );
	}

	/*! Update a set of frames (recursively) given a mixed variadic template
	  @param frame_id : the frame to update
	  @param msg : the velocity of the frame
	  @param dt : the duration for which the given velocity was experienced
	  @param rest : the rest of the parameters in the variadic template */
	template<class... __Rest>
	void updateFrames( const TfManager::_TfFrameId & frame_id, const _VelocityMsg::ConstPtr & msg, const TimedPolicy<>::_Duration & dt, __Rest&&... rest )
	{
		// now that we have a TfManager::_Transform, pass everything to the next updateFrames function
		updateFrames( updateFrameFromVelocity( frame_id, msg, dt ), rest... );
	}

	/*! Update a set of frames (recursively) given a mixed variadic template
	  @param transform : the transform containing the data used to update an existing transform in tf_manager_ */
	template<class... __Rest>
	void updateFrames( const TfManager::_Transform & transform, __Rest&&... rest )
	{
		// update the current transform and pass the rest of the parameters along for processing
		tf_manager_.updateTransforms( transform );
		updateFrames( rest... );
	}

	/*! Bottom level in the updateFrames recursion */
	void updateFrames() const {}

	/*! Register a transform with our TfManager so that it can be looked up locally
	  @param from_frame_id : the name of the parent frame
	  @param to_frame_id : the name of the child frame
	  @param rest : the rest of the parameters in the variadic template */
	template<class... __Rest>
	void registerFrames( const TfManager::_TfFrameId & from_frame_id, const TfManager::_TfFrameId & to_frame_id, __Rest&&... rest )
	{
		if( tf_manager_.exists( to_frame_id ) )
		{
			PRINT_INFO( "Transform [ %s -> %s ] already registered", from_frame_id.c_str(), to_frame_id.c_str() );
		}
		else
		{
			PRINT_INFO( "Registering transform [ %s -> %s ]", from_frame_id.c_str(), to_frame_id.c_str() );
			tf_manager_.updateTransforms( TfManager::_Transform( btTransform( tf::createIdentityQuaternion() ), ros::Time( 0 ), from_frame_id, to_frame_id ) );
		}
		registerFrames( rest... );
	}

	/*! Bottom level in registerFrames recursion */
	void registerFrames() const {}

	/*! Update a single frame (specified by id) given a velocity and duration
	  @param frame_id : the frame to update
	  @param msg : the velocity message to pull values from
	  @param dt : the duration during which the given velocity was experienced */
	TfManager::_Transform updateFrameFromVelocity( const TfManager::_TfFrameId & frame_id, const _VelocityMsg::ConstPtr & msg, const TimedPolicy<>::_Duration & dt ) const
	{
		auto transform = tf_manager_[frame_id];

		btTransform velocity_tf( convert( *msg ) );
		velocity_tf *= dt;

		transform.setData( transform * velocity_tf );

		return transform;
	}

	/*! Update a set of frames (spedified by id) given a velocity and duration
	  @param frame_ids : the list of frames to update; { "frame_id1", "frame_id2", "frame_idN" }
	  @param msg : the velocity message to use when calculating the change in position/orientation of the frames
	  @param dt : the duration to integrate velocity over when calculating the change in position/orientation of the frames */
	void updateFramesFromVelocity( const std::initializer_list<TfManager::_TfFrameId> & frame_ids, const _VelocityMsg::ConstPtr & msg, const TimedPolicy<>::_Duration & dt )
	{
		for( auto frame_id = frame_ids.begin(); frame_id != frame_ids.end(); ++frame_id )
		{
			tf_manager_.updateTransforms( updateFrameFromVelocity( *frame_id, msg, dt ) );
		}
	}
};

}

#endif // BASE_LIBS_BASE_LIBS_TF_MANAGER_POLICY_H_
