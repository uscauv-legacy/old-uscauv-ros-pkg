/***************************************************************************
 *  include/base_libs/robot_controller_policy.h
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

#ifndef BASE_LIBS_BASE_LIBS_ROBOT_CONTROLLER_POLICY_H_
#define BASE_LIBS_BASE_LIBS_ROBOT_CONTROLLER_POLICY_H_

#include <base_libs/tf_manager_policy.h>
#include <base_libs/node_handle_policy.h>
#include <base_libs/multi_subscriber.h>
#include <base_libs/multi_publisher.h>

namespace base_libs
{

BASE_LIBS_DECLARE_POLICY( RobotController, TfManagerPolicy, NodeHandlePolicy )

template<class __MotorValsMsg = void>
BASE_LIBS_DECLARE_POLICY_CLASS( RobotController )
{
	BASE_LIBS_MAKE_POLICY_NAME( RobotController )

public:
	typedef void _EmptyMsg;
	typedef TfManagerPolicy::_VelocityMsg _VelocityMsg;

protected:
	ros::MultiPublisher<> multi_pub_;
	ros::MultiSubscriber<> multi_sub_;

	std::string
		robot_name_,
		world_frame_name_,
		robot_frame_name_,
		target_frame_name_,
		cmd_vel_topic_name_,
		motor_vals_topic_name_;

	BASE_LIBS_DECLARE_POLICY_CONSTRUCTOR( RobotController ),
		initialized_( false )
	{
		printPolicyActionStart( "create", this );
		printPolicyActionDone( "create", this );
	}

private:
	// if our motor vals message type is empty, don't create a publisher for it
	template<class __Message>
	BASE_LIBS_ENABLE_IF_SAME( void, __Message, _EmptyMsg )
	createMotorValsPublisher(){}

	// if our motor vals message type is not empty, create a publisher for it
	template<class __Message>
	BASE_LIBS_ENABLE_IF_NOT_SAME( void, __Message, _EmptyMsg )
	createMotorValsPublisher()
	{
		auto & nh_rel = NodeHandlePolicy::getNodeHandle();

		multi_pub_.addPublishers<__MotorValsMsg>( nh_rel, { motor_vals_topic_name_ } );
	}

	void postInit()
	{
		auto & nh_rel = NodeHandlePolicy::getNodeHandle();

		multi_sub_.addSubscriber( nh_rel, cmd_vel_topic_name_, &RobotControllerPolicy::cmdVelCB, this );

		createMotorValsPublisher<__MotorValsMsg>();
	}

public:
	BASE_LIBS_ENABLE_INIT
	{
		printPolicyActionStart( "initialize", this );

		auto & nh_rel = NodeHandlePolicy::getNodeHandle();

		const auto robot_name_param = getMetaParamDef<std::string>( "robot_name_param", "robot_name", args... );
		robot_name_ = ros::ParamReader<std::string, 1>::readParam( nh_rel, robot_name_param, "" );

		const auto world_frame_name_param = getMetaParamDef<std::string>( "world_frame_name_param", "world_frame_name", args... );
		world_frame_name_ = ros::ParamReader<std::string, 1>::readParam( nh_rel, world_frame_name_param, "/world" );

		const auto robot_frame_name_param = getMetaParamDef<std::string>( "robot_frame_name_param", "robot_frame_name", args... );
		robot_frame_name_ = ros::ParamReader<std::string, 1>::readParam( nh_rel, robot_frame_name_param, "base_link" );

		const auto target_frame_name_param = getMetaParamDef<std::string>( "target_frame_name_param", "target_frame_name", args... );
		target_frame_name_ = ros::ParamReader<std::string, 1>::readParam( nh_rel, target_frame_name_param, "desired_pose" );

		target_frame_name_ = robot_name_ + "/" + target_frame_name_;
		robot_frame_name_ = robot_name_ + "/" + robot_frame_name_;

		cmd_vel_topic_name_ = getMetaParamDef<std::string>( "cmd_vel_topic_name_param", robot_name_.size() > 0 ? "/" + robot_name_ + "/cmd_vel" : "cmd_vel", args... );
		motor_vals_topic_name_ = getMetaParamDef<std::string>( "motor_vals_topic_name_param", robot_name_.size() > 0 ? "/" + robot_name_ + "/motor_vals" : "motor_vals", args... );

		// make sure our frames are initialized in the manager (if any frame is already initialized it will not be modified here)
		TfManagerPolicy::registerFrames( world_frame_name_, target_frame_name_ );

		postInit();

		BASE_LIBS_SET_INITIALIZED;

		printPolicyActionDone( "initialize", this );
	}

	BASE_LIBS_DECLARE_MESSAGE_CALLBACK( cmdVelCB, _VelocityMsg )
	{
		BASE_LIBS_CHECK_INITIALIZED;

		TfManagerPolicy::TimedPolicy::update();
		// update the frame by the given velocity (duration will be auto-calculated from our TimedPolicy)
		TfManagerPolicy::updateFrames( target_frame_name_, msg );

		PRINT_INFO( "dt: %f", TfManagerPolicy::TimedPolicy::dt() );

		publishMotorVals<__MotorValsMsg>();
	}

	// if our motor vals message type is empty, don't attempt to publish to its topic
	template<class __Message>
	BASE_LIBS_ENABLE_IF_SAME( void, __Message, _EmptyMsg )
	publishMotorVals(){}

	// if our motor vals message type is not empty, publish it as usual
	template<class __Message>
	BASE_LIBS_ENABLE_IF_NOT_SAME( void, __Message, _EmptyMsg )
	publishMotorVals()
	{
		typename __MotorValsMsg::Ptr motor_vals_msg( new __MotorValsMsg );
		multi_pub_.publish( motor_vals_topic_name_, motor_vals_msg );
	}

	void update()
	{
		BASE_LIBS_CHECK_INITIALIZED;

		// publish all known transforms
		TfManagerPolicy::update();
	}

	tf::StampedTransform getTransformToTarget() const
	{
		BASE_LIBS_CHECK_INITIALIZED;

		return lookupTransform( robot_frame_name_, target_frame_name_, TimedPolicy::now() );
	}
};

}

#endif // BASE_LIBS_BASE_LIBS_ROBOT_CONTROLLER_POLICY_H_
