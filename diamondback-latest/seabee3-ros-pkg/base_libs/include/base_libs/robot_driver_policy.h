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

#include <base_libs/node_handle_policy.h>
#include <base_libs/timed_policy.h>
#include <base_libs/callback_policy.h>
#include <base_libs/multi_subscriber.h>
#include <base_libs/multi_publisher.h>
#include <boost/thread/mutex.hpp>

namespace base_libs
{



BASE_LIBS_DECLARE_POLICY( RobotDriver, NodeHandlePolicy, TimedPolicy )

template<class __MotorValsMsg = void>
class RobotDriverPolicy : public GenericPolicyAdapter<NodeHandlePolicy, TimedPolicy, MessageCallbackPolicy<__MotorValsMsg> >
{
	BASE_LIBS_MAKE_POLICY_NAME( RobotDriver )

protected:
	typedef MessageCallbackPolicy<__MotorValsMsg> _MessageCallbackPolicy;
	typedef GenericPolicyAdapter<NodeHandlePolicy, TimedPolicy, _MessageCallbackPolicy > _PolicyAdapter;
	typedef void _EmptyMsg;
	//boost::mutex motor_vals_cache_mutex_;
	//typename __MotorValsMsg::ConstPtr motor_vals_cache_;

	ros::MultiSubscriber<> multi_sub_;
	// publisher for sensor data, etc
	ros::MultiPublisher<> multi_pub_;

	std::string
		robot_name_,
		motor_vals_topic_name_;

	template<class... __Args>
	RobotDriverPolicy( __Args&&... args ) : _PolicyAdapter( args... ),
		initialized_( false )
	{
		printPolicyActionStart( "create", this );
		printPolicyActionDone( "create", this );
	}

private:
	template<class __Message>
	BASE_LIBS_ENABLE_IF_SAME( void, __Message, _EmptyMsg )
	createMotorValsSubscriber(){}

	template<class __Message>
	BASE_LIBS_ENABLE_IF_NOT_SAME( void, __Message, _EmptyMsg )
	createMotorValsSubscriber()
	{
		auto & nh_rel = NodeHandlePolicy::getNodeHandle();

		multi_sub_.addSubscriber( nh_rel, motor_vals_topic_name_, &RobotDriverPolicy::motorValsCB<__MotorValsMsg>, this );
	}

	void postInit()
	{
		createMotorValsSubscriber<__MotorValsMsg>();
	}

public:
	BASE_LIBS_ENABLE_INIT
	{
		printPolicyActionStart( "initialize", this );

		auto & nh_rel = NodeHandlePolicy::getNodeHandle();

		const auto robot_name_param = getMetaParamDef<std::string>( "robot_name_param", "robot_name", args... );
		robot_name_ = ros::ParamReader<std::string, 1>::readParam( nh_rel, robot_name_param, "" );

		motor_vals_topic_name_ = getMetaParamDef<std::string>( "motor_vals_topic_name_param", robot_name_.size() > 0 ? "/" + robot_name_ + "/motor_vals" : "motor_vals" , args... );

		postInit();

		BASE_LIBS_SET_INITIALIZED;

		printPolicyActionDone( "initialize", this );
	}

private:
	template<class __Message>
	BASE_LIBS_ENABLE_IF_SAME( void, __Message, _EmptyMsg )
	motorValsCB(){}

	template<class __Message>
	BASE_LIBS_ENABLE_IF_NOT_SAME( void, __Message, _EmptyMsg )
	motorValsCB( const typename __Message::ConstPtr & msg )
	{
		BASE_LIBS_CHECK_INITIALIZED;

		//if( !motor_vals_cache_mutex_.try_lock() ) return;

		//motor_vals_cache_ = msg;

		_MessageCallbackPolicy::invokeCallback( msg );

		TimedPolicy::update();

		//motor_vals_cache_mutex_.unlock();
	}
};

}

#endif // BASE_LIBS_BASE_LIBS_ROBOT_DRIVER_POLICY_H_
