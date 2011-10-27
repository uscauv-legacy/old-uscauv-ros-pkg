/***************************************************************************
 *  include/base_libs/joystick_policy.h
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

#ifndef BASE_LIBS_BASE_LIBS_JOYSTICK_POLICY_H_
#define BASE_LIBS_BASE_LIBS_JOYSTICK_POLICY_H_

#include <base_libs/node_handle_policy.h>
#include <base_libs/multi_publisher.h>
#include <base_libs/multi_subscriber.h>
#include <geometry_msgs/Twist.h>
#include <joy/Joy.h>
#include <map>
#include <sstream>

namespace base_libs
{

BASE_LIBS_DECLARE_POLICY( Joystick, NodeHandlePolicy )

BASE_LIBS_DECLARE_POLICY_CLASS( Joystick )
{
	BASE_LIBS_MAKE_POLICY_NAME( Joystick )

public:
	typedef std::string _AxisName;
	typedef int _AxisIndex;
	typedef double _AxisScale;
	typedef double _AxisValue;
	typedef joy::Joy _JoystickMsg;
	typedef geometry_msgs::Twist _VelocityMsg;
	
	struct Axis
	{
		_AxisName name_;
		_AxisIndex index_;
		_AxisScale scale_;
		bool is_button_;
		
		Axis( _AxisName name = _AxisName(), _AxisIndex index = _AxisIndex(), _AxisScale scale = _AxisScale(), bool is_button = false )
		:
			name_( name ), index_( index ), scale_( scale ), is_button_( is_button )
		{
			//
		}
		
		inline _AxisValue getRawValue( const _JoystickMsg::ConstPtr & msg ) const
		{
			return msg ? ( is_button_ ? msg->buttons[index_] : msg->axes[index_] ) : _AxisValue();
		}
		
		inline _AxisValue getValue( const _JoystickMsg::ConstPtr & msg ) const
		{
			return scale_ * getRawValue( msg );
		}
		
		inline _AxisValue getValueAsButton( const _JoystickMsg::ConstPtr & msg ) const
		{
			return is_button_ ? getRawValue( msg ) : getValue( msg ) < 0.75;
		}
		
		std::string str() const
		{
			std::stringstream ss;
			ss << "[ " << index_ << " : " << name_;
			
			if( is_button_ ) ss << " (button)";
			else ss << " * " << scale_;
			
			ss << " ]";
			return ss.str();
		}
	};
	
	typedef Axis _Axis;
	typedef std::map<_AxisName, _Axis> _AxesMap;

protected:
	ros::MultiPublisher<> multi_pub_;
	ros::MultiSubscriber<> multi_sub_;
	
	makePtr<_VelocityMsg>::_Shared velocity_msg_;
	
	_JoystickMsg::ConstPtr last_joystick_message_;
	ros::Time last_joystick_message_time_;
	bool enabled_;
	double joystick_timeout_;
	
	_AxesMap axes_map_;
	

	BASE_LIBS_DECLARE_POLICY_CONSTRUCTOR( Joystick ),
		last_joystick_message_time_( ros::Time::now() ),
		enabled_( false )
	{
		printPolicyActionStart( "create", this );
		
		preInit();
		
		printPolicyActionDone( "create", this );
	}
	
	void preInit()
	{
		ros::NodeHandle & nh_rel = NodeHandlePolicy::getNodeHandle();
		
		multi_pub_.addPublishers<geometry_msgs::Twist>( nh_rel, { "cmd_vel" } );
		multi_sub_.addSubscriber( nh_rel, "joystick", &JoystickPolicy::joystickCB_0, this );
		
		// if we don't get any joystick messages after this much time, zero out all fields of the outgoing velocity message
		joystick_timeout_ = ros::ParamReader<double, 1>::readParam( nh_rel, "keep_alive_period", 1.0 );
		
		// read as many axis#_index values as exist on the parameter server starting with axis0_index
		auto axis_indices = ros::ParamReader<_AxisIndex, 0>::readParams( nh_rel, "axis", "_index", 0 );
		auto axis_names = ros::ParamReader<_AxisName, 0>::readParams( nh_rel, "axis", "_name", 0 );
		auto axis_scales = ros::ParamReader<_AxisScale, 0>::readParams( nh_rel, "axis", "_scale", 0 );
		
		auto button_indices = ros::ParamReader<_AxisIndex, 0>::readParams( nh_rel, "button", "_index", 0 );
		auto button_names = ros::ParamReader<_AxisName, 0>::readParams( nh_rel, "button", "_name", 0 );
		
		// unroll the axis names and indices into a map from names to indices
		for( unsigned int i = 0; i < axis_names.size() && i < axis_indices.size(); ++i )
		{
			const _AxisName axis_name = axis_names[i];
			const _AxisIndex axis_index = axis_indices[i];
			const _AxisScale axis_scale = axis_scales.size() > i ? axis_scales[i] : _AxisScale( 1.0 );
			_Axis axis( axis_name, axis_index, axis_scale );
			
			PRINT_INFO( "Adding axis: %s", axis.str().c_str() );
			axes_map_[axis_name] = axis;
		}
		
		// unroll the button names and indices into a map from names to indices
		for( unsigned int i = 0; i < button_names.size() && i < button_indices.size(); ++i )
		{
			const _AxisName axis_name = button_names[i];
			const _AxisIndex axis_index = button_indices[i];
			_Axis axis( axis_name, axis_index, 1.0, true );
			
			PRINT_INFO( "Adding axis: %s", axis.str().c_str() );
			axes_map_[axis_name] = axis;
		}
	}

	BASE_LIBS_DECLARE_MESSAGE_CALLBACK( joystickCB_0, _JoystickMsg )
	{
		last_joystick_message_ = msg;
		last_joystick_message_time_ = ros::Time::now();
		
		joystickCB( msg );
	}
	
	virtual BASE_LIBS_DECLARE_MESSAGE_CALLBACK( joystickCB, _JoystickMsg ){}
	
	bool updateVelocityComponent( const _AxisName & axis_name, _AxisValue & axis_value )
	{
		if( !last_joystick_message_ ) return false;
		
		const auto axis_it = axes_map_.find( axis_name );
		if( axis_it == axes_map_.end() ) return false;
		
		const _Axis & axis = axis_it->second;
		
		if( ( ros::Time::now() - last_joystick_message_time_ ).toSec() > joystick_timeout_ ) axis_value = 0;
		else axis_value = axis.getValue( last_joystick_message_ );
		
		return true;
	}
	
	bool axisExists( const _AxisName & axis_name ) const
	{
		return axes_map_.count( axis_name );
	}
	
	_Axis getAxis( const _AxisName & axis_name ) const
	{
		const auto axis_it = axes_map_.find( axis_name );
		if( axis_it == axes_map_.end() ) return _Axis();
		return axis_it->second;
	}
	
	void update( const bool & auto_publish = true )
	{
		velocity_msg_ = makePtr<geometry_msgs::Twist>::_Shared( new geometry_msgs::Twist );
		
		const auto enable_axis_it = axes_map_.find( "enable" );
		enabled_ = enable_axis_it == axes_map_.end() || enable_axis_it->second.getValue( last_joystick_message_ ) > 0;
		
		updateVelocityComponent( "linear.x", velocity_msg_->linear.x );
		updateVelocityComponent( "linear.y", velocity_msg_->linear.y );
		updateVelocityComponent( "linear.z", velocity_msg_->linear.z );
			
		updateVelocityComponent( "angular.x", velocity_msg_->angular.x );
		updateVelocityComponent( "angular.y", velocity_msg_->angular.y );
		updateVelocityComponent( "angular.z", velocity_msg_->angular.z );
		
		if( auto_publish ) publish();
	}
	
	void publish()
	{
		multi_pub_.publish( "cmd_vel", velocity_msg_ );
	}
	
	inline bool isEnabled() const
	{
		return enabled_;
	}
};

}

#endif // BASE_LIBS_BASE_LIBS_JOYSTICK_POLICY_H_
