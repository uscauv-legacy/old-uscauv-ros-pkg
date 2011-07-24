/*******************************************************************************
 *
 *      seabee3_teleop
 * 
 *      Copyright (c) 2010
 *
 *      Edward T. Kaszubski (ekaszubski@gmail.com)
 *
 *      All rights reserved.
 *
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions are
 *      met:
 *      
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following disclaimer
 *        in the documentation and/or other materials provided with the
 *        distribution.
 *      * Neither the name of the USC Underwater Robotics Team nor the names of its
 *        contributors may be used to endorse or promote products derived from
 *        this software without specific prior written permission.
 *      
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *      A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#include <base_teleop/base_teleop.h>
#include <seabee3_common/movement_common.h>
#include <seabee3_driver_base/FiringDeviceAction.h>
#include <common_utils/math.h>
#include <std_srvs/Empty.h>

class Seabee3Teleop: public BaseTeleop<>
{
public:
	//typedef movement_common::FiringDeviceIDs _FiringDeviceIDs;

private:
	// button indices
	int dead_man_, f_dev_inc_, f_dev_dec_, reset_pose_, current_device_;
	// axis indices
	int speed_, strafe_, depth_, heading_, roll_, fire_dev_;
	// axis scales
	double speed_scale_, strafe_scale_, depth_scale_, heading_scale_, roll_scale_;

	double keep_alive_period_;

	geometry_msgs::Twist last_cmd_vel_;

	ros::ServiceClient shooter1_cli_, shooter2_cli_, dropper1_cli_, dropper2_cli_, reset_pose_cli_;
	seabee3_driver_base::FiringDeviceAction device_action_;
	bool button_action_busy_, keep_alive_;
	int button_action_id_, axis_action_id_;
	ros::Timer timer_;

public:
	Seabee3Teleop( ros::NodeHandle & nh ) :
		BaseTeleop<>( nh, "/seabee3/cmd_vel" ), current_device_( 0 ), button_action_busy_( false ), button_action_id_( -1 ), axis_action_id_( -1 )
	{
		nh_local_.param( "speed", speed_, 1 );
		nh_local_.param( "strafe", strafe_, 0 );
		nh_local_.param( "depth", depth_, 4 );
		nh_local_.param( "heading", heading_, 3 );

		nh_local_.param( "dead_man", dead_man_, 4 );
		nh_local_.param( "next_firing_device", f_dev_inc_, 0 );
		nh_local_.param( "prev_firing_device", f_dev_dec_, 1 );
		nh_local_.param( "fire_device", fire_dev_, 5 );
		nh_local_.param( "reset_pose", reset_pose_, 9 );

		nh_local_.param( "speed_scale", speed_scale_, 1.0 );
		nh_local_.param( "strafe_scale", strafe_scale_, 1.0 );
		nh_local_.param( "depth_scale", depth_scale_, 1.0 );
		nh_local_.param( "heading_scale", heading_scale_, 1.0 );

		nh_local_.param( "keep_alive_period", keep_alive_period_, 0.25 );

		shooter1_cli_ = nh_local_.serviceClient<seabee3_driver_base::FiringDeviceAction> ( "/seabee3/shooter1_action" );
		shooter2_cli_ = nh_local_.serviceClient<seabee3_driver_base::FiringDeviceAction> ( "/seabee3/shooter2_action" );
		dropper1_cli_ = nh_local_.serviceClient<seabee3_driver_base::FiringDeviceAction> ( "/seabee3/dropper1_action" );
		dropper2_cli_ = nh_local_.serviceClient<seabee3_driver_base::FiringDeviceAction> ( "/seabee3/dropper2_action" );
		reset_pose_cli_ = nh_local_.serviceClient<std_srvs::Empty> ( "/seabee3/reset_pose" );

		timer_ = nh_local_.createTimer( ros::Duration( keep_alive_period_ ), &Seabee3Teleop::keepAlive, this );
		timer_.start();
	}

	void fireCurrentDevice( bool reset = false )
	{
		device_action_.request.action = seabee3_driver_base::FiringDeviceAction::Request::FIRE;

		ROS_INFO("Firing device: %d", current_device_ );

		switch ( current_device_ )
		{
		case movement_common::FiringDeviceIDs::shooter1:
			shooter1_cli_.call( device_action_ );
			break;
		case movement_common::FiringDeviceIDs::shooter2:
			shooter2_cli_.call( device_action_ );
			break;
		case movement_common::FiringDeviceIDs::dropper_stage1:
			dropper1_cli_.call( device_action_ );
			break;
		case movement_common::FiringDeviceIDs::dropper_stage2:
			dropper2_cli_.call( device_action_ );
			break;
		}
	}
	
	void updateFiringDevices( int action = -1 )
	{
		bool fire = false;
		switch ( action )
		{
		case 0:
			current_device_++;
			break;
		case 1:
			current_device_--;
			break;
		case 2:
			fire = true;
			break;
		}

		current_device_ = current_device_ > movement_common::NUM_FIRING_DEVICES - 1 ? 0 : current_device_ < 0 ? movement_common::NUM_FIRING_DEVICES - 1 : current_device_;

		ROS_INFO( "current device %d", current_device_ );

		if ( fire ) fireCurrentDevice();
	}

	void resetPose()
	{
		std_srvs::Empty::Request req;
		std_srvs::Empty::Response resp;

		reset_pose_cli_.call( req, resp );
	}

	virtual void joyCB( const joy::Joy::ConstPtr& joy )
	{
		if ( joy->buttons[reset_pose_] == 1 ) resetPose();

		keep_alive_ = false;

		if ( joy->buttons[dead_man_] != 1 ) return;

		keep_alive_ = true;

		if ( button_action_busy_ )
		{
			if ( ( button_action_id_ >= 0 && joy->buttons[button_action_id_] == 0 ) || ( axis_action_id_ >= 0 && joy->axes[axis_action_id_] >= -0.3 ) )
			{
				button_action_busy_ = false;
				button_action_id_ = -1;
				axis_action_id_ = -1;
			}
		}
		else
		{
			button_action_busy_ = joy->buttons[f_dev_inc_] == 1 || joy->buttons[f_dev_dec_] == 1 || joy->axes[fire_dev_] < -0.3;
			button_action_id_ = joy->buttons[f_dev_inc_] == 1 ? f_dev_inc_ : button_action_id_;
			button_action_id_ = joy->buttons[f_dev_dec_] == 1 ? f_dev_dec_ : button_action_id_;
			axis_action_id_ = joy->axes[fire_dev_] < -0.3 ? fire_dev_ : axis_action_id_;

			if( axis_action_id_ >= 0 )
				button_action_id_ = -1;

			if ( button_action_busy_ )
			{
				updateFiringDevices( joy->buttons[f_dev_inc_] == 1 ? 0 : joy->buttons[f_dev_dec_] == 1 ? 1 : joy->axes[fire_dev_] < -0.3 ? 2 : -1 );
			}
		}

		double joy_speed = applyDeadZone( joy->axes[speed_] );
		double joy_strafe = applyDeadZone( joy->axes[strafe_] );
		double joy_depth = applyDeadZone( joy->axes[depth_] );
		double joy_heading = applyDeadZone( joy->axes[heading_] );

		cmd_vel_.linear.x = speed_scale_ * joy_speed; //speed
		cmd_vel_.linear.y = strafe_scale_ * joy_strafe; //strafe
		cmd_vel_.linear.z = depth_scale_ * joy_depth;

		cmd_vel_.angular.z = math_utils::degToRad( heading_scale_ ) * joy_heading; //heading

		last_cmd_vel_ = cmd_vel_;

		cmd_vel_pub_.publish( cmd_vel_ );
	}

	virtual void keepAlive( const ros::TimerEvent & evt )
	{
		if ( keep_alive_ ) cmd_vel_pub_.publish( last_cmd_vel_ );
	}
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "seabee3_teleop" );
	ros::NodeHandle nh( "~" );
	
	Seabee3Teleop seabee3_teleop( nh );
	seabee3_teleop.spin();

	return 0;
}

