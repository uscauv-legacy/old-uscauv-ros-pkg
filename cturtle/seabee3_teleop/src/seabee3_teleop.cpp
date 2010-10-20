/*******************************************************************************
 *
 *      seabee3_teleop
 * 
 *      Copyright (c) 2010, Edward T. Kaszubski (ekaszubski@gmail.com)
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
#include <seabee3_beestem/BeeStem3Driver.h>
#include <seabee3_driver_base/FiringDeviceAction.h>

class Seabee3Teleop: public BaseTeleop
{
private:
	int speed_, strafe_, surface_, dive_, heading_, roll_, f_dev_inc_, f_dev_dec_, fire_dev_, current_device_;
	double speed_scale_, strafe_scale_, surface_scale_, dive_scale_, heading_scale_, roll_scale_;

	ros::ServiceClient shooter_srv_, dropper1_srv_, dropper2_srv_;
	seabee3_driver_base::FiringDeviceAction device_action_;
	bool button_action_busy_ = false;
	int button_action_id_ = -1;

public:
	Seabee3Teleop( ros::NodeHandle & nh ) :
		BaseTeleop( nh, "/seabee3/cmd_vel" ), current_device_( 0 )
	{
		nh_priv_.param( "speed", speed_, 0 );
		nh_priv_.param( "strafe", strafe_, 1 );
		nh_priv_.param( "surface", surface_, 2 );
		nh_priv_.param( "dive", dive_, 5 );
		nh_priv_.param( "heading", heading_, 4 );
		nh_priv_.param( "roll", roll_, 3 );
		nh_priv_.param( "next_firing_device", f_dev_inc_, 0 );
		nh_priv_.param( "prev_firing_device", f_dev_dec_, 1 );
		nh_priv_.param( "fire_defice", fire_dev_, 5 );

		nh_priv_.param( "speed_scale", speed_scale_, 1.0 );
		nh_priv_.param( "strafe_scale", strafe_scale_, 1.0 );
		nh_priv_.param( "surface_scale", surface_scale_, 1.0 );
		nh_priv_.param( "dive_scale", dive_scale_, 1.0 );
		nh_priv_.param( "heading_scale", heading_scale_, 1.0 );
		nh_priv_.param( "roll_scale", roll_scale_, 1.0 );

		shooter_srv_ = nh_priv_.serviceClient<seabee3_driver_base::FiringDeviceAction> ( "/seabee3/shooter1_action" );
		dropper1_srv_ = nh_priv_.serviceClient<seabee3_driver_base::FiringDeviceAction> ( "/seabee3/dropper1_action" );
		dropper2_srv_ = nh_priv_.serviceClient<seabee3_driver_base::FiringDeviceAction> ( "/seabee3/dropper2_action" );
	}

	void fireCurrentDevice( bool reset = false )
	{
		int mode = reset ? 1 : 0;
		device_action_.request.action = mode;
		switch ( current_device_ )
		{
		case BeeStem3Driver::FiringDeviceID::shooter:
			shooter_srv_.call( device_action_ );
			break;
		case BeeStem3Driver::FiringDeviceID::dropper_stage1:
			dropper1_srv_.call( device_action_ );
			break;
		case BeeStem3Driver::FiringDeviceID::dropper_stage2:
			dropper2_srv_.call( device_action_ );
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

		current_device_ = current_device_ > BeeStem3Driver::FiringDeviceID::dropper_stage2 ? 0 : current_device_ < 0 ? BeeStem3Driver::FiringDeviceID::dropper_stage2 : current_device_;

		ROS_INFO( "current device %d", current_device_ );

		if ( fire )
		{
			fireCurrentDevice();
		}
	}

	virtual void joyCB( const joy::Joy::ConstPtr& joy )
	{
		if ( button_action_busy_ )
		{
			if ( joy->buttons[button_action_id_] == 0 )
			{
				button_action_busy_ = false;
			}
		}
		else
		{
			button_action_busy_ = joy->buttons[f_dev_inc_] == 1 || joy->buttons[f_dev_dec_] == 1 || joy->buttons[fire_dev_] == 1;
			button_action_id_ = joy->buttons[f_dev_inc_] == 1 ? f_dev_inc_ : button_action_id_;
			button_action_id_ = joy->buttons[f_dev_dec_] == 1 ? f_dev_dec_ : button_action_id_;
			button_action_id_ = joy->buttons[fire_dev_] == 1 ? fire_dev_ : button_action_id_;
			if ( button_action_busy_ )
			{
				updateFiringDevices( joy->buttons[f_dev_inc_] == 1 ? 0 : joy->buttons[f_dev_dec_] == 1 ? 1 : joy->buttons[fire_dev_] == 1 ? 2 : -1 );
			}
		}

		double joy_speed = applyDeadZone( (double) ( joy->axes[speed_] ) );
		double joy_strafe = applyDeadZone( (double) ( joy->axes[strafe_] ) );
		double joy_dive = applyDeadZone( (double) ( joy->axes[dive_] ) );
		double joy_surface = applyDeadZone( (double) ( joy->axes[surface_] ) );
		double joy_heading = applyDeadZone( (double) ( joy->axes[heading_] ) );
		double joy_roll = applyDeadZone( (double) ( joy->axes[roll_] ) );

		cmd_vel_.linear.x = speed_scale_ * joy_speed; //speed
		cmd_vel_.linear.y = strafe_scale_ * joy_strafe; //strafe
		cmd_vel_.linear.z = 0.5 * (float) ( dive_scale_ * joy_dive - surface_scale_ * joy_surface ); //dive - surface

		cmd_vel_.angular.x = roll_scale_ * joy_roll; //roll
		cmd_vel_.angular.y = 0;
		cmd_vel_.angular.z = heading_scale_ * joy_heading; //heading

		cmd_vel_pub_->publish( cmd_vel_ );
	}
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "seabee3_teleop" );
	ros::NodeHandle n( "~" );
	
	Seabee3Teleop seabee3_teleop( nh );
	seabee3_teleop.spin();

	return 0;
}

