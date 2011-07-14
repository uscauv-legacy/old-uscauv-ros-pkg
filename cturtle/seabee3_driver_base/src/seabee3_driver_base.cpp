/*******************************************************************************
 *
 *      seabee3_driver_base
 * 
 *      Copyright (c) 2010,
 *      Edward T. Kaszubski (ekaszubski@gmail.com)
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

//tools
#include <base_node/base_node.h>
#include <seabee3_beestem/BeeStem3Driver.h> //for BeeStem3Driver, control_common::Axes
#include <string>
//msgs
#include <seabee3_driver_base/Depth.h> // for outgoing Depth
#include <seabee3_driver_base/KillSwitch.h> // for outgoing KillSwitch
#include <seabee3_driver_base/MotorCntl.h> // for incoming MotorCntl
#include <seabee3_driver_base/Pressure.h> // for outgoing Pressure
//srvs
#include <seabee3_driver_base/FiringDeviceAction.h> // for FiringDeviceAction
#include <common_utils/filters.h>

using namespace movement_common;
class Seabee3DriverBase: public BaseNode<>
{
public:
	typedef movement_common::ThrusterArrayCfg _ThrusterArrayCfg;

private:
	ros::Subscriber motor_cntl_sub_;

	ros::Publisher intl_pressure_pub_;
	ros::Publisher extl_pressure_pub_;
	ros::Publisher depth_pub_;
	ros::Publisher kill_switch_pub_;

	ros::ServiceServer dropper1_action_srv_;
	ros::ServiceServer dropper2_action_srv_;
	ros::ServiceServer shooter1_action_srv_;
	ros::ServiceServer shooter2_action_srv_;

//	filters::MovingAverageFilter<float, 5> depth_filter_;
	BeeStem3Driver * bee_stem_3_driver_;
	std::string port_;
	double surface_pressure_;
	double montalbos_per_meter_;
	bool pressure_calibrated_;

	// used to decode motor values
	_ThrusterArrayCfg thruster_dir_cfg_;

	int min_motor_value_;

public:
	//#define SURFACE_PRESSURE 908

	Seabee3DriverBase( ros::NodeHandle & nh ) :
		BaseNode<> ( nh )
	{
		pressure_calibrated_ = false;

		nh_local_.param( "surface_pressure", surface_pressure_, 900.0 );
		nh_local_.param( "montalbos_per_meter", montalbos_per_meter_, 35.7 );

		nh_local_.param( "port", port_, std::string( "/dev/ttyUSB0" ) );

		ROS_INFO( "constructing new driver instance" );
		bee_stem_3_driver_ = new BeeStem3Driver( port_ );

		nh_local_.param( "shooter1/trigger_time", bee_stem_3_driver_->shooter1_params_.trigger_time_, 50 );
		nh_local_.param( "shooter1/trigger_value", bee_stem_3_driver_->shooter1_params_.trigger_value_, -75 );

		nh_local_.param( "shooter2/trigger_time", bee_stem_3_driver_->shooter2_params_.trigger_time_, 50 );
		nh_local_.param( "shooter2/trigger_value", bee_stem_3_driver_->shooter2_params_.trigger_value_, 75 );

		nh_local_.param( "dropper1/trigger_time", bee_stem_3_driver_->dropper1_params_.trigger_time_, 50 );
		nh_local_.param( "dropper1/trigger_value", bee_stem_3_driver_->dropper1_params_.trigger_value_, 40 );

		nh_local_.param( "dropper2/trigger_time", bee_stem_3_driver_->dropper2_params_.trigger_time_, 50 );
		nh_local_.param( "dropper2/trigger_value", bee_stem_3_driver_->dropper2_params_.trigger_value_, 40 );

		// try to grab this param from the high-level driver so the values are synced
		nh_local_.param( "/seabee3_driver/min_motor_value_", min_motor_value_, 15 );

		double thruster_dir;

		nh_local_.param( "speed_m1_dir", thruster_dir, 1.0 );
		thruster_dir_cfg_[MotorControllerIDs::FWD_LEFT_THRUSTER] = thruster_dir;
		nh_local_.param( "speed_m2_dir", thruster_dir, 1.0 );
		thruster_dir_cfg_[MotorControllerIDs::FWD_RIGHT_THRUSTER] = thruster_dir;

		nh_local_.param( "strafe_m1_dir", thruster_dir, 1.0 );
		thruster_dir_cfg_[MotorControllerIDs::STRAFE_FRONT_THRUSTER] = thruster_dir;
		nh_local_.param( "strafe_m2_dir", thruster_dir, 1.0 );
		thruster_dir_cfg_[MotorControllerIDs::STRAFE_BACK_THRUSTER] = thruster_dir;

		nh_local_.param( "depth_m1_dir", thruster_dir, 1.0 );
		thruster_dir_cfg_[MotorControllerIDs::DEPTH_LEFT_THRUSTER] = thruster_dir;
		nh_local_.param( "depth_m2_dir", thruster_dir, 1.0 );
		thruster_dir_cfg_[MotorControllerIDs::DEPTH_RIGHT_THRUSTER] = thruster_dir;

		motor_cntl_sub_ = nh.subscribe( "/seabee3/motor_cntl", 1, &Seabee3DriverBase::motorCntlCB, this );

		intl_pressure_pub_ = nh.advertise<seabee3_driver_base::Pressure> ( "/seabee3/intl_pressure", 1 );
		extl_pressure_pub_ = nh.advertise<seabee3_driver_base::Pressure> ( "/seabee3/extl_pressure", 1 );
		depth_pub_ = nh.advertise<seabee3_driver_base::Depth> ( "/seabee3/depth", 1 );
		kill_switch_pub_ = nh.advertise<seabee3_driver_base::KillSwitch> ( "/seabee3/kill_switch", 1 );

		dropper1_action_srv_ = nh.advertiseService( "/seabee3/dropper1_action", &Seabee3DriverBase::dropper1ActionCB, this );
		dropper2_action_srv_ = nh.advertiseService( "/seabee3/dropper2_action", &Seabee3DriverBase::dropper2ActionCB, this );
		shooter1_action_srv_ = nh.advertiseService( "/seabee3/shooter1_action", &Seabee3DriverBase::shooter1ActionCB, this );
		shooter2_action_srv_ = nh.advertiseService( "/seabee3/shooter2_action", &Seabee3DriverBase::shooter2ActionCB, this );
	}

	float getDepthFromPressure( int pressure )
	{
		return (float) ( pressure - surface_pressure_ ) / montalbos_per_meter_;
	}

	void motorCntlCB( const seabee3_driver_base::MotorCntlConstPtr & msg )
	{
		int dir;
		for ( unsigned int i = 0; i < msg->motors.size(); i++ )
		{
			dir = i < movement_common::MotorControllerIDs::SHOOTER ? thruster_dir_cfg_[i] : 1.0;
			if ( msg->mask[i] == 1 ) bee_stem_3_driver_->setThruster( i, abs( msg->motors[i] ) >= min_motor_value_ ? dir * msg->motors[i] : 0 );
		}
	}

	bool executeFiringDeviceAction( seabee3_driver_base::FiringDeviceAction::Request &req, seabee3_driver_base::FiringDeviceAction::Response &res, int device_id )
	{
		bool & device_status = bee_stem_3_driver_->getDeviceStatus( device_id );
		switch ( req.action )
		{
		case seabee3_driver_base::FiringDeviceAction::Request::CHECK_STATUS:
			break;
		case seabee3_driver_base::FiringDeviceAction::Request::RESET_STATUS:
			device_status = true;
			break;
		case seabee3_driver_base::FiringDeviceAction::Request::FIRE:
			bee_stem_3_driver_->fireDevice( device_id );
			break;
		}
		res.is_loaded = device_status;
		return true;
	}

	bool dropper1ActionCB( seabee3_driver_base::FiringDeviceAction::Request &req, seabee3_driver_base::FiringDeviceAction::Response &res )
	{
		return executeFiringDeviceAction( req, res, FiringDeviceIDs::dropper_stage1 );
	}

	bool dropper2ActionCB( seabee3_driver_base::FiringDeviceAction::Request &req, seabee3_driver_base::FiringDeviceAction::Response &res )
	{
		return executeFiringDeviceAction( req, res, FiringDeviceIDs::dropper_stage2 );
	}

	bool shooter1ActionCB( seabee3_driver_base::FiringDeviceAction::Request &req, seabee3_driver_base::FiringDeviceAction::Response &res )
	{
		return executeFiringDeviceAction( req, res, FiringDeviceIDs::shooter1 );
	}

	bool shooter2ActionCB( seabee3_driver_base::FiringDeviceAction::Request &req, seabee3_driver_base::FiringDeviceAction::Response &res )
	{
		return executeFiringDeviceAction( req, res, FiringDeviceIDs::shooter2 );
	}

	virtual void spinOnce()
	{
		seabee3_driver_base::Pressure intl_pressure_msg;
		seabee3_driver_base::Pressure extl_pressure_msg;
		seabee3_driver_base::Depth depth_msg;
		seabee3_driver_base::KillSwitch kill_switch_msg;

		bee_stem_3_driver_->readPressure( intl_pressure_msg.value, extl_pressure_msg.value );

		if ( !pressure_calibrated_ )
		{
			surface_pressure_ = extl_pressure_msg.value;
			pressure_calibrated_ = true;
		}

		bee_stem_3_driver_->readKillSwitch( kill_switch_msg.is_killed );
//		depth_msg.value = depth_filter_.update( getDepthFromPressure( extl_pressure_msg.value ) );
		depth_msg.value = getDepthFromPressure( extl_pressure_msg.value );
		intl_pressure_pub_.publish( intl_pressure_msg );
		extl_pressure_pub_.publish( extl_pressure_msg );
		depth_pub_.publish( depth_msg );
		kill_switch_pub_.publish( kill_switch_msg );
	}

};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "seabee3_driver_base" );
	ros::NodeHandle nh( "~" );
	
	Seabee3DriverBase seabee3_driver_base( nh );
	seabee3_driver_base.spin();

	return 0;
}
