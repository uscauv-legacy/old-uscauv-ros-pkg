/*******************************************************************************
 *
 *      seabee3_driver
 * 
 *      Copyright (c) 2010,
 *
 *      Edward T. Kaszubski (ekaszubski@gmail.com),
 *      Michael Montalbo (mmontalbo@gmail.com)
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

// tools
#include <base_robot_driver/base_robot_driver.h>
#include <control_toolbox/pid.h> // for Pid
#include <seabee3_beestem/BeeStem3.h> // for MotorControllerIDs
#include <mathy_math/mathy_math.h>
#include <string>
// msgs
#include <seabee3_driver_base/MotorCntl.h> // for outgoing thruster commands
#include <seabee3_msgs/PhysicsState.h>
// seabee3_driver/Vector3Masked.msg <-- seabee3_driver/SetDesiredPose.srv

// srvs
#include <seabee3_msgs/SetDesiredPose.h> // for SetDesiredPose
#include <std_srvs/Empty.h>

void operator *=( geometry_msgs::Vector3 & v, const double & scale )
{
	v.x = v.x * scale;
	v.y = v.y * scale;
	v.z = v.z * scale;
}

// copy @v and scale the result
geometry_msgs::Vector3 operator *( const geometry_msgs::Vector3 & v, const double & scale )
{
	geometry_msgs::Vector3 result( v );
	result *= scale;
	return result;
}

// copy @twist and scale its components
geometry_msgs::Twist operator *( const geometry_msgs::Twist & twist, const double & scale )
{
	geometry_msgs::Twist result;
	result.linear = twist.linear * scale;
	result.angular = twist.angular * scale;

	return result;
}

class Seabee3Driver: public BaseRobotDriver<>
{
public:
	// define the direction of each thruster in an array that is responsible for controlling a single axis of movement
	struct ThrusterArrayCfg
	{
		std::vector<double> thrusters;
		double & at( const unsigned int & i )
		{
			if ( thrusters.size() < i )
			{
				thrusters.resize( i );
			}
			return thrusters.at( i - 1 );
		}
	};

	struct Axes
	{
		const static int speed = 0;
		const static int strafe = 1;
		const static int depth = 2;

		const static int roll = 3;
		const static int pitch = 4;
		const static int yaw = 5;

		const static int speed_rel = 6;
		const static int strafe_rel = 7;
		const static int depth_rel = 8;

		const static int roll_rel = 9;
		const static int pitch_rel = 10;
		const static int yaw_rel = 11;
	};

	struct PidConfig
	{
		double p;
		double i;
		double d;
	};

	struct ConfiguredPid
	{
		control_toolbox::Pid pid;
		PidConfig cfg;

		inline void initPid( double i_min, double i_max )
		{
			pid.initPid( cfg.p, cfg.i, cfg.d, i_min, i_max );
		}
		;
		inline void reset()
		{
			pid.reset();
		}
		;
	};

	struct Pid3D
	{
		ConfiguredPid x, y, z;

		inline void initPid( double i_min, double i_max )
		{
			x.initPid( i_min, i_max );
			y.initPid( i_min, i_max );
			z.initPid( i_min, i_max );
		}
		;
		inline void reset()
		{
			x.reset();
			y.reset();
			z.reset();
		}
		;
	};

private:
	std::string global_frame_;

	seabee3_driver_base::MotorCntl motor_cntl_msg_;
	seabee3_msgs::PhysicsState physics_state_msg_;

	geometry_msgs::Vector3 error_in_rpy_, error_in_xyz_;
	geometry_msgs::Vector3 max_error_in_rpy_, max_error_in_xyz_;

	ros::Time last_pid_update_time_;

	tf::Transform desired_pose_tf_;
	tf::Transform current_pose_tf_;

	geometry_msgs::Twist desired_pose_;
	geometry_msgs::Twist desired_pose_velocity_;
	geometry_msgs::Twist current_pose_;

	std::vector<ThrusterArrayCfg> thruster_dir_cfg_;

	double axis_dir_cfg_[6];
	double pid_i_min_, pid_i_max_;

	Pid3D xyz_pid_, rpy_pid_;

	ros::Publisher motor_cntl_pub_;

	ros::ServiceServer reset_pose_srv_;
	ros::ServiceServer set_desired_pose_srv_;
	ros::Subscriber physics_state_sub_;

	double motor_val_min_, motor_val_max_;

public:
	Seabee3Driver( ros::NodeHandle & nh ) :
		BaseRobotDriver<> ( nh, "/seabee3/cmd_vel" )
	{
		thruster_dir_cfg_.resize( 6 );

		last_pid_update_time_ = ros::Time( -1 );

		nh_priv_.param( "global_frame", global_frame_, std::string( "/landmark_map" ) );


		// scale motor values to be within the following min+max value such that
		// |D'| = [ motor_val_min, motor_val_max ]
		nh_priv_.param( "motor_val_min", motor_val_min_, 20.0 );
		nh_priv_.param( "motor_val_max", motor_val_max_, 100.0 );

		nh_priv_.param( "speed_err_cap", max_error_in_xyz_.x, 100.0 );
		nh_priv_.param( "strafe_err_cap", max_error_in_xyz_.y, 100.0 );
		nh_priv_.param( "depth_err_cap", max_error_in_xyz_.z, 100.0 );

		nh_priv_.param( "roll_err_cap", max_error_in_rpy_.x, 25.0 );
		nh_priv_.param( "pitch_err_cap", max_error_in_rpy_.y, 25.0 );
		nh_priv_.param( "yaw_err_cap", max_error_in_rpy_.z, 25.0 );

		nh_priv_.param( "pid/pos/X/p", xyz_pid_.x.cfg.p, 2.5 );
		nh_priv_.param( "pid/pos/X/i", xyz_pid_.x.cfg.i, 0.05 );
		nh_priv_.param( "pid/pos/X/d", xyz_pid_.x.cfg.d, 0.2 );

		nh_priv_.param( "pid/pos/Y/p", xyz_pid_.y.cfg.p, 2.5 );
		nh_priv_.param( "pid/pos/Y/i", xyz_pid_.y.cfg.i, 0.05 );
		nh_priv_.param( "pid/pos/Y/d", xyz_pid_.y.cfg.d, 0.2 );

		nh_priv_.param( "pid/pos/Z/p", xyz_pid_.z.cfg.p, 2.5 );
		nh_priv_.param( "pid/pos/Z/i", xyz_pid_.z.cfg.i, 0.05 );
		nh_priv_.param( "pid/pos/Z/d", xyz_pid_.z.cfg.d, 0.2 );

		nh_priv_.param( "pid/ori/R/p", rpy_pid_.x.cfg.p, 2.5 );
		nh_priv_.param( "pid/ori/R/i", rpy_pid_.x.cfg.i, 0.05 );
		nh_priv_.param( "pid/ori/R/d", rpy_pid_.x.cfg.d, 0.2 );

		nh_priv_.param( "pid/ori/P/p", rpy_pid_.y.cfg.p, 2.5 );
		nh_priv_.param( "pid/ori/P/i", rpy_pid_.y.cfg.i, 0.05 );
		nh_priv_.param( "pid/ori/P/d", rpy_pid_.y.cfg.d, 0.2 );

		nh_priv_.param( "pid/ori/Y/p", rpy_pid_.z.cfg.p, 2.5 );
		nh_priv_.param( "pid/ori/Y/i", rpy_pid_.z.cfg.i, 0.05 );
		nh_priv_.param( "pid/ori/Y/d", rpy_pid_.z.cfg.d, 0.2 );

		nh_priv_.param( "pid/i_max", pid_i_max_, 1.0 );
		nh_priv_.param( "pid/i_min", pid_i_min_, -1.0 );

		nh_priv_.param( "speed_m1_dir", thruster_dir_cfg_[Axes::speed].at( 1 ), 1.0 );
		nh_priv_.param( "speed_m2_dir", thruster_dir_cfg_[Axes::speed].at( 2 ), 1.0 );

		nh_priv_.param( "strafe_m1_dir", thruster_dir_cfg_[Axes::strafe].at( 1 ), 1.0 );
		nh_priv_.param( "strafe_m2_dir", thruster_dir_cfg_[Axes::strafe].at( 2 ), 1.0 );

		nh_priv_.param( "depth_m1_dir", thruster_dir_cfg_[Axes::depth].at( 1 ), 1.0 );
		nh_priv_.param( "depth_m2_dir", thruster_dir_cfg_[Axes::depth].at( 2 ), 1.0 );

		nh_priv_.param( "roll_m1_dir", thruster_dir_cfg_[Axes::roll].at( 1 ), 1.0 );
		nh_priv_.param( "roll_m2_dir", thruster_dir_cfg_[Axes::roll].at( 2 ), 1.0 );

		nh_priv_.param( "pitch_m1_dir", thruster_dir_cfg_[Axes::pitch].at( 1 ), 1.0 );
		nh_priv_.param( "pitch_m2_dir", thruster_dir_cfg_[Axes::pitch].at( 2 ), 1.0 );

		nh_priv_.param( "yaw_m1_dir", thruster_dir_cfg_[Axes::yaw].at( 1 ), -1.0 );
		nh_priv_.param( "yaw_m2_dir", thruster_dir_cfg_[Axes::yaw].at( 2 ), 1.0 );

		nh_priv_.param( "speed_axis_dir", axis_dir_cfg_[Axes::speed], 1.0 );
		nh_priv_.param( "strafe_axis_dir", axis_dir_cfg_[Axes::strafe], 1.0 );
		nh_priv_.param( "depth_axis_dir", axis_dir_cfg_[Axes::depth], -1.0 );
		nh_priv_.param( "roll_axis_dir", axis_dir_cfg_[Axes::roll], -1.0 );
		nh_priv_.param( "pitch_axis_dir", axis_dir_cfg_[Axes::pitch], -1.0 );
		nh_priv_.param( "yaw_axis_dir", axis_dir_cfg_[Axes::yaw], -1.0 );

		xyz_pid_.initPid( pid_i_min_, pid_i_max_ );
		xyz_pid_.reset();

		rpy_pid_.initPid( pid_i_min_, pid_i_max_ );
		rpy_pid_.reset();

		resetMotorCntlMsg();

		motor_cntl_pub_ = nh.advertise<seabee3_driver_base::MotorCntl> ( "/seabee3/motor_cntl", 1 );
		physics_state_sub_ = nh.subscribe( "/seabee3/physics_state", 1, &Seabee3Driver::physicsStateCB, this );

		reset_pose_srv_ = nh.advertiseService( "/seabee3/reset_pose", &Seabee3Driver::resetPoseCB, this );
		set_desired_pose_srv_ = nh.advertiseService( "/seabee3/set_desired_pose", &Seabee3Driver::setDesiredPoseCB, this );


		//set current xyz to 0, desired RPY to current RPY
		resetPose();
	}

	void physicsStateCB( const seabee3_msgs::PhysicsStateConstPtr & msg )
	{
		physics_state_msg_ = *msg;
	}

	void updateMotorCntlMsg( seabee3_driver_base::MotorCntl & msg, int axis, int p_value )
	{
		int value = p_value;

		int motor1 = -1, motor2 = -1;
		double motor1_scale = 1.0, motor2_scale = 1.0;

		switch ( axis )
		{
		case Axes::speed: //relative to the robot
			motor1 = BeeStem3::MotorControllerIDs::FWD_RIGHT_THRUSTER;
			motor2 = BeeStem3::MotorControllerIDs::FWD_LEFT_THRUSTER;
			motor1_scale = thruster_dir_cfg_[Axes::speed].at( 1 );
			motor2_scale = thruster_dir_cfg_[Axes::speed].at( 2 );
			break;
		case Axes::strafe: //absolute; relative to the world
			updateMotorCntlMsg( msg, Axes::strafe_rel, value );
			//updateMotorCntlMsg(msg, Axes::strafe_rel, value * cos( LocalizationUtil::degToRad( IMUDataCache->ori.x ) ) );
			//updateMotorCntlMsg(msg, Axes::depth_rel, value * -sin( LocalizationUtil::degToRad( IMUDataCache->ori.x ) ) );
			return;
		case Axes::strafe_rel: //relative to the robot
			motor1 = BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER;
			motor2 = BeeStem3::MotorControllerIDs::STRAFE_BACK_THRUSTER;
			motor1_scale = thruster_dir_cfg_[Axes::strafe].at( 1 );
			motor2_scale = thruster_dir_cfg_[Axes::strafe].at( 2 );
			break;
		case Axes::depth: //absolute; relative to the world
			updateMotorCntlMsg( msg, Axes::depth_rel, value * -1.0 );
			//updateMotorCntlMsg(msg, Axes::depth_rel, value * -cos( LocalizationUtil::degToRad( IMUDataCache->ori.x ) ) );
			//updateMotorCntlMsg(msg, Axes::strafe_rel, value * -sin( LocalizationUtil::degToRad( IMUDataCache->ori.x ) ) );
			return;
		case Axes::depth_rel: //relative to the robot
			motor1 = BeeStem3::MotorControllerIDs::DEPTH_RIGHT_THRUSTER;
			motor2 = BeeStem3::MotorControllerIDs::DEPTH_LEFT_THRUSTER;
			motor1_scale = thruster_dir_cfg_[Axes::depth].at( 1 );
			motor2_scale = thruster_dir_cfg_[Axes::depth].at( 2 );
			break;
		case Axes::roll: //absolute; relative to the world
			motor1 = BeeStem3::MotorControllerIDs::DEPTH_RIGHT_THRUSTER;
			motor2 = BeeStem3::MotorControllerIDs::DEPTH_LEFT_THRUSTER;
			motor1_scale = thruster_dir_cfg_[Axes::roll].at( 1 );
			motor2_scale = thruster_dir_cfg_[Axes::roll].at( 2 );
			break;
		case Axes::pitch: //pitch is only available as seabee starts to roll; make sure that's the case here
			//updateMotorCntlMsg( msg, Axes::yaw_rel, value * -sin( LocalizationUtil::degToRad( IMUDataCache->ori.x ) ) );
			return;
		case Axes::yaw: //absolute; relative to the world
			//updateMotorCntlMsg(msg, Axes::yaw_rel, value * cos( LocalizationUtil::degToRad( IMUDataCache->ori.x ) ) );
			updateMotorCntlMsg( msg, Axes::yaw_rel, value );
			return;
		case Axes::yaw_rel: //relative to the robot
			motor1 = BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER;
			motor2 = BeeStem3::MotorControllerIDs::STRAFE_BACK_THRUSTER;
			motor1_scale = thruster_dir_cfg_[Axes::yaw].at( 1 );
			motor2_scale = thruster_dir_cfg_[Axes::yaw].at( 2 );
			break;
		}

		if ( motor1 < 0 || motor2 < 0 ) return;

		int motor1_val = msg.motors[motor1];
		int motor2_val = msg.motors[motor2];

		motor1_val += motor1_scale * value;
		motor2_val += motor2_scale * value;

		MathyMath::capValueProp( motor1_val, motor2_val, 100 );

		msg.motors[motor1] = motor1_val;
		msg.motors[motor2] = motor2_val;

		msg.mask[motor1] = 1;
		msg.mask[motor2] = 1;
	}

	int scaleMotorValue( int value )
	{
		//ROS_INFO( "value: %d", value );
		if ( value == 0 ) return value;

		double value_d = (double) value;

		value_d = motor_val_min_ * ( value < 0 ? -1.0 : 1.0 ) + value_d * ( motor_val_max_ - motor_val_min_ ) / 100.0;

		return (int) round( value_d );
	}

	void resetMotorCntlMsg()
	{
		for ( int i = 0; i < BeeStem3::NUM_MOTOR_CONTROLLERS; i++ )
		{
			motor_cntl_msg_.mask[i] = 0;
			motor_cntl_msg_.motors[i] = 0;
		}
	}

	void setDesiredPoseComponent( double & mask, double & desired, double & value, double & mode )
	{
		// mode = 0 -> desired = value
		// mode = 1 -> desired += value
		if ( mask != 0.0 ) desired = value + ( mode == 1.0 ? desired : 0.0 );
	}

	void setDesiredPose( geometry_msgs::Vector3 & mask, geometry_msgs::Vector3 & desired, geometry_msgs::Vector3 & value, geometry_msgs::Vector3 & mode )
	{
		setDesiredPoseComponent( mask.x, desired.x, value.x, mode.x );
		setDesiredPoseComponent( mask.y, desired.y, value.y, mode.y );
		setDesiredPoseComponent( mask.z, desired.z, value.z, mode.z );
	}

	bool resetPoseCB( std_srvs::Empty::Request & req, std_srvs::Empty::Response & resp )
	{
		resetPose();

		return true;
	}

	void resetPose()
	{
		desired_pose_ = geometry_msgs::Twist();
		current_pose_ = geometry_msgs::Twist();
	}

	bool setDesiredPoseCB( seabee3_msgs::SetDesiredPose::Request & req, seabee3_msgs::SetDesiredPose::Response & resp )
	{
		setDesiredPose( req.ori.mask, desired_pose_.angular, req.ori.values, req.ori.mode );

		geometry_msgs::Vector3 velocity = req.pos.values;

		req.pos.values.x = velocity.x * cos( desired_pose_.angular.z ) - velocity.y * sin( desired_pose_.angular.z );
		req.pos.values.y = velocity.x * sin( desired_pose_.angular.z ) + velocity.y * cos( desired_pose_.angular.z );

		setDesiredPose( req.pos.mask, desired_pose_.linear, req.pos.values, req.pos.mode );

		return true;
	}

	void requestChangeInDesiredPose( const geometry_msgs::Twist & msg )
	{
		seabee3_msgs::SetDesiredPose::Request req;
		seabee3_msgs::SetDesiredPose::Response resp;
		seabee3_msgs::Vector3Masked change_in_desired_pose;

		change_in_desired_pose.mask.x = change_in_desired_pose.mask.y = change_in_desired_pose.mask.z = 1; //enable all values
		change_in_desired_pose.mode.x = change_in_desired_pose.mode.y = change_in_desired_pose.mode.z = 1; //set mode to incremental

		req.pos = req.ori = change_in_desired_pose;

		req.pos.values = msg.linear;
		req.ori.values = msg.angular;

		setDesiredPoseCB( req, resp );
	}

	void pidStep()
	{
		desired_pose_velocity_ = twist_cache_;


		//fetchTfFrame( desired_pose_tf_, "landmark_map", "seabee3/desired_pose" );
		fetchTfFrame( current_pose_tf_, global_frame_, "seabee3/base_link" );


		//convert tf frames to Twist messages; fuck quaternions
		//desired_pose_tf_ >> desired_pose_;
		current_pose_tf_ >> current_pose_;

		if ( last_pid_update_time_ != ros::Time( -1 ) )
		{
			resetMotorCntlMsg();

			ros::Duration dt = ros::Time::now() - last_pid_update_time_;


			//twistCache is essentially the linear and angular change in desired pose per second (linear + angular velocity)
			//we multiply by the change in time to obtain the desired change in pose
			//this change in pose is then added to the current pose; the result is desiredPose
			const double t1 = dt.toSec();
			geometry_msgs::Twist change_in_desired_pose = twist_cache_ * t1;
			requestChangeInDesiredPose( change_in_desired_pose );

			desired_pose_ >> desired_pose_tf_;
			publishTfFrame( desired_pose_tf_, global_frame_, "/seabee3/desired_pose" );

			MathyMath::normalizeAngle( desired_pose_.angular.x );
			MathyMath::normalizeAngle( desired_pose_.angular.y );
			MathyMath::normalizeAngle( desired_pose_.angular.z );

			MathyMath::normalizeAngle( current_pose_.angular.x );
			MathyMath::normalizeAngle( current_pose_.angular.x );
			MathyMath::normalizeAngle( current_pose_.angular.x );

			error_in_xyz_.x = desired_pose_.linear.x - current_pose_.linear.x;
			error_in_xyz_.y = desired_pose_.linear.y - current_pose_.linear.y;
			error_in_xyz_.z = desired_pose_.linear.z - current_pose_.linear.z;

			error_in_rpy_.x = MathyMath::angleDistRel( MathyMath::radToDeg( desired_pose_.angular.x ), MathyMath::radToDeg( current_pose_.angular.x ) );
			error_in_rpy_.y = MathyMath::angleDistRel( MathyMath::radToDeg( desired_pose_.angular.y ), MathyMath::radToDeg( current_pose_.angular.y ) );
			error_in_rpy_.z = MathyMath::angleDistRel( MathyMath::radToDeg( desired_pose_.angular.z ), MathyMath::radToDeg( current_pose_.angular.z ) );


			//printf("error x %f y %f z %f r %f p %f y %f\n", error_in_xyz_.x, error_in_xyz_.y, error_in_xyz_.z, error_in_rpy_.x, error_in_rpy_.y, error_in_rpy_.z );

			MathyMath::capValue( error_in_xyz_.x, max_error_in_xyz_.x );
			MathyMath::capValue( error_in_xyz_.y, max_error_in_xyz_.y );
			MathyMath::capValue( error_in_xyz_.z, max_error_in_xyz_.z );

			MathyMath::capValue( error_in_rpy_.x, max_error_in_rpy_.x );
			MathyMath::capValue( error_in_rpy_.y, max_error_in_rpy_.y );
			MathyMath::capValue( error_in_rpy_.z, max_error_in_rpy_.z );


			//			printf( "error x %f y %f z %f r %f p %f y %f\n", error_in_xyz_.x, error_in_xyz_.y, error_in_xyz_.z, error_in_rpy_.x, error_in_rpy_.y, error_in_rpy_.z );

			geometry_msgs::Twist error_in_velocity;
			error_in_velocity.linear.x = physics_state_msg_.velocity.linear.x - desired_pose_velocity_.linear.x;
			error_in_velocity.linear.y = physics_state_msg_.velocity.linear.y - desired_pose_velocity_.linear.y;
			error_in_velocity.linear.z = physics_state_msg_.velocity.linear.z - desired_pose_velocity_.linear.z;

			error_in_velocity.angular.x = physics_state_msg_.velocity.angular.x - desired_pose_velocity_.angular.x;
			error_in_velocity.angular.y = physics_state_msg_.velocity.angular.y - desired_pose_velocity_.angular.y;
			error_in_velocity.angular.z = physics_state_msg_.velocity.angular.z - desired_pose_velocity_.angular.z;


			// error in work is the total work able to be done over the available distance minus the work required to stop the robot at the desired location
			// W=Fx=mv^2/2
			geometry_msgs::Vector3 error_in_work;
			error_in_work.x = 10.0 * error_in_xyz_.x - physics_state_msg_.mass.linear.x * ( error_in_velocity.linear.x < 0 ? -1.0 : 1.0 ) * pow( error_in_velocity.linear.x, 2 ) / 2.0;
			error_in_work.y = 10.0 * error_in_xyz_.y - physics_state_msg_.mass.linear.x * ( error_in_velocity.linear.y < 0 ? -1.0 : 1.0 ) * pow( error_in_velocity.linear.y, 2 ) / 2.0;
			error_in_work.z = 5.0 * error_in_xyz_.z - physics_state_msg_.mass.linear.x * ( error_in_velocity.linear.z < 0 ? -1.0 : 1.0 ) * pow( error_in_velocity.linear.z, 2 ) / 2.0;


			// Fx_min=mv^2/2; minimum stopping distnace x_min=mv^2/2F
			// error in distance x_err=x-x_min
			/*geometry_msgs::Vector3 error_in_distance;
			 error_in_distance.x = error_in_xyz_.x - physics_state_msg_.mass.linear.x * ( error_in_velocity.linear.x < 0 ? -1.0 : 1.0 ) * pow( error_in_velocity.linear.x, 2 ) / ( 2.0 * 10.0 );
			 error_in_distance.y = error_in_xyz_.y - physics_state_msg_.mass.linear.x * ( error_in_velocity.linear.y < 0 ? -1.0 : 1.0 ) * pow( error_in_velocity.linear.y, 2 ) / ( 2.0 * 10.0 );
			 error_in_distance.z = error_in_xyz_.z - physics_state_msg_.mass.linear.x * ( error_in_velocity.linear.z < 0 ? -1.0 : 1.0 ) * pow( error_in_velocity.linear.z, 2 ) / ( 2.0 * 5.0 );*/

			double speed_motor_val = axis_dir_cfg_[Axes::speed] * xyz_pid_.x.pid.updatePid( error_in_work.x, dt );
			double strafe_motor_val = axis_dir_cfg_[Axes::strafe] * xyz_pid_.y.pid.updatePid( error_in_work.y, dt );
			double depth_motor_val = axis_dir_cfg_[Axes::depth] * xyz_pid_.z.pid.updatePid( error_in_work.z, dt );


			//double rollMotorVal = axis_dir_cfg_[Axes::roll] * rpy_pid_.x.pid.updatePid( error_in_rpy_.x, dt );
			//double pitchMotorVal = axis_dir_cfg_[Axes::pitch] * rpy_pid_.y.pid.updatePid( error_in_rpy_.y, dt );

			//printf("error_in_velocity.angular.z %f\n", error_in_velocity.angular.z);
			//printf("error_in_rpy.z %f\n", MathyMath::degToRad( error_in_rpy_.z ) );

			//printf("w1 %f\n", 5.0 * 0.3 * MathyMath::degToRad( error_in_rpy_.z ));
			//printf("w2 %f\n", 100.0 * physics_state_msg_.mass.angular.z * ( error_in_velocity.angular.z < 0 ? -1.0 : 1.0 ) * pow( error_in_velocity.angular.z, 2 ) / 2.0);

			// W=Fx=mv^2/2
			double error_in_work_yaw = 5.0 * 0.3 * MathyMath::degToRad( error_in_rpy_.z ) - 100.0 * physics_state_msg_.mass.angular.z * ( error_in_velocity.angular.z < 0 ? 1.0 : -1.0 ) * pow(
					error_in_velocity.angular.z, 2 ) / 2.0;
			double yaw_motor_val = axis_dir_cfg_[Axes::yaw] * rpy_pid_.z.pid.updatePid( error_in_work_yaw, dt );


			//printf( "speed %f strafe %f depth %f yaw %f\n", speed_motor_val, strafe_motor_val, depth_motor_val, yaw_motor_val );


			//MathyMath::capValue( rollMotorVal, 50.0 );
			//MathyMath::capValue( pitchMotorVal, 50.0 );

			MathyMath::capValue( yaw_motor_val, 50.0 );
			MathyMath::capValue( depth_motor_val, 50.0 );

			updateMotorCntlMsg( motor_cntl_msg_, Axes::speed, speed_motor_val );
			updateMotorCntlMsg( motor_cntl_msg_, Axes::strafe, strafe_motor_val );
			updateMotorCntlMsg( motor_cntl_msg_, Axes::depth, depth_motor_val );
			//updateMotorCntlMsg( motorCntlMsg, Axes::roll, rollMotorVal );
			//updateMotorCntlMsg( motorCntlMsg, Axes::pitch, pitchMotorVal );
			updateMotorCntlMsg( motor_cntl_msg_, Axes::yaw, yaw_motor_val );
		}
		last_pid_update_time_ = ros::Time::now();
	}

	virtual void spinOnce()
	{
		//this also grabs and publishes tf frames
		pidStep();


		/*for ( size_t i; i < motor_cntl_msg_.motors.size(); i++ )
		 {
		 motor_cntl_msg_.motors[i] = scaleMotorValue( motor_cntl_msg_.motors[i] );
		 }*/

		motor_cntl_pub_.publish( motor_cntl_msg_ );

		ros::Rate( 20 ).sleep();
	}
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "seabee3_driver" );
	ros::NodeHandle nh;
	
	Seabee3Driver seabee3_driver( nh );
	seabee3_driver.spin( SpinModeId::loop_spin_once );

	return 0;
}
