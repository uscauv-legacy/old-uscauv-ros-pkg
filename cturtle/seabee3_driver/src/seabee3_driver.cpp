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
#include <seabee3_common/movement_common.h> // for MotorControllerIDs
#include <seabee3_common/control_common.h> // for Pid6D
#include <common_utils/math.h>
#include <string>
// msgs
#include <seabee3_driver_base/MotorCntl.h> // for outgoing thruster commands
#include <seabee3_common/PhysicsState.h>
// seabee3_driver/Vector3Masked.msg <-- seabee3_driver/SetDesiredPose.srv

// srvs
#include <seabee3_common/SetDesiredPose.h> // for SetDesiredPose
#include <std_srvs/Empty.h>

using namespace movement_common;

typedef double __StorageDataType;
typedef double __TimeDataType;

class Seabee3Driver : public BaseRobotDriver<>
{
public:

	typedef movement_common::AxisArrayCfg _AxisArrayCfg;

	typedef ConfiguredPidBase<__StorageDataType, __TimeDataType >_PidBaseType;

	typedef Pid<_PidBaseType, 1> _ConfiguredPid;
	typedef Pid3D<_PidBaseType> _ConfiguredPid3D;
	typedef Pid6D<_PidBaseType> _ConfiguredPid6D;

	typedef _ConfiguredPid _Pid;
	typedef _ConfiguredPid3D _Pid3D;
	typedef _ConfiguredPid6D _Pid6D;

	typedef _PidBaseType::Settings _Settings;
	typedef std::vector<_Settings> _SettingsArray;

	// how to interpret incoming CmdVel messages; any type except for 'pid' sets thruster values directly
			struct CmdVelConversionType
			{
				const static int pid = 0;
				const static int linear = 1;
			};

		private:

			std::string global_frame_;

			seabee3_driver_base::MotorCntl motor_cntl_msg_;
			seabee3_common::PhysicsState physics_state_msg_;

			geometry_msgs::Vector3 error_in_rpy_, error_in_xyz_;
			geometry_msgs::Vector3 max_error_in_rpy_, max_error_in_xyz_;

			ros::Time last_pid_update_time_;

			tf::Transform desired_pose_tf_;
			tf::Transform current_pose_tf_;

			geometry_msgs::Twist desired_pose_;
			geometry_msgs::Twist desired_pose_velocity_;
			geometry_msgs::Twist current_pose_;

			double axis_dir_cfg_[6];
			double pid_i_min_, pid_i_max_;

			_Pid6D pid_controller_;

			ros::Publisher motor_cntl_pub_;

			ros::ServiceServer reset_pose_srv_;
			ros::ServiceServer set_desired_pose_srv_;
			ros::Subscriber physics_state_sub_;

			double motor_val_deadzone_, motor_val_min_, motor_val_max_;

			bool use_pid_assist_;

			int cmd_vel_conversion_mode_;

			std::map<int, double> cmd_vel_conversions;

			// used to encode motor values
			std::vector<_AxisArrayCfg> motors_dir_cfg_;

			const static _Settings settings_array[6];

		public:
			Seabee3Driver( ros::NodeHandle & nh ) :
			BaseRobotDriver<> ( nh, "/seabee3/cmd_vel" )

			{
				last_pid_update_time_ = ros::Time( -1 );

				motors_dir_cfg_.resize( 6 );

				nh_local_.param( "global_frame", global_frame_, std::string( "/landmark_map" ) );

				nh_local_.param( "cmd_vel_conversion", cmd_vel_conversion_mode_, CmdVelConversionType::pid );

				nh_local_.param( "use_pid_assist", use_pid_assist_, false );

				if ( !nh.getParam( "/seabee3_teleop/speed_scale", cmd_vel_conversions[Axes::speed] ) ) cmd_vel_conversions[Axes::speed] = 1.0;
				if ( !nh.getParam( "/seabee3_teleop/strafe_scale", cmd_vel_conversions[Axes::strafe] ) ) cmd_vel_conversions[Axes::strafe] = 1.0;
				if ( !nh.getParam( "/seabee3_teleop/depth_scale", cmd_vel_conversions[Axes::depth] ) ) cmd_vel_conversions[Axes::depth] = 1.0;
				if ( !nh.getParam( "/seabee3_teleop/yaw_scale", cmd_vel_conversions[Axes::yaw] ) ) cmd_vel_conversions[Axes::yaw] = 1.0;

				// scale motor values to be within the following min+max value such that
				// |D'| = [ motor_val_min, motor_val_max ]
				nh_local_.param( "motor_val_deadzone", motor_val_deadzone_, 5.0 );
				nh_local_.param( "motor_val_min", motor_val_min_, 20.0 );
				nh_local_.param( "motor_val_max", motor_val_max_, 100.0 );

				nh_local_.param( "speed_err_cap", max_error_in_xyz_.x, 100.0 );
				nh_local_.param( "strafe_err_cap", max_error_in_xyz_.y, 100.0 );
				nh_local_.param( "depth_err_cap", max_error_in_xyz_.z, 100.0 );

				nh_local_.param( "roll_err_cap", max_error_in_rpy_.x, 25.0 );
				nh_local_.param( "pitch_err_cap", max_error_in_rpy_.y, 25.0 );
				nh_local_.param( "yaw_err_cap", max_error_in_rpy_.z, 25.0 );

				pid_controller_.applySettings( _SettingsArray( settings_array, &settings_array[6] ) );

				/*nh_local_.param( "pid/pos/X/p", pid_controller_.linear_x->settings.p, 2.5 );
				nh_local_.param( "pid/pos/X/i", pid_controller_.linear_x->settings.i, 0.05 );
				nh_local_.param( "pid/pos/X/d", pid_controller_.linear_x->settings.d, 0.2 );

				nh_local_.param( "pid/pos/Y/p", pid_controller_.linear_y->settings.p, 2.5 );
				nh_local_.param( "pid/pos/Y/i", pid_controller_.linear_y->settings.i, 0.05 );
				nh_local_.param( "pid/pos/Y/d", pid_controller_.linear_y->settings.d, 0.2 );

				nh_local_.param( "pid/pos/Z/p", pid_controller_.linear_z->settings.p, 2.5 );
				nh_local_.param( "pid/pos/Z/i", pid_controller_.linear_z->settings.i, 0.05 );
				nh_local_.param( "pid/pos/Z/d", pid_controller_.linear_z->settings.d, 0.2 );

				nh_local_.param( "pid/ori/R/p", pid_controller_.angular_x->settings.p, 2.5 );
				nh_local_.param( "pid/ori/R/i", pid_controller_.angular_x->settings.i, 0.05 );
				nh_local_.param( "pid/ori/R/d", pid_controller_.angular_x->settings.d, 0.2 );

				nh_local_.param( "pid/ori/P/p", pid_controller_.angular_y->settings.p, 2.5 );
				nh_local_.param( "pid/ori/P/i", pid_controller_.angular_y->settings.i, 0.05 );
				nh_local_.param( "pid/ori/P/d", pid_controller_.angular_y->settings.d, 0.2 );

				nh_local_.param( "pid/ori/Y/p", pid_controller_.angular_z->settings.p, 2.5 );
				nh_local_.param( "pid/ori/Y/i", pid_controller_.angular_z->settings.i, 0.05 );
				nh_local_.param( "pid/ori/Y/d", pid_controller_.angular_z->settings.d, 0.2 );*/

				//nh_local_.param( "pid/i_max", pid_i_max_, 1.0 );
				//nh_local_.param( "pid/i_min", pid_i_min_, -1.0 );

				nh_local_.param( "speed_m1_dir", motors_dir_cfg_[Axes::speed].at( 1 ), 1.0 );
				nh_local_.param( "speed_m2_dir", motors_dir_cfg_[Axes::speed].at( 2 ), 1.0 );

				nh_local_.param( "strafe_m1_dir", motors_dir_cfg_[Axes::strafe].at( 1 ), 1.0 );
				nh_local_.param( "strafe_m2_dir", motors_dir_cfg_[Axes::strafe].at( 2 ), 1.0 );

				nh_local_.param( "depth_m1_dir", motors_dir_cfg_[Axes::depth].at( 1 ), 1.0 );
				nh_local_.param( "depth_m2_dir", motors_dir_cfg_[Axes::depth].at( 2 ), 1.0 );

				nh_local_.param( "roll_m1_dir", motors_dir_cfg_[Axes::roll].at( 1 ), 1.0 );
				nh_local_.param( "roll_m2_dir", motors_dir_cfg_[Axes::roll].at( 2 ), 1.0 );

				nh_local_.param( "pitch_m1_dir", motors_dir_cfg_[Axes::pitch].at( 1 ), 1.0 );
				nh_local_.param( "pitch_m2_dir", motors_dir_cfg_[Axes::pitch].at( 2 ), 1.0 );

				nh_local_.param( "yaw_m1_dir", motors_dir_cfg_[Axes::yaw].at( 1 ), -1.0 );
				nh_local_.param( "yaw_m2_dir", motors_dir_cfg_[Axes::yaw].at( 2 ), 1.0 );

				nh_local_.param( "speed_axis_dir", axis_dir_cfg_[Axes::speed], 1.0 );
				nh_local_.param( "strafe_axis_dir", axis_dir_cfg_[Axes::strafe], 1.0 );
				nh_local_.param( "depth_axis_dir", axis_dir_cfg_[Axes::depth], -1.0 );
				nh_local_.param( "roll_axis_dir", axis_dir_cfg_[Axes::roll], -1.0 );
				nh_local_.param( "pitch_axis_dir", axis_dir_cfg_[Axes::pitch], -1.0 );
				nh_local_.param( "yaw_axis_dir", axis_dir_cfg_[Axes::yaw], -1.0 );

				//pid_controller_.linear.initPid( pid_i_min_, pid_i_max_ );
				pid_controller_.reset();

				//pid_controller_.angular.initPid( pid_i_min_, pid_i_max_ );
				//pid_controller_.reset();

				resetMotorCntlMsg();

				motor_cntl_pub_ = nh_local_.advertise<seabee3_driver_base::MotorCntl> ( "motor_cntl", 1 );
				physics_state_sub_ = nh_local_.subscribe( "/seabee3/physics_state", 1, &Seabee3Driver::physicsStateCB, this );

				reset_pose_srv_ = nh_local_.advertiseService( "/seabee3/reset_pose", &Seabee3Driver::resetPoseCB, this );
				set_desired_pose_srv_ = nh_local_.advertiseService( "/seabee3/set_desired_pose", &Seabee3Driver::setDesiredPoseCB, this );

				//set current xyz to 0, desired RPY to current RPY
				resetPose();
			}

			void physicsStateCB( const seabee3_common::PhysicsStateConstPtr & msg )
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
					motor1 = MotorControllerIDs::FWD_RIGHT_THRUSTER;
					motor2 = MotorControllerIDs::FWD_LEFT_THRUSTER;
					motor1_scale = motors_dir_cfg_[Axes::speed].at( 1 );
					motor2_scale = motors_dir_cfg_[Axes::speed].at( 2 );
					break;
					case Axes::strafe: //absolute; relative to the world
					updateMotorCntlMsg( msg, Axes::strafe_rel, value );
					//updateMotorCntlMsg(msg, strafe_rel, value * cos( LocalizationUtil::degToRad( IMUDataCache->ori.x ) ) );
					//updateMotorCntlMsg(msg, depth_rel, value * -sin( LocalizationUtil::degToRad( IMUDataCache->ori.x ) ) );
					return;
					case Axes::strafe_rel: //relative to the robot
					motor1 = MotorControllerIDs::STRAFE_FRONT_THRUSTER;
					motor2 = MotorControllerIDs::STRAFE_BACK_THRUSTER;
					motor1_scale = motors_dir_cfg_[Axes::strafe].at( 1 );
					motor2_scale = motors_dir_cfg_[Axes::strafe].at( 2 );
					break;
					case Axes::depth: //absolute; relative to the world
					updateMotorCntlMsg( msg, Axes::depth_rel, value * -1.0 );
					//updateMotorCntlMsg(msg, depth_rel, value * -cos( LocalizationUtil::degToRad( IMUDataCache->ori.x ) ) );
					//updateMotorCntlMsg(msg, strafe_rel, value * -sin( LocalizationUtil::degToRad( IMUDataCache->ori.x ) ) );
					return;
					case Axes::depth_rel: //relative to the robot
					motor1 = MotorControllerIDs::DEPTH_RIGHT_THRUSTER;
					motor2 = MotorControllerIDs::DEPTH_LEFT_THRUSTER;
					motor1_scale = motors_dir_cfg_[Axes::depth].at( 1 );
					motor2_scale = motors_dir_cfg_[Axes::depth].at( 2 );
					break;
					case Axes::roll: //absolute; relative to the world
					motor1 = MotorControllerIDs::DEPTH_RIGHT_THRUSTER;
					motor2 = MotorControllerIDs::DEPTH_LEFT_THRUSTER;
					motor1_scale = motors_dir_cfg_[Axes::roll].at( 1 );
					motor2_scale = motors_dir_cfg_[Axes::roll].at( 2 );
					break;
					case Axes::pitch: //pitch is only available as seabee starts to roll; make sure that's the case here
					//updateMotorCntlMsg( msg, yaw_rel, value * -sin( LocalizationUtil::degToRad( IMUDataCache->ori.x ) ) );
					return;
					case Axes::yaw: //absolute; relative to the world
					//updateMotorCntlMsg(msg, yaw_rel, value * cos( LocalizationUtil::degToRad( IMUDataCache->ori.x ) ) );
					updateMotorCntlMsg( msg, Axes::yaw_rel, value );
					return;
					case Axes::yaw_rel: //relative to the robot
					motor1 = MotorControllerIDs::STRAFE_FRONT_THRUSTER;
					motor2 = MotorControllerIDs::STRAFE_BACK_THRUSTER;
					motor1_scale = motors_dir_cfg_[Axes::yaw].at( 1 );
					motor2_scale = motors_dir_cfg_[Axes::yaw].at( 2 );
					break;
				}

				if ( motor1 < 0 || motor2 < 0 ) return;

				int motor1_val = msg.motors[motor1];
				int motor2_val = msg.motors[motor2];

				motor1_val += motor1_scale * value;
				motor2_val += motor2_scale * value;

				math_utils::capValueProp( motor1_val, motor2_val, 100 );

				msg.motors[motor1] = motor1_val;
				msg.motors[motor2] = motor2_val;

				msg.mask[motor1] = 1;
				msg.mask[motor2] = 1;
			}

			void scaleMotorValues( seabee3_driver_base::MotorCntl & msg )
			{
				for ( size_t i = 0; i < msg.motors.size(); ++i )
				{
					msg.motors[i] = scaleMotorValue( msg.motors[i] );
				}
			}

			int scaleMotorValue( int value )
			{
				//ROS_INFO( "value: %d", value );
				if ( value == 0 ) return value;

				if ( abs( value ) < motor_val_deadzone_ ) return 0;

				double value_d = fabs( value );
				double direction = ( value < 0 ? -1.0 : 1.0 );

				value_d = direction * ( motor_val_min_ + ( value_d - motor_val_deadzone_ ) * ( motor_val_max_ - motor_val_min_ ) / 100.0 );

				return (int) round( value_d );
			}

			void resetMotorCntlMsg()
			{
				for ( int i = 0; i < movement_common::NUM_MOTOR_CONTROLLERS; i++ )
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

			// set the current pose to the desired pose to zero out all errors
			void resetPose()
			{
				// copy current pose into desired pose
				current_pose_tf_ >> desired_pose_;
				current_pose_ = desired_pose_;
				desired_pose_ >> desired_pose_tf_;

				// publish the pose
				publishTfFrame( desired_pose_tf_, global_frame_, "/seabee3/desired_pose" );
			}

			bool setDesiredPoseCB( seabee3_common::SetDesiredPose::Request & req, seabee3_common::SetDesiredPose::Response & resp )
			{
				setDesiredPose( req.ori.mask, desired_pose_.angular, req.ori.values, req.ori.mode );

				geometry_msgs::Vector3 velocity = req.pos.values;

				req.pos.values.x = velocity.x * cos( desired_pose_.angular.z ) - velocity.y * sin( desired_pose_.angular.z );
				req.pos.values.y = velocity.x * sin( desired_pose_.angular.z ) + velocity.y * cos( desired_pose_.angular.z );

				setDesiredPose( req.pos.mask, desired_pose_.linear, req.pos.values, req.pos.mode );

				desired_pose_ >> desired_pose_tf_;
				// publish the pose
				publishTfFrame( desired_pose_tf_, global_frame_, "/seabee3/desired_pose" );

				return true;
			}

			void requestChangeInDesiredPose( const geometry_msgs::Twist & msg )
			{
				seabee3_common::SetDesiredPose::Request req;
				seabee3_common::SetDesiredPose::Response resp;
				seabee3_common::Vector3Masked change_in_desired_pose;

				change_in_desired_pose.mask.x = change_in_desired_pose.mask.y = change_in_desired_pose.mask.z = 1; //enable all values
				change_in_desired_pose.mode.x = change_in_desired_pose.mode.y = change_in_desired_pose.mode.z = 1; //set mode to incremental

				req.pos = req.ori = change_in_desired_pose;

				req.pos.values = msg.linear;
				req.ori.values = msg.angular;

				setDesiredPoseCB( req, resp );
			}

			void generateMotorCntlMsg( std::map<int, double> motor_values )
			{
				//math_utils::capValue( values[yaw], 100.0 );
				//math_utils::capValue( values[depth], 100.0 );

				updateMotorCntlMsg( motor_cntl_msg_, Axes::speed, motor_values[Axes::speed] );
				updateMotorCntlMsg( motor_cntl_msg_, Axes::strafe, motor_values[Axes::strafe] );
				updateMotorCntlMsg( motor_cntl_msg_, Axes::depth, motor_values[Axes::depth] );
				//updateMotorCntlMsg( motorCntlMsg, roll, rollMotorVal );
				//updateMotorCntlMsg( motorCntlMsg, pitch, pitchMotorVal );
				updateMotorCntlMsg( motor_cntl_msg_, Axes::yaw, motor_values[Axes::yaw] );
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

					math_utils::normalizeAngle( desired_pose_.angular.x );
					math_utils::normalizeAngle( desired_pose_.angular.y );
					math_utils::normalizeAngle( desired_pose_.angular.z );

					math_utils::normalizeAngle( current_pose_.angular.x );
					math_utils::normalizeAngle( current_pose_.angular.y );
					math_utils::normalizeAngle( current_pose_.angular.z );

					error_in_xyz_.x = desired_pose_.linear.x - current_pose_.linear.x;
					error_in_xyz_.y = desired_pose_.linear.y - current_pose_.linear.y;
					error_in_xyz_.z = desired_pose_.linear.z - current_pose_.linear.z;

					error_in_rpy_.x = math_utils::angleDistRel( math_utils::radToDeg( desired_pose_.angular.x ), math_utils::radToDeg( current_pose_.angular.x ) );
					error_in_rpy_.y = math_utils::angleDistRel( math_utils::radToDeg( desired_pose_.angular.y ), math_utils::radToDeg( current_pose_.angular.y ) );
					error_in_rpy_.z = math_utils::angleDistRel( math_utils::radToDeg( desired_pose_.angular.z ), math_utils::radToDeg( current_pose_.angular.z ) );

					//printf("error x %f y %f z %f r %f p %f y %f\n", error_in_xyz_.x, error_in_xyz_.y, error_in_xyz_.z, error_in_rpy_.x, error_in_rpy_.y, error_in_rpy_.z );

					math_utils::capValue( error_in_xyz_.x, max_error_in_xyz_.x );
					math_utils::capValue( error_in_xyz_.y, max_error_in_xyz_.y );
					math_utils::capValue( error_in_xyz_.z, max_error_in_xyz_.z );

					math_utils::capValue( error_in_rpy_.x, max_error_in_rpy_.x );
					math_utils::capValue( error_in_rpy_.y, max_error_in_rpy_.y );
					math_utils::capValue( error_in_rpy_.z, max_error_in_rpy_.z );

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

					std::map<int, double> motor_values;

					motor_values[Axes::speed] = axis_dir_cfg_[Axes::speed] * pid_controller_.linear_x->update( error_in_work.x );
					motor_values[Axes::strafe] = axis_dir_cfg_[Axes::strafe] * pid_controller_.linear_y->update( error_in_work.y );
					motor_values[Axes::depth] = axis_dir_cfg_[Axes::depth] * pid_controller_.linear_z->update( error_in_work.z );

					//double rollMotorVal = axis_dir_cfg_[roll] * pid_controller_.angular.x.updatePid( error_in_rpy_.x, dt );
					//double pitchMotorVal = axis_dir_cfg_[pitch] * pid_controller_.angular.y.updatePid( error_in_rpy_.y, dt );

					//printf("error_in_velocity.angular.z %f\n", error_in_velocity.angular.z);
					//printf("error_in_rpy.z %f\n", math_utils::degToRad( error_in_rpy_.z ) );

					//printf("w1 %f\n", 5.0 * 0.3 * math_utils::degToRad( error_in_rpy_.z ));
					//printf("w2 %f\n", 100.0 * physics_state_msg_.mass.angular.z * ( error_in_velocity.angular.z < 0 ? -1.0 : 1.0 ) * pow( error_in_velocity.angular.z, 2 ) / 2.0);

					// W=Fx=mv^2/2
					double error_in_work_yaw = 5.0 * 0.3 * math_utils::degToRad( error_in_rpy_.z ) - 100.0 * physics_state_msg_.mass.angular.z * ( error_in_velocity.angular.z < 0 ? 1.0 : -1.0 ) * pow(
							error_in_velocity.angular.z, 2 ) / 2.0;
					motor_values[Axes::yaw] = axis_dir_cfg_[Axes::yaw] * pid_controller_.angular_z->update( error_in_work_yaw );

					//printf( "speed %f strafe %f depth %f yaw %f\n", speed_motor_val, strafe_motor_val, depth_motor_val, yaw_motor_val );


					//math_utils::capValue( rollMotorVal, 50.0 );
					//math_utils::capValue( pitchMotorVal, 50.0 );

					generateMotorCntlMsg( motor_values );
				}
				last_pid_update_time_ = ros::Time::now();
			}

			// temporary change to enable heading-only PID
			void stepAbsoluteMeasurementPID()
			{
				desired_pose_velocity_ = twist_cache_;

				//fetchTfFrame( desired_pose_tf_, "landmark_map", "seabee3/desired_pose" );
				fetchTfFrame( current_pose_tf_, global_frame_, "/seabee3/base_link" );

				//convert tf frames to Twist messages; fuck quaternions
				//desired_pose_tf_ >> desired_pose_;
				current_pose_tf_ >> current_pose_;

				if ( last_pid_update_time_ != ros::Time( -1 ) )
				{
					ros::Duration dt = ros::Time::now() - last_pid_update_time_;

					//twistCache is essentially the linear and angular change in desired pose per second (linear + angular velocity)
					//we multiply by the change in time to obtain the desired change in pose
					//this change in pose is then added to the current pose; the result is desiredPose
					const double t1 = dt.toSec();
					geometry_msgs::Twist change_in_desired_pose = twist_cache_ * t1;
					requestChangeInDesiredPose( change_in_desired_pose );

					desired_pose_ >> desired_pose_tf_;
					publishTfFrame( desired_pose_tf_, global_frame_, "/seabee3/desired_pose" );

					math_utils::normalizeAngle( desired_pose_.angular.z );

					math_utils::normalizeAngle( current_pose_.angular.z );

					error_in_xyz_.z = desired_pose_.linear.z - current_pose_.linear.z;

					error_in_rpy_.z = math_utils::angleDistRel( math_utils::radToDeg( desired_pose_.angular.z ), math_utils::radToDeg( current_pose_.angular.z ) );

					printf("error x %f y %f z %f r %f p %f y %f\n", error_in_xyz_.x, error_in_xyz_.y, error_in_xyz_.z, error_in_rpy_.x, error_in_rpy_.y, error_in_rpy_.z );

					math_utils::capValue( error_in_rpy_.z, max_error_in_rpy_.z );

					printf( "error x %f y %f z %f r %f p %f y %f\n", error_in_xyz_.x, error_in_xyz_.y, error_in_xyz_.z, error_in_rpy_.x, error_in_rpy_.y, error_in_rpy_.z );

					double depth_motor_value = axis_dir_cfg_[Axes::depth] * pid_controller_.linear_z->update( error_in_xyz_.z );
					double yaw_motor_value = axis_dir_cfg_[Axes::yaw] * pid_controller_.angular_z->update( error_in_rpy_.z );

					printf( "depth %f yaw %f\n", depth_motor_value, yaw_motor_value );

					printf( "depth moto value: %f yaw motor value %f\n" );

					math_utils::capValue( depth_motor_value, 100.0 );
					math_utils::capValue( yaw_motor_value, 100.0 );

					updateMotorCntlMsg( motor_cntl_msg_, Axes::depth, depth_motor_value );
					updateMotorCntlMsg( motor_cntl_msg_, Axes::yaw, yaw_motor_value );

				}
				last_pid_update_time_ = ros::Time::now();
			}

			void setMotorValsFromCmdVel( bool use_pid_assist = false )
			{
				resetMotorCntlMsg();

				updateMotorCntlMsg( motor_cntl_msg_, Axes::speed, -100 * axis_dir_cfg_[Axes::speed] * twist_cache_.linear.x / cmd_vel_conversions[Axes::speed] );
				updateMotorCntlMsg( motor_cntl_msg_, Axes::strafe, -100 * axis_dir_cfg_[Axes::strafe] * twist_cache_.linear.y / cmd_vel_conversions[Axes::strafe] );

				if ( use_pid_assist )
				{
					stepAbsoluteMeasurementPID();
				}
				else
				{
					updateMotorCntlMsg( motor_cntl_msg_, Axes::depth, -100 * axis_dir_cfg_[Axes::depth] * twist_cache_.linear.z / cmd_vel_conversions[Axes::depth] );
					updateMotorCntlMsg( motor_cntl_msg_, Axes::yaw, 100 * axis_dir_cfg_[Axes::yaw] * twist_cache_.angular.z / cmd_vel_conversions[Axes::yaw] );
				}
			}

			void spinOnce()
			{
				//this also grabs and publishes tf frames

				if ( cmd_vel_conversion_mode_ == CmdVelConversionType::pid ) pidStep();
				else if ( cmd_vel_conversion_mode_ == CmdVelConversionType::linear ) setMotorValsFromCmdVel( use_pid_assist_ );

				/*for ( size_t i; i < motor_cntl_msg_.motors.size(); i++ )
				 {
				 motor_cntl_msg_.motors[i] = scaleMotorValue( motor_cntl_msg_.motors[i] );
				 }*/

				scaleMotorValues( motor_cntl_msg_ );
				motor_cntl_pub_.publish( motor_cntl_msg_ );
			}
		};

const Seabee3Driver::_Settings Seabee3Driver::settings_array[6] = {
																	_Settings( "linear/x" ),
																	_Settings( "linear/y" ),
																	_Settings( "linear/z" ),
																	_Settings( "angular/x" ),
																	_Settings( "angular/y" ),
																	_Settings( "angular/z" ) };

int main( int argc, char** argv )
{
	ros::init( argc, argv, "seabee3_driver" );
	ros::NodeHandle nh( "~" );
	
	Seabee3Driver seabee3_driver( nh );
	seabee3_driver.spin();

	return 0;
}
