/*******************************************************************************
 *
 *      seabee3_driver
 * 
 *      Copyright (c) 2010,
 *      Edward T. Kaszubski (ekaszubski@gmail.com),
 *      Michael Montalbo (mmontalbo@gmail.com)
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
#include <control_toolbox/pid.h> // for Pid
#include <localization_tools/Util.h>
#include <ros/ros.h>
#include <seabee3_beestem/BeeStem3.h> // for MotorControllerIDs
#include <string>
#include <tf/transform_broadcaster.h> // for TransformBroadcaster
#include <tf/transform_listener.h> // for TransformListener
//msgs
#include <geometry_msgs/Twist.h> // for cmd_vel
#include <geometry_msgs/Vector3.h> // for generic 3d vectors
#include <seabee3_driver_base/MotorCntl.h> // for outgoing thruster commands
// seabee3_driver/Vector3Masked.msg <-- seabee3_driver/SetDesiredPose.srv

//srvs
#include <seabee3_driver/SetDesiredPose.h> // for SetDesiredPose
void operator >>( const geometry_msgs::Twist & the_pose, tf::Transform & the_pose_tf )
{
	the_pose_tf.setOrigin( tf::Vector3( the_pose.linear.x, the_pose.linear.y, the_pose.linear.z ) );
	the_pose_tf.setRotation( tf::Quaternion( the_pose.angular.z, the_pose.angular.y, the_pose.angular.x ) );
}

void operator >>( const tf::Transform & the_pose_tf, geometry_msgs::Twist & the_pose )
{
	the_pose.linear.x = the_pose_tf.getOrigin().x();
	the_pose.linear.y = the_pose_tf.getOrigin().y();
	the_pose.linear.z = the_pose_tf.getOrigin().z();

	the_pose_tf.getBasis().getEulerZYX( the_pose.angular.z, the_pose.angular.y, the_pose.angular.x );
}

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

class Seabee3Driver
{
public:
	struct ThrusterArrayCfg // define the direction of each thruster in an array that is responsible for controlling a single axis of movement
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

	geometry_msgs::Twist twist_cache_;

	seabee3_driver_base::MotorCntl motor_cntl_msg_;

	geometry_msgs::Vector3 desired_rpy_, error_in_rpy_, desired_change_in_rpy_per_sec_, desired_xyz_, error_in_xyz_, desired_change_in_xyz_per_sec_;
	geometry_msgs::Vector3 max_error_in_rpy_, max_error_in_xyz_;

	ros::Time last_pid_update_time_;

	tf::Transform desired_pose_tf_;
	tf::Transform current_pose_tf_;

	geometry_msgs::Twist desired_pose_;
	geometry_msgs::Twist current_pose_;

	tf::TransformListener * tl_;
	tf::TransformBroadcaster * tb_;

	bool depth_initialized_;

	std::vector<ThrusterArrayCfg> thruster_dir_cfg_;

	double axis_dir_cfg_[6];
	double pid_i_min_, pid_i_max_;

	Pid3D xyz_pid_, rpy_pid_;

	ros::NodeHandle n_priv_;
	ros::Subscriber cmd_vel_sub_;
	ros::Publisher motor_cntl_pub_;
	ros::ServiceServer set_desired_pose_srv_;

public:
	Seabee3Driver( ros::NodeHandle & n ) :
		n_priv_( "~" )
	{
		tl_ = new tf::TransformListener;
		tb_ = new tf::TransformBroadcaster;

		last_pid_update_time_ = ros::Time( -1 );

		depth_initialized_ = false;

		n_priv_.param( "global_frame", global_frame_, std::string( "/landmark_map" ) );

		n_priv_.param( "speed_err_cap", max_error_in_xyz_.x, 100.0 );
		n_priv_.param( "strafe_err_cap", max_error_in_xyz_.y, 100.0 );
		n_priv_.param( "depth_err_cap", max_error_in_xyz_.z, 100.0 );

		n_priv_.param( "roll_err_cap", max_error_in_rpy_.x, 25.0 );
		n_priv_.param( "pitch_err_cap", max_error_in_rpy_.y, 25.0 );
		n_priv_.param( "yaw_err_cap", max_error_in_rpy_.z, 25.0 );

		n_priv_.param( "pid/pos/X/p", xyz_pid_.x.cfg.p, 2.5 );
		n_priv_.param( "pid/pos/X/i", xyz_pid_.x.cfg.i, 0.05 );
		n_priv_.param( "pid/pos/X/d", xyz_pid_.x.cfg.d, 0.2 );

		n_priv_.param( "pid/pos/Y/p", xyz_pid_.y.cfg.p, 2.5 );
		n_priv_.param( "pid/pos/Y/i", xyz_pid_.y.cfg.i, 0.05 );
		n_priv_.param( "pid/pos/Y/d", xyz_pid_.y.cfg.d, 0.2 );

		n_priv_.param( "pid/pos/Z/p", xyz_pid_.z.cfg.p, 2.5 );
		n_priv_.param( "pid/pos/Z/i", xyz_pid_.z.cfg.i, 0.05 );
		n_priv_.param( "pid/pos/Z/d", xyz_pid_.z.cfg.d, 0.2 );

		n_priv_.param( "pid/ori/R/p", rpy_pid_.x.cfg.p, 2.5 );
		n_priv_.param( "pid/ori/R/i", rpy_pid_.x.cfg.i, 0.05 );
		n_priv_.param( "pid/ori/R/d", rpy_pid_.x.cfg.d, 0.2 );

		n_priv_.param( "pid/ori/P/p", rpy_pid_.y.cfg.p, 2.5 );
		n_priv_.param( "pid/ori/P/i", rpy_pid_.y.cfg.i, 0.05 );
		n_priv_.param( "pid/ori/P/d", rpy_pid_.y.cfg.d, 0.2 );

		n_priv_.param( "pid/ori/Y/p", rpy_pid_.z.cfg.p, 2.5 );
		n_priv_.param( "pid/ori/Y/i", rpy_pid_.z.cfg.i, 0.05 );
		n_priv_.param( "pid/ori/Y/d", rpy_pid_.z.cfg.d, 0.2 );

		n_priv_.param( "pid/i_max", pid_i_max_, 1.0 );
		n_priv_.param( "pid/i_min", pid_i_min_, -1.0 );

		n_priv_.param( "speed_m1_dir", thruster_dir_cfg_[Axes::speed].at( 1 ), 1.0 );
		n_priv_.param( "speed_m2_dir", thruster_dir_cfg_[Axes::speed].at( 2 ), 1.0 );

		n_priv_.param( "strafe_m1_dir", thruster_dir_cfg_[Axes::strafe].at( 1 ), 1.0 );
		n_priv_.param( "strafe_m2_dir", thruster_dir_cfg_[Axes::strafe].at( 2 ), 1.0 );

		n_priv_.param( "depth_m1_dir", thruster_dir_cfg_[Axes::depth].at( 1 ), 1.0 );
		n_priv_.param( "depth_m2_dir", thruster_dir_cfg_[Axes::depth].at( 2 ), 1.0 );

		n_priv_.param( "roll_m1_dir", thruster_dir_cfg_[Axes::roll].at( 1 ), 1.0 );
		n_priv_.param( "roll_m2_dir", thruster_dir_cfg_[Axes::roll].at( 2 ), 1.0 );

		n_priv_.param( "pitch_m1_dir", thruster_dir_cfg_[Axes::pitch].at( 1 ), 1.0 );
		n_priv_.param( "pitch_m2_dir", thruster_dir_cfg_[Axes::pitch].at( 2 ), 1.0 );

		n_priv_.param( "yaw_m1_dir", thruster_dir_cfg_[Axes::yaw].at( 1 ), -1.0 );
		n_priv_.param( "yaw_m2_dir", thruster_dir_cfg_[Axes::yaw].at( 2 ), 1.0 );

		n_priv_.param( "speed_axis_dir", axis_dir_cfg_[Axes::speed], 1.0 );
		n_priv_.param( "strafe_axis_dir", axis_dir_cfg_[Axes::strafe], 1.0 );
		n_priv_.param( "depth_axis_dir", axis_dir_cfg_[Axes::depth], -1.0 );
		n_priv_.param( "roll_axis_dir", axis_dir_cfg_[Axes::roll], -1.0 );
		n_priv_.param( "pitch_axis_dir", axis_dir_cfg_[Axes::pitch], -1.0 );
		n_priv_.param( "yaw_axis_dir", axis_dir_cfg_[Axes::yaw], -1.0 );

		xyz_pid_.initPid( pid_i_min_, pid_i_max_ );
		xyz_pid_.reset();

		rpy_pid_.initPid( pid_i_min_, pid_i_max_ );
		rpy_pid_.reset();

		for ( int i = 0; i < BeeStem3::NUM_MOTOR_CONTROLLERS; i++ )
		{
			motor_cntl_msg_.motors[i] = 0;
			motor_cntl_msg_.mask[i] = 0;
		}

		cmd_vel_sub_ = n.subscribe( "/seabee3/cmd_vel", 1, &Seabee3Driver::cmdVelCB, this );
		motor_cntl_pub_ = n.advertise<seabee3_driver_base::MotorCntl> ( "/seabee3/motor_cntl", 1 );
		set_desired_pose_srv_ = n.advertiseService( "/seabee3/set_desired_pose", &Seabee3Driver::setDesiredPoseCB, this );

		setDesiredPose( geometry_msgs::Twist() ); //set current xyz to 0, desired RPY to current RPY
	}

	~Seabee3Driver()
	{
		delete tl_;
		delete tb_;
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

		if( motor1 < 0 || motor2 < 0 ) return;

		int motor1_val = msg.motors[motor1];
		int motor2_val = msg.motors[motor2];

		motor1_val += motor1_scale * value;
		motor2_val += motor2_scale * value;

		LocalizationUtil::capValueProp( motor1_val, motor2_val, 100 );

		msg.motors[motor1] = motor1_val;
		msg.motors[motor2] = motor2_val;

		msg.mask[motor1] = 1;
		msg.mask[motor2] = 1;
	}


	/*void updateMotorCntlFromTwist( const geometry_msgs::TwistConstPtr & twist )
	 {
	 //ROS_INFO("udpateMotorCntlFromTwist()");
	 tf::Vector3 vel_desired_lin( twist->linear.x, twist->linear.y, twist->linear.z );
	 tf::Vector3 vel_desired_ang( twist->angular.x, twist->angular.y, twist->angular.z );
	 //tf::Vector3 vel_diff_lin (vel_desired_lin - *vel_est_lin);
	 //tf::Vector3 vel_diff_ang (vel_desired_ang - *vel_est_ang);

	 //	ROS_INFO("-----------------------------------------------------");
	 //
	 //	ROS_INFO("linear x %f y %f z %f", vel_desired_lin.getX(), vel_desired_lin.getY(), vel_desired_lin.getZ());
	 //	ROS_INFO("angular x %f y %f z %f", vel_desired_ang.getX(), vel_desired_ang.getY(), vel_desired_ang.getZ());

	 desiredSpeed = vel_desired_lin.getX() * 100.0;
	 desiredStrafe = vel_desired_lin.getY() * 100.0;
	 // *desiredDepth = vel_diff_lin.getZ() * 100.0;

	 //updateMotorCntlMsg(*motorCntlMsg, Axes::speed, vel_diff_lin.getX() * 100.0);
	 //updateMotorCntlMsg(*motorCntlMsg, Axes::strafe, vel_diff_lin.getY() * 50.0);
	 //updateMotorCntlMsg(*motorCntlMsg, Axes::depth, vel_diff_lin.getZ() * 100.0);
	 //updateMotorCntlMsg(*motorCntlMsg, Axes::roll, vel_diff_ang.getX() * 100.0);
	 //updateMotorCntlMsg(*motorCntlMsg, Axes::yaw, vel_diff_ang.getZ() * 100.0);

	 if ( lastUpdateTime != ros::Time( -1 ) )
	 {
	 //ros::Duration dt = ros::Time::now() - *lastYawUpdateTime;

	 //desiredChangeInXYZPerSec->x = 100.0 * twist->linear.x;
	 //desiredChangeInXYZPerSec->y = 100.0 * twist->linear.y;
	 desiredChangeInXYZPerSec->z = 100.0 * twist->linear.z;


	 //desiredChangeInRPYPerSec->x = 100.0 * twist->angular.x;
	 //desiredChangeInRPYPerSec->y = 100.0 * twist->angular.y;
	 desiredChangeInRPYPerSec->z = 100.0 * twist->angular.z;


	 //ROS_INFO("yaw: %f desired yaw: %f dt: %f", IMUDataCache->ori.z, *desiredRPY.z, dt.toSec());

	 //		if(!IMUDataCache->empty())
	 //			ROS_INFO("desired yaw: %f actualYaw: %f dt: %f", *desiredRPY.z, IMUDataCache->front().ori.z, dt.toSec());
	 }

	 lastUpdateTime = ros::Time::now();
	 }*/

	bool setDesiredPoseCB( seabee3_driver::SetDesiredPose::Request & req, seabee3_driver::SetDesiredPose::Response & resp )
	{
		//desiredPoseTf >> desiredPose;

		if ( req.pos.mask.x != 0.0 ) desired_pose_.linear.x = req.pos.values.x + ( req.pos.mode.x == 1.0 ? desired_pose_.linear.x : 0.0 );
		if ( req.pos.mask.y != 0.0 ) desired_pose_.linear.y = req.pos.values.y + ( req.pos.mode.y == 1.0 ? desired_pose_.linear.y : 0.0 );
		if ( req.pos.mask.z != 0.0 ) desired_pose_.linear.z = req.pos.values.z + ( req.pos.mode.z == 1.0 ? desired_pose_.linear.z : 0.0 );

		if ( req.ori.mask.x != 0.0 ) desired_pose_.angular.x = req.ori.values.x + ( req.ori.mode.x == 1.0 ? desired_pose_.angular.x : 0.0 );
		if ( req.ori.mask.y != 0.0 ) desired_pose_.angular.y = req.ori.values.y + ( req.ori.mode.y == 1.0 ? desired_pose_.angular.y : 0.0 );
		if ( req.ori.mask.z != 0.0 ) desired_pose_.angular.z = req.ori.values.z + ( req.ori.mode.z == 1.0 ? desired_pose_.angular.z : 0.0 );


		//desiredPose >> desiredPoseTf;
		return true;
	}

	void setDesiredPose( const geometry_msgs::Twist & msg )
	{
		seabee3_driver::SetDesiredPose::Request req;
		seabee3_driver::SetDesiredPose::Response resp;
		seabee3_driver::Vector3Masked change_in_desired_pose;

		change_in_desired_pose.mask.x = change_in_desired_pose.mask.y = change_in_desired_pose.mask.z = 1; //enable all values
		change_in_desired_pose.mode.x = change_in_desired_pose.mode.y = change_in_desired_pose.mode.z = 1; //set mode to incremental

		req.pos = req.ori = change_in_desired_pose;

		req.pos.values = msg.linear;
		req.ori.values = msg.angular;

		setDesiredPoseCB( req, resp );
	}

	void fetchTfFrame( tf::Transform transform, const std::string & frame1, const std::string & frame2 )
	{
		tf::StampedTransform temp;
		try
		{
			tl_->lookupTransform( frame1, frame2, ros::Time( 0 ), temp );

			transform.setOrigin( temp.getOrigin() );
			transform.setRotation( temp.getRotation() );
		}
		catch ( tf::TransformException ex )
		{
			ROS_ERROR( "%s", ex.what() );
		}
	}

	void pidStep()
	{
		fetchTfFrame( desired_pose_tf_, "seabee3/landmark_map", "seabee3/desired_pose" );
		fetchTfFrame( current_pose_tf_, "seabee3/landmark_map", "seabee3/base_link" );

		desired_pose_tf_ >> desired_pose_;
		current_pose_tf_ >> current_pose_;

		if ( last_pid_update_time_ != ros::Time( -1 ) )
		{
			for ( int i = 0; i < BeeStem3::NUM_MOTOR_CONTROLLERS; i++ )
			{
				motor_cntl_msg_.mask[i] = 0;
				motor_cntl_msg_.motors[i] = 0;
			}

			ros::Duration dt = ros::Time::now() - last_pid_update_time_;


			//twistCache is essentially the linear and angular change in desired pose
			//we multiply by the change in time and an arbitrary constant to obtain the desired change in pose per second
			//this change in pose is then added to the current pose; the result is desiredPose
			const double t1 = dt.toSec() * -100.0;
			geometry_msgs::Twist changeInDesiredPose = twist_cache_ * t1;
			setDesiredPose( changeInDesiredPose );

			LocalizationUtil::normalizeAngle( desired_pose_.angular.x );
			LocalizationUtil::normalizeAngle( desired_pose_.angular.y );
			LocalizationUtil::normalizeAngle( desired_pose_.angular.z );

			error_in_xyz_.x = desired_pose_.linear.x - current_pose_.linear.x;
			error_in_xyz_.y = desired_pose_.linear.y - current_pose_.linear.y;
			error_in_xyz_.z = desired_pose_.linear.z - current_pose_.linear.z;

			error_in_rpy_.x = LocalizationUtil::angleDistRel( desired_pose_.angular.x, current_pose_.angular.x );
			error_in_rpy_.y = LocalizationUtil::angleDistRel( desired_pose_.angular.y, current_pose_.angular.y );
			error_in_rpy_.z = LocalizationUtil::angleDistRel( desired_pose_.angular.z, current_pose_.angular.z );

			LocalizationUtil::capValue( error_in_xyz_.x, max_error_in_xyz_.x );
			LocalizationUtil::capValue( error_in_xyz_.y, max_error_in_xyz_.y );
			LocalizationUtil::capValue( error_in_xyz_.z, max_error_in_xyz_.z );

			LocalizationUtil::capValue( error_in_rpy_.x, max_error_in_rpy_.x );
			LocalizationUtil::capValue( error_in_rpy_.y, max_error_in_rpy_.y );
			LocalizationUtil::capValue( error_in_rpy_.z, max_error_in_rpy_.z );

			double speedMotorVal = axis_dir_cfg_[Axes::speed] * xyz_pid_.x.pid.updatePid( error_in_xyz_.x, dt );
			double strafeMotorVal = axis_dir_cfg_[Axes::strafe] * xyz_pid_.y.pid.updatePid( error_in_xyz_.y, dt );
			double depthMotorVal = axis_dir_cfg_[Axes::depth] * xyz_pid_.z.pid.updatePid( error_in_xyz_.z, dt );

			//double rollMotorVal = axis_dir_cfg_[Axes::roll] * rpy_pid_.x.pid.updatePid( error_in_rpy_.x, dt );
			//double pitchMotorVal = axis_dir_cfg_[Axes::pitch] * rpy_pid_.y.pid.updatePid( error_in_rpy_.y, dt );
			double yawMotorVal = axis_dir_cfg_[Axes::yaw] * rpy_pid_.z.pid.updatePid( error_in_rpy_.z, dt );


			/*LocalizationUtil::capValue(rollMotorVal, 50.0);
			 LocalizationUtil::capValue(pitchMotorVal, 50.0);
			 LocalizationUtil::capValue(yawMotorVal, 50.0);
			 LocalizationUtil::capValue(depthMotorVal, 50.0);*/

			updateMotorCntlMsg( motor_cntl_msg_, Axes::speed, speedMotorVal );
			updateMotorCntlMsg( motor_cntl_msg_, Axes::strafe, strafeMotorVal );
			updateMotorCntlMsg( motor_cntl_msg_, Axes::depth, depthMotorVal );
			//updateMotorCntlMsg( motorCntlMsg, Axes::roll, rollMotorVal );
			//updateMotorCntlMsg( motorCntlMsg, Axes::pitch, pitchMotorVal );
			updateMotorCntlMsg( motor_cntl_msg_, Axes::yaw, yawMotorVal );

			desired_pose_ >> desired_pose_tf_;
			tb_->sendTransform( tf::StampedTransform( desired_pose_tf_, ros::Time::now(), global_frame_, "seabee3/desired_pose" ) );
		}
		last_pid_update_time_ = ros::Time::now();
	}

	void cmdVelCB( const geometry_msgs::TwistConstPtr & twist )
	{
		twist_cache_ = *twist;
	}

	void spin()
	{
		while ( ros::ok() )
		{
			//this also grabs and publishes tf frames
			pidStep();

			motor_cntl_pub_.publish( motor_cntl_msg_ );

			ros::spinOnce();
			ros::Rate( 20 ).sleep();
		}
	}
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "seabee3_driver" );
	ros::NodeHandle n;
	
	Seabee3Driver seabee3_driver( n );
	seabee3_driver.spin();

	return 0;
}
