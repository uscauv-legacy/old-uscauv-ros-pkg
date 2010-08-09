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
#include <ros/ros.h>
#include <seabee3_beestem/BeeStem3.h> // for MotorControllerIDs
#include <tf/transform_broadcaster.h> // for TransformBroadcaster
#include <tf/transform_listener.h> // for TransformListener

//msgs
#include <geometry_msgs/Twist.h> // for cmd_vel
#include <geometry_msgs/Vector3.h> // for generic 3d vectors
#include <seabee3_driver_base/MotorCntl.h> // for outgoing thruster commands
// seabee3_driver/Vector3Masked.msg <-- seabee3_driver/SetDesiredPose.srv

//srvs
#include <seabee3_driver/SetDesiredPose.h> // for SetDesiredPose

geometry_msgs::Twist * TwistCache;

seabee3_driver_base::MotorCntl * motorCntlMsg;

geometry_msgs::Vector3 * desiredRPY, *errorInRPY, *desiredChangeInRPYPerSec, *desiredXYZ, *errorInXYZ, *desiredChangeInXYZPerSec;
double maxRollError, maxPitchError, maxYawError, maxDepthError, desiredSpeed, desiredStrafe;

ros::Time * lastUpdateTime;
ros::Time * lastPidUpdateTime;
ros::Time lastCmdVelUpdateTime;
tf::Vector3 mImuOriOffset;

tf::Transform desiredPose;
tf::Transform currentPose;

double speed_m1_dir, speed_m2_dir, strafe_m1_dir, strafe_m2_dir, depth_m1_dir, depth_m2_dir, roll_m1_dir, roll_m2_dir, yaw_m1_dir, yaw_m2_dir;

double speed_axis_dir, strafe_axis_dir, depth_axis_dir, roll_axis_dir, pitch_axis_dir, yaw_axis_dir, cmdVelTimeout;

bool depthInitialized;

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

struct PidConfig3D
{
	PidConfig x, y, z;
};

struct Pid3D
{
	control_toolbox::Pid x, y, z;
	PidConfig3D cfg;
};

Pid3D xyzPid, rpyPid;

void updateMotorCntlMsg( seabee3_driver_base::MotorCntl & msg, int axis, int p_value )
{
	int value = p_value;
	
	int motor1, motor2;
	double motor1_scale = 1.0, motor2_scale = 1.0;
	
	switch ( axis )
	{
	case Axes::speed: //relative to the robot
		motor1 = BeeStem3::MotorControllerIDs::FWD_RIGHT_THRUSTER;
		motor2 = BeeStem3::MotorControllerIDs::FWD_LEFT_THRUSTER;
		motor1_scale = speed_m1_dir;
		motor2_scale = speed_m2_dir;
		break;
	case Axes::strafe: //absolute; relative to the world
		updateMotorCntlMsg( msg, Axes::strafe_rel, value );
		//updateMotorCntlMsg(msg, Axes::strafe_rel, value * cos( LocalizationUtil::degToRad( IMUDataCache->ori.x ) ) );
		//updateMotorCntlMsg(msg, Axes::depth_rel, value * -sin( LocalizationUtil::degToRad( IMUDataCache->ori.x ) ) );
		return;
	case Axes::strafe_rel: //relative to the robot
		motor1 = BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER;
		motor2 = BeeStem3::MotorControllerIDs::STRAFE_BACK_THRUSTER;
		motor1_scale = strafe_m1_dir;
		motor2_scale = strafe_m2_dir;
		break;
	case Axes::depth: //absolute; relative to the world
		updateMotorCntlMsg( msg, Axes::depth_rel, value * -1.0 );
		//updateMotorCntlMsg(msg, Axes::depth_rel, value * -cos( LocalizationUtil::degToRad( IMUDataCache->ori.x ) ) );
		//updateMotorCntlMsg(msg, Axes::strafe_rel, value * -sin( LocalizationUtil::degToRad( IMUDataCache->ori.x ) ) );
		return;
	case Axes::depth_rel: //relative to the robot
		motor1 = BeeStem3::MotorControllerIDs::DEPTH_RIGHT_THRUSTER;
		motor2 = BeeStem3::MotorControllerIDs::DEPTH_LEFT_THRUSTER;
		motor1_scale = depth_m1_dir;
		motor2_scale = depth_m2_dir;
		break;
	case Axes::roll: //absolute; relative to the world
		motor1 = BeeStem3::MotorControllerIDs::DEPTH_RIGHT_THRUSTER;
		motor2 = BeeStem3::MotorControllerIDs::DEPTH_LEFT_THRUSTER;
		motor1_scale = roll_m1_dir;
		motor2_scale = roll_m2_dir;
		break;
	case Axes::pitch: //pitch is only available as seabee starts to roll; make sure that's the case here
		updateMotorCntlMsg( msg, Axes::yaw_rel, value * -sin( LocalizationUtil::degToRad( IMUDataCache->ori.x ) ) );
		return;
	case Axes::yaw: //absolute; relative to the world
		//updateMotorCntlMsg(msg, Axes::yaw_rel, value * cos( LocalizationUtil::degToRad( IMUDataCache->ori.x ) ) );
		updateMotorCntlMsg( msg, Axes::yaw_rel, value );
		return;
	case Axes::yaw_rel: //relative to the robot
		motor1 = BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER;
		motor2 = BeeStem3::MotorControllerIDs::STRAFE_BACK_THRUSTER;
		motor1_scale = yaw_m1_dir;
		motor2_scale = yaw_m2_dir;
		break;
	}

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

void updateMotorCntlFromTwist( const geometry_msgs::TwistConstPtr & twist )
{
	//ROS_INFO("udpateMotorCntlFromTwist()");
	tf::Vector3 vel_desired_lin( twist->linear.x, twist->linear.y, twist->linear.z );
	tf::Vector3 vel_desired_ang( twist->angular.x, twist->angular.y, twist->angular.z );
	//tf::Vector3 vel_diff_lin (vel_desired_lin - *vel_est_lin);
	//tf::Vector3 vel_diff_ang (vel_desired_ang - *vel_est_ang);

	/*ROS_INFO("-----------------------------------------------------");

	 ROS_INFO("linear x %f y %f z %f", vel_desired_lin.getX(), vel_desired_lin.getY(), vel_desired_lin.getZ());
	 ROS_INFO("angular x %f y %f z %f", vel_desired_ang.getX(), vel_desired_ang.getY(), vel_desired_ang.getZ());*/

	desiredSpeed = vel_desired_lin.getX() * 100.0;
	desiredStrafe = vel_desired_lin.getY() * 100.0;
	//*desiredDepth = vel_diff_lin.getZ() * 100.0;

	//updateMotorCntlMsg(*motorCntlMsg, Axes::speed, vel_diff_lin.getX() * 100.0);
	//updateMotorCntlMsg(*motorCntlMsg, Axes::strafe, vel_diff_lin.getY() * 50.0);
	//updateMotorCntlMsg(*motorCntlMsg, Axes::depth, vel_diff_lin.getZ() * 100.0);
	//updateMotorCntlMsg(*motorCntlMsg, Axes::roll, vel_diff_ang.getX() * 100.0);
	//updateMotorCntlMsg(*motorCntlMsg, Axes::yaw, vel_diff_ang.getZ() * 100.0);

	if ( *lastUpdateTime != ros::Time( -1 ) )
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
	
	*lastUpdateTime = ros::Time::now();
}

void pidStep()
{
	if ( *lastPidUpdateTime != ros::Time( -1 ) )
	{
		for ( int i = 0; i < BeeStem3::NUM_MOTOR_CONTROLLERS; i++ )
		{
			motorCntlMsg->mask[i] = 0;
			motorCntlMsg->motors[i] = 0;
		}

		ros::Duration dt = ros::Time::now() - *lastPidUpdateTime;
		
		
		//desiredRPY->x -= desiredChangeInRPYPerSec->x * dt.toSec();
		//desiredRPY->y -= desiredChangeInRPYPerSec->y * dt.toSec();
		desiredRPY->z -= desiredChangeInRPYPerSec->z * dt.toSec();

		
		//desiredXYZ->x -= desiredChangeInXYZPerSec->x * dt.toSec();
		//desiredXYZ->y -= desiredChangeInXYZPerSec->y * dt.toSec();
		desiredXYZ->z -= desiredChangeInXYZPerSec->z * dt.toSec();


		//LocalizationUtil::normalizeAngle(desiredRPY->x);
		//LocalizationUtil::normalizeAngle(desiredRPY->y);
		LocalizationUtil::normalizeAngle( desiredRPY->z );


		//actual - desired; 0 - 40 = -40;
		//desired -> actual; 40 -> 0 = 40 cw = -40
		//double yawError = LocalizationUtil::angleDistRel(desiredRPY->z, IMUDataCache->ori.z);

		//errorInXYZ->x = desiredXYZ->y - <some sensor value>;
		//errorInXYZ->y = desiredXYZ->x - <some sensor value>;
		errorInXYZ->z = desiredXYZ->z - depthCache->value;
		

		//errorInRPY->x = LocalizationUtil::angleDistRel(desiredRPY->x, IMUDataCache->ori.x);
		//errorInRPY->y = LocalizationUtil::angleDistRel(desiredRPY->y, IMUDataCache->ori.y);
		errorInRPY->z = LocalizationUtil::angleDistRel( desiredRPY->z, IMUDataCache->ori.z );

		
		//LocalizationUtil::capValue(errorInDepth->x, maxXError);
		//LocalizationUtil::capValue(errorInDepth->y, maxYError);
		LocalizationUtil::capValue( errorInXYZ->z, maxDepthError );

		
		//LocalizationUtil::capValue(errorInRPY->x, maxRollError);
		//LocalizationUtil::capValue(errorInRPY->y, maxPitchError);
		LocalizationUtil::capValue( errorInRPY->z, maxYawError );

		
		//yawError = abs(yawError) > *maxYawError ? *maxYawError * yawError / abs(yawError) : yawError;

		double speedMotorVal = speed_axis_dir * desiredSpeed;
		double strafeMotorVal = strafe_axis_dir * desiredStrafe;
		double depthMotorVal = depth_axis_dir * pid_D->updatePid( errorInXYZ->z, dt );
		//double rollMotorVal = roll_axis_dir * pid_R->updatePid(errorInRPY->x, dt);
		//double pitchMotorVal = pitch_axis_dir * pid_P->updatePid(errorInRPY->y, dt);
		double yawMotorVal = yaw_axis_dir * pid_Y->updatePid( errorInRPY->z, dt );


		//ROS_INFO("initial motor val: %f", motorVal);

		/*LocalizationUtil::capValue(rollMotorVal, 50.0);
		 LocalizationUtil::capValue(pitchMotorVal, 50.0);
		 LocalizationUtil::capValue(yawMotorVal, 50.0);
		 LocalizationUtil::capValue(depthMotorVal, 50.0);*/

		//motorVal = abs(motorVal) > 100 ? 100 * motorVal / abs(motorVal) : motorVal;

		//ROS_INFO("yaw: %f desired yaw: %f yaw error: %f motorValue: %f", IMUDataCache->ori.z, desiredRPY->z, errorInRPY->z, yawMotorVal);

		//ROS_INFO("Depth; desired: %d current %d error %d motorVal %f", *desiredDepth, depthCache->Value, *errorInDepth, depthMotorVal);

		updateMotorCntlMsg( *motorCntlMsg, Axes::speed, speedMotorVal );
		updateMotorCntlMsg( *motorCntlMsg, Axes::strafe, strafeMotorVal );
		updateMotorCntlMsg( *motorCntlMsg, Axes::depth, depthMotorVal );
		//updateMotorCntlMsg(*motorCntlMsg, Axes::roll, rollMotorVal);
		//updateMotorCntlMsg(*motorCntlMsg, Axes::pitch, pitchMotorVal);
		updateMotorCntlMsg( *motorCntlMsg, Axes::yaw, yawMotorVal );


		//ROS_INFO("yaw error: %f motorVal: %f", yawError, motorVal);
	}
	*lastPidUpdateTime = ros::Time::now();
}

void IMUDataCallback( const xsens_node::IMUDataConstPtr & data )
{
	//	ROS_INFO("IMUDataCallback()");
	*IMUDataCache = *data;
	
	IMUDataCache->ori.x += mImuOriOffset.x();
	IMUDataCache->ori.y += mImuOriOffset.y();
	IMUDataCache->ori.z += mImuOriOffset.z();
	
	while ( IMUDataCache->ori.z > 360 )
		IMUDataCache->ori.z -= 360;
	while ( IMUDataCache->ori.z < 0 )
		IMUDataCache->ori.z += 360;
	

	//estimatedPose.setRotation( tf::Quaternion( IMUDataCache->ori.z, IMUDataCache->ori.y, IMUDataCache->ori.x ) );
	//ROS_INFO("x %f y %f z %f", IMUDataCache->ori.x, IMUDataCache->ori.y, IMUDataCache->ori.z);
	//while(IMUDataCache->size() > 5)
	//{
	//	IMUDataCache->pop();
	//}
}

void CmdVelCallback( const geometry_msgs::TwistConstPtr & twist )
{
	*TwistCache = *twist;
	
	SetDesiredPose(*twist);

	//updateMotorCntlFromTwist( twist );
}

void DepthCallback( const seabee3_driver_base::DepthConstPtr & depth )
{
	*depthCache = *depth;
	if ( !depthInitialized )
	{
		desiredXYZ->z = depthCache->value;
		depthInitialized = true;
	}
}

void SetDesiredPoseCallback(seabee3_driver::SetDesiredPose::Request & req, seabee3_driver::SetDesiredPose::Reponse & resp)
{
	if(req.pos.mask == 1)
	{

	}
}

void SetDesiredPose(const geometry_msgs::Twist & msg)
{
	seabee3_driver::Vector3Masked changeInDesiredPose;

	seabee3_driver::SetDesiredPose setDesiredPose;
	setDesiredPoseRequest = changeInDesiredPose;
	setDesiredPose.call
}

void KillSwitchCallback( const seabee3_driver_base::KillSwitchConstPtr & killSwitch )
{
	if ( killSwitch->isKilled == 0 && killSwitchCache->isKilled == 1 )
	{
		//set desired pose to our current pose so we don't move yet
		SetDesiredPose();
	}
	*killSwitchCache = *killSwitch;
}

int main( int argc, char** argv )
{
	ros::init( argc, argv, "seabee3_driver" );
	ros::NodeHandle n( "~" );

	
	//vel_est_lin = new tf::Vector3(0, 0, 0);
	//vel_est_ang = new tf::Vector3(0, 0, 0);
	//velocity_sample_start = new ros::Time(-1);

	IMUDataCache = new xsens_node::IMUData;
	TwistCache = new geometry_msgs::Twist;
	
	ros::Subscriber imu_sub = n.subscribe( "/xsens/data_calibrated", 1, IMUDataCallback ); //likely necessary to remap this to something real ie. /xsens/data_calibrated
	//ros::Subscriber odom_prim_sub = n.subscribe("/seabee3/odom_prim", 1, OdomPrimCallback);
	ros::Subscriber cmd_vel_sub = n.subscribe( "/seabee3/cmd_vel", 1, CmdVelCallback );
	ros::Subscriber extl_depth_sub = n.subscribe( "/seabee3/depth", 1, DepthCallback );
	ros::Subscriber kill_switch_sub = n.subscribe( "/seabee3/kill_switch", 1, KillSwitchCallback );
	
	desiredRPY = new geometry_msgs::Vector3;
	desiredRPY->x = 0;
	desiredRPY->y = 0;
	desiredRPY->z = 0;
	errorInRPY = new geometry_msgs::Vector3;
	errorInRPY->x = 0;
	errorInRPY->y = 0;
	errorInRPY->z = 0;
	desiredChangeInRPYPerSec = new geometry_msgs::Vector3;
	desiredChangeInRPYPerSec->x = 0;
	desiredChangeInRPYPerSec->y = 0;
	desiredChangeInRPYPerSec->z = 0;
	
	desiredXYZ = new geometry_msgs::Vector3;
	desiredXYZ->x = 0;
	desiredXYZ->y = 0;
	desiredXYZ->z = 0;
	errorInXYZ = new geometry_msgs::Vector3;
	errorInXYZ->x = 0;
	errorInXYZ->y = 0;
	errorInXYZ->z = 0;
	desiredChangeInXYZPerSec = new geometry_msgs::Vector3;
	desiredChangeInXYZPerSec->x = 0;
	desiredChangeInXYZPerSec->y = 0;
	desiredChangeInXYZPerSec->z = 0;
	
	lastUpdateTime = new ros::Time( ros::Time( -1 ) );
	lastPidUpdateTime = new ros::Time( ros::Time( -1 ) );
	depthCache = new seabee3_driver_base::Depth;
	killSwitchCache = new seabee3_driver_base::KillSwitch;
	
	depthInitialized = false;
	
	pid_D = new control_toolbox::Pid;
	pid_R = new control_toolbox::Pid;
	pid_P = new control_toolbox::Pid;
	pid_Y = new control_toolbox::Pid;
	
	PIDConfig pid_D_cfg, pid_R_cfg, pid_P_cfg, pid_Y_cfg;
	
	double pid_i_min, pid_i_max;
	
	n.param( "cmd_vel_timeout", cmdVelTimeout, 2.0 );
	
	n.param( "depth_err_cap", maxDepthError, 100.0 );
	n.param( "roll_err_cap", maxRollError, 25.0 );
	n.param( "pitch_err_cap", maxPitchError, 25.0 );
	n.param( "yaw_err_cap", maxYawError, 25.0 );
	
	n.param( "pid/D/p", pid_D_cfg.p, 2.5 );
	n.param( "pid/D/i", pid_D_cfg.i, 0.05 );
	n.param( "pid/D/d", pid_D_cfg.d, 0.2 );
	
	n.param( "pid/R/p", pid_R_cfg.p, 2.5 );
	n.param( "pid/R/i", pid_R_cfg.i, 0.05 );
	n.param( "pid/R/d", pid_R_cfg.d, 0.2 );
	
	n.param( "pid/P/p", pid_P_cfg.p, 2.5 );
	n.param( "pid/P/i", pid_P_cfg.i, 0.05 );
	n.param( "pid/P/d", pid_P_cfg.d, 0.2 );
	
	n.param( "pid/Y/p", pid_Y_cfg.p, 2.5 );
	n.param( "pid/Y/i", pid_Y_cfg.i, 0.05 );
	n.param( "pid/Y/d", pid_Y_cfg.d, 0.2 );
	
	n.param( "pid/i_max", pid_i_max, 1.0 );
	n.param( "pid/i_min", pid_i_min, -1.0 );
	
	n.param( "speed_m1_dir", speed_m1_dir, 1.0 );
	n.param( "speed_m2_dir", speed_m2_dir, 1.0 );
	
	n.param( "strafe_m1_dir", strafe_m1_dir, 1.0 );
	n.param( "strafe_m2_dir", strafe_m2_dir, 1.0 );
	
	n.param( "depth_m1_dir", depth_m1_dir, 1.0 );
	n.param( "depth_m2_dir", depth_m2_dir, 1.0 );
	
	n.param( "roll_m1_dir", roll_m1_dir, 1.0 );
	n.param( "roll_m2_dir", roll_m2_dir, 1.0 );
	
	n.param( "yaw_m1_dir", yaw_m1_dir, -1.0 );
	n.param( "yaw_m2_dir", yaw_m2_dir, 1.0 );
	
	n.param( "speed_axis_dir", speed_axis_dir, 1.0 );
	n.param( "strafe_axis_dir", strafe_axis_dir, 1.0 );
	n.param( "depth_axis_dir", depth_axis_dir, -1.0 );
	n.param( "roll_axis_dir", roll_axis_dir, -1.0 );
	n.param( "pitch_axis_dir", pitch_axis_dir, -1.0 );
	n.param( "yaw_axis_dir", yaw_axis_dir, -1.0 );
	
	pid_D->initPid( pid_D_cfg.p, pid_D_cfg.i, pid_D_cfg.d, pid_i_max, pid_i_min );
	pid_D->reset();
	pid_R->initPid( pid_R_cfg.p, pid_R_cfg.i, pid_R_cfg.d, pid_i_max, pid_i_min );
	pid_R->reset();
	pid_P->initPid( pid_P_cfg.p, pid_P_cfg.i, pid_P_cfg.d, pid_i_max, pid_i_min );
	pid_P->reset();
	pid_Y->initPid( pid_Y_cfg.p, pid_Y_cfg.i, pid_Y_cfg.d, pid_i_max, pid_i_min );
	pid_Y->reset();
	
	motorCntlMsg = new seabee3_driver_base::MotorCntl;
	
	for ( int i = 0; i < BeeStem3::NUM_MOTOR_CONTROLLERS; i++ )
	{
		motorCntlMsg->motors[i] = 0;
		motorCntlMsg->mask[i] = 0;
	}

	ros::Publisher motor_cntl_pub = n.advertise<seabee3_driver_base::MotorCntl> ( "/seabee3/motor_cntl", 1 );
	
	
	//tf::TransformBroadcaster tb;
	//estimatedPose.setRotation( estimatedPose.getRotation().normalize() );

	SetDesiredPose(); //set current xyz to 0, desired RPY to current RPY

	while ( ros::ok() )
	{
		yawPidStep();

		motor_cntl_pub.publish( *motorCntlMsg );
		
		
		//tb.sendTransform(tf::StampedTransform(estimatedPose, ros::Time::now(), "/seabee3/odom", "/seabee3/base_link") );

		ros::spinOnce();
		ros::Rate( 20 ).sleep();
	}

	ros::spin();
	return 0;
}
