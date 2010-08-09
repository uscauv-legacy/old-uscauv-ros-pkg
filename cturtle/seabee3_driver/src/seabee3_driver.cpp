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

std::string global_frame;

geometry_msgs::Twist twistCache;

seabee3_driver_base::MotorCntl motorCntlMsg;

geometry_msgs::Vector3 desiredRPY, errorInRPY, desiredChangeInRPYPerSec, desiredXYZ, errorInXYZ, desiredChangeInXYZPerSec;
geometry_msgs::Vector3 maxErrorInRPY, maxErrorInXYZ;

ros::Time lastPidUpdateTime;

tf::Transform desiredPoseTf;
tf::Transform currentPoseTf;

geometry_msgs::Twist desiredPose;
geometry_msgs::Twist currentPose;

tf::TransformListener * tl;
tf::TransformBroadcaster * tb;

bool depthInitialized;

struct ThrusterArrayCfg // define the direction of each thruster in an array that is responsible for controlling a single axis of movement
{
	std::vector<double> thrusters;
	double & at( const unsigned int & i )
	{
		if(thrusters.size() < i)
		{
			thrusters.resize(i);
		}
		return thrusters.at(i - 1);
	}
};

std::vector<ThrusterArrayCfg> thrusterDirCfg;

double axisDirCfg [6];

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

	inline void initPid(double i_min, double i_max) { pid.initPid( cfg.p, cfg.i, cfg.d, i_min, i_max ); };
	inline void reset() { pid.reset(); };
};

struct Pid3D
{
	ConfiguredPid x, y, z;

	inline void initPid(double i_min, double i_max) { x.initPid(i_min, i_max); y.initPid(i_min, i_max); z.initPid(i_min, i_max); };
	inline void reset() { x.reset(); y.reset(); z.reset(); };
};

Pid3D xyzPid, rpyPid;

void operator >> ( const geometry_msgs::Twist & thePose, tf::Transform & thePoseTf )
{
	thePoseTf.setOrigin( tf::Vector3( thePose.linear.x, thePose.linear.y, thePose.linear.z ) );
	thePoseTf.setRotation( tf::Quaternion( thePose.angular.z, thePose.angular.y, thePose.angular.x ) );
}

void operator >> ( const tf::Transform & thePoseTf, geometry_msgs::Twist & thePose )
{
	thePose.linear.x = thePoseTf.getOrigin().x();
	thePose.linear.y = thePoseTf.getOrigin().y();
	thePose.linear.z = thePoseTf.getOrigin().z();

	thePoseTf.getBasis().getEulerZYX( thePose.angular.z, thePose.angular.y, thePose.angular.x );
}

void operator *= ( geometry_msgs::Vector3 & v, const double & scale )
{
	v.x = v.x * scale;
	v.y = v.y * scale;
	v.z = v.z * scale;
}

geometry_msgs::Twist operator * ( geometry_msgs::Twist & twist, const double & scale )
{
	twist.linear *= scale;
	twist.angular *= scale;
}

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
		motor1_scale = thrusterDirCfg[Axes::speed].at(1);
		motor2_scale = thrusterDirCfg[Axes::speed].at(2);
		break;
	case Axes::strafe: //absolute; relative to the world
		updateMotorCntlMsg( msg, Axes::strafe_rel, value );
		//updateMotorCntlMsg(msg, Axes::strafe_rel, value * cos( LocalizationUtil::degToRad( IMUDataCache->ori.x ) ) );
		//updateMotorCntlMsg(msg, Axes::depth_rel, value * -sin( LocalizationUtil::degToRad( IMUDataCache->ori.x ) ) );
		return;
	case Axes::strafe_rel: //relative to the robot
		motor1 = BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER;
		motor2 = BeeStem3::MotorControllerIDs::STRAFE_BACK_THRUSTER;
		motor1_scale = thrusterDirCfg[Axes::strafe].at(1);
		motor2_scale = thrusterDirCfg[Axes::strafe].at(2);
		break;
	case Axes::depth: //absolute; relative to the world
		updateMotorCntlMsg( msg, Axes::depth_rel, value * -1.0 );
		//updateMotorCntlMsg(msg, Axes::depth_rel, value * -cos( LocalizationUtil::degToRad( IMUDataCache->ori.x ) ) );
		//updateMotorCntlMsg(msg, Axes::strafe_rel, value * -sin( LocalizationUtil::degToRad( IMUDataCache->ori.x ) ) );
		return;
	case Axes::depth_rel: //relative to the robot
		motor1 = BeeStem3::MotorControllerIDs::DEPTH_RIGHT_THRUSTER;
		motor2 = BeeStem3::MotorControllerIDs::DEPTH_LEFT_THRUSTER;
		motor1_scale = thrusterDirCfg[Axes::depth].at(1);
		motor2_scale = thrusterDirCfg[Axes::depth].at(2);
		break;
	case Axes::roll: //absolute; relative to the world
		motor1 = BeeStem3::MotorControllerIDs::DEPTH_RIGHT_THRUSTER;
		motor2 = BeeStem3::MotorControllerIDs::DEPTH_LEFT_THRUSTER;
		motor1_scale = thrusterDirCfg[Axes::roll].at(1);
		motor2_scale = thrusterDirCfg[Axes::roll].at(2);
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
		motor1_scale = thrusterDirCfg[Axes::yaw].at(1);
		motor2_scale = thrusterDirCfg[Axes::yaw].at(2);
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

bool SetDesiredPoseCallback( seabee3_driver::SetDesiredPose::Request & req, seabee3_driver::SetDesiredPose::Response & resp )
{
	//desiredPoseTf >> desiredPose;

	if(req.pos.mask.x != 0.0)
		desiredPose.linear.x =  req.pos.values.x + ( req.pos.mode.x == 1.0 ? desiredPose.linear.x : 0.0 );
	if(req.pos.mask.y != 0.0)
		desiredPose.linear.y = req.pos.values.y + ( req.pos.mode.y == 1.0 ? desiredPose.linear.y : 0.0 );
	if(req.pos.mask.z != 0.0)
		desiredPose.linear.z = req.pos.values.z + ( req.pos.mode.z == 1.0 ? desiredPose.linear.z : 0.0 );

	if(req.ori.mask.x != 0.0)
		desiredPose.angular.x = req.ori.values.x + ( req.ori.mode.x == 1.0 ? desiredPose.angular.x : 0.0 );
	if(req.ori.mask.y != 0.0)
		desiredPose.angular.y = req.ori.values.y + ( req.ori.mode.y == 1.0 ? desiredPose.angular.y : 0.0 );
	if(req.ori.mask.z != 0.0)
		desiredPose.angular.z = req.ori.values.z + ( req.ori.mode.z == 1.0 ? desiredPose.angular.z : 0.0 );

	//desiredPose >> desiredPoseTf;
	return true;
}

void SetDesiredPose(const geometry_msgs::Twist & msg)
{
	seabee3_driver::SetDesiredPose::Request req;
	seabee3_driver::SetDesiredPose::Response resp;
	seabee3_driver::Vector3Masked changeInDesiredPose;

	changeInDesiredPose.mask.x = changeInDesiredPose.mask.y = changeInDesiredPose.mask.z = 1; //enable all values
	changeInDesiredPose.mode.x = changeInDesiredPose.mode.y = changeInDesiredPose.mode.z = 1; //set mode to incremental

	req.pos = req.ori = changeInDesiredPose;

	req.pos.values = msg.linear;
	req.ori.values = msg.angular;

	SetDesiredPoseCallback(req, resp);
}

void fetchTfFrame(tf::Transform transform, const std::string & frame1, const std::string & frame2)
{
	tf::StampedTransform temp;
	try
	{
		tl->lookupTransform( frame1, frame2, ros::Time(0), temp );

		transform.setOrigin( temp.getOrigin() );
		transform.setRotation( temp.getRotation() );
	}
	catch( tf::TransformException ex )
	{
		ROS_ERROR( "%s", ex.what() );
	}
}

void pidStep()
{
	fetchTfFrame( desiredPoseTf, "seabee3/landmark_map", "seabee3/desired_pose" );
	fetchTfFrame( currentPoseTf, "seabee3/landmark_map", "seabee3/base_link" );

	desiredPoseTf >> desiredPose;
	currentPoseTf >> currentPose;

	if ( lastPidUpdateTime != ros::Time( -1 ) )
	{
		for ( int i = 0; i < BeeStem3::NUM_MOTOR_CONTROLLERS; i++ )
		{
			motorCntlMsg.mask[i] = 0;
			motorCntlMsg.motors[i] = 0;
		}

		ros::Duration dt = ros::Time::now() - lastPidUpdateTime;
		
		//twistCache is essentially the linear and angular change in desired pose
		//we multiply by the change in time and an arbitrary constant to obtain the desired change in pose per second
		//this change in pose is then added to the current pose; the result is desiredPose
		const double t1 = dt.toSec() * -100.0;
		geometry_msgs::Twist changeInDesiredPose = twistCache * t1;
		SetDesiredPose(  changeInDesiredPose );

		LocalizationUtil::normalizeAngle( desiredPose.angular.x );
		LocalizationUtil::normalizeAngle( desiredPose.angular.y );
		LocalizationUtil::normalizeAngle( desiredPose.angular.z );

		errorInXYZ.x = desiredPose.linear.x - currentPose.linear.x;
		errorInXYZ.y = desiredPose.linear.y - currentPose.linear.y;
		errorInXYZ.z = desiredPose.linear.z - currentPose.linear.z;
		

		errorInRPY.x = LocalizationUtil::angleDistRel( desiredPose.angular.x, currentPose.angular.x );
		errorInRPY.y = LocalizationUtil::angleDistRel( desiredPose.angular.y, currentPose.angular.y );
		errorInRPY.z = LocalizationUtil::angleDistRel( desiredPose.angular.z, currentPose.angular.z );

		LocalizationUtil::capValue( errorInXYZ.x, maxErrorInXYZ.x );
		LocalizationUtil::capValue( errorInXYZ.y, maxErrorInXYZ.y );
		LocalizationUtil::capValue( errorInXYZ.z, maxErrorInXYZ.z );

		LocalizationUtil::capValue( errorInRPY.x, maxErrorInRPY.x );
		LocalizationUtil::capValue( errorInRPY.y, maxErrorInRPY.y );
		LocalizationUtil::capValue( errorInRPY.z, maxErrorInRPY.z );

		double speedMotorVal = axisDirCfg[Axes::speed] * xyzPid.x.pid.updatePid( errorInXYZ.x, dt );
		double strafeMotorVal = axisDirCfg[Axes::strafe] * xyzPid.y.pid.updatePid( errorInXYZ.y, dt );
		double depthMotorVal = axisDirCfg[Axes::depth] * xyzPid.z.pid.updatePid( errorInXYZ.z, dt );

		double rollMotorVal = axisDirCfg[Axes::roll] * rpyPid.x.pid.updatePid( errorInRPY.x, dt );
		double pitchMotorVal = axisDirCfg[Axes::pitch] * rpyPid.y.pid.updatePid( errorInRPY.y, dt );
		double yawMotorVal = axisDirCfg[Axes::yaw] * rpyPid.z.pid.updatePid( errorInRPY.z, dt );

		/*LocalizationUtil::capValue(rollMotorVal, 50.0);
		 LocalizationUtil::capValue(pitchMotorVal, 50.0);
		 LocalizationUtil::capValue(yawMotorVal, 50.0);
		 LocalizationUtil::capValue(depthMotorVal, 50.0);*/

		updateMotorCntlMsg( motorCntlMsg, Axes::speed, speedMotorVal );
		updateMotorCntlMsg( motorCntlMsg, Axes::strafe, strafeMotorVal );
		updateMotorCntlMsg( motorCntlMsg, Axes::depth, depthMotorVal );
		//updateMotorCntlMsg( motorCntlMsg, Axes::roll, rollMotorVal );
		//updateMotorCntlMsg( motorCntlMsg, Axes::pitch, pitchMotorVal );
		updateMotorCntlMsg( motorCntlMsg, Axes::yaw, yawMotorVal );

		desiredPose >> desiredPoseTf;
		tb->sendTransform( tf::StampedTransform( desiredPoseTf, ros::Time::now(), global_frame, "seabee3/desired_pose" ) );
	}
	lastPidUpdateTime = ros::Time::now();
}

void CmdVelCallback( const geometry_msgs::TwistConstPtr & twist )
{
	twistCache = *twist;
}

int main( int argc, char** argv )
{
	ros::init( argc, argv, "seabee3_driver" );
	ros::NodeHandle n;
	ros::NodeHandle n_priv( "~" ); // allow direct access to parameters

	tl = new tf::TransformListener;
	tb = new tf::TransformBroadcaster;
	
	ros::Subscriber cmd_vel_sub = n.subscribe( "/seabee3/cmd_vel", 1, CmdVelCallback );
	
	lastPidUpdateTime = ros::Time( -1 );
	
	depthInitialized = false;
	
	double pid_i_min, pid_i_max;
	
	n_priv.param( "global_frame", global_frame, std::string("/landmark_map") );
	
	n_priv.param( "speed_err_cap", maxErrorInXYZ.x, 100.0 );
	n_priv.param( "strafe_err_cap", maxErrorInXYZ.y, 100.0 );
	n_priv.param( "depth_err_cap", maxErrorInXYZ.z, 100.0 );

	n_priv.param( "roll_err_cap", maxErrorInRPY.x, 25.0 );
	n_priv.param( "pitch_err_cap", maxErrorInRPY.y, 25.0 );
	n_priv.param( "yaw_err_cap", maxErrorInRPY.z, 25.0 );
	
	n_priv.param( "pid/pos/X/p", xyzPid.x.cfg.p, 2.5 );
	n_priv.param( "pid/pos/X/i", xyzPid.x.cfg.i, 0.05 );
	n_priv.param( "pid/pos/X/d", xyzPid.x.cfg.d, 0.2 );

	n_priv.param( "pid/pos/Y/p", xyzPid.y.cfg.p, 2.5 );
	n_priv.param( "pid/pos/Y/i", xyzPid.y.cfg.i, 0.05 );
	n_priv.param( "pid/pos/Y/d", xyzPid.y.cfg.d, 0.2 );

	n_priv.param( "pid/pos/Z/p", xyzPid.z.cfg.p, 2.5 );
	n_priv.param( "pid/pos/Z/i", xyzPid.z.cfg.i, 0.05 );
	n_priv.param( "pid/pos/Z/d", xyzPid.z.cfg.d, 0.2 );
	
	n_priv.param( "pid/ori/R/p", rpyPid.x.cfg.p, 2.5 );
	n_priv.param( "pid/ori/R/i", rpyPid.x.cfg.i, 0.05 );
	n_priv.param( "pid/ori/R/d", rpyPid.x.cfg.d, 0.2 );
	
	n_priv.param( "pid/ori/P/p", rpyPid.y.cfg.p, 2.5 );
	n_priv.param( "pid/ori/P/i", rpyPid.y.cfg.i, 0.05 );
	n_priv.param( "pid/ori/P/d", rpyPid.y.cfg.d, 0.2 );
	
	n_priv.param( "pid/ori/Y/p", rpyPid.z.cfg.p, 2.5 );
	n_priv.param( "pid/ori/Y/i", rpyPid.z.cfg.i, 0.05 );
	n_priv.param( "pid/ori/Y/d", rpyPid.z.cfg.d, 0.2 );
	
	n_priv.param( "pid/i_max", pid_i_max, 1.0 );
	n_priv.param( "pid/i_min", pid_i_min, -1.0 );
	
	n_priv.param( "speed_m1_dir", thrusterDirCfg[Axes::speed].at(1), 1.0 );
	n_priv.param( "speed_m2_dir", thrusterDirCfg[Axes::speed].at(2), 1.0 );

	n_priv.param( "strafe_m1_dir", thrusterDirCfg[Axes::strafe].at(1), 1.0 );
	n_priv.param( "strafe_m2_dir", thrusterDirCfg[Axes::strafe].at(2), 1.0 );
	
	n_priv.param( "depth_m1_dir", thrusterDirCfg[Axes::depth].at(1), 1.0 );
	n_priv.param( "depth_m2_dir", thrusterDirCfg[Axes::depth].at(2), 1.0 );
	
	n_priv.param( "roll_m1_dir", thrusterDirCfg[Axes::roll].at(1), 1.0 );
	n_priv.param( "roll_m2_dir", thrusterDirCfg[Axes::roll].at(2), 1.0 );
	
	n_priv.param( "pitch_m1_dir", thrusterDirCfg[Axes::pitch].at(1), 1.0 );
	n_priv.param( "pitch_m2_dir", thrusterDirCfg[Axes::pitch].at(2), 1.0 );
	
	n_priv.param( "yaw_m1_dir", thrusterDirCfg[Axes::yaw].at(1), -1.0 );
	n_priv.param( "yaw_m2_dir", thrusterDirCfg[Axes::yaw].at(2), 1.0 );
	
	n_priv.param( "speed_axis_dir", axisDirCfg[Axes::speed], 1.0 );
	n_priv.param( "strafe_axis_dir", axisDirCfg[Axes::strafe], 1.0 );
	n_priv.param( "depth_axis_dir", axisDirCfg[Axes::depth], -1.0 );
	n_priv.param( "roll_axis_dir", axisDirCfg[Axes::roll], -1.0 );
	n_priv.param( "pitch_axis_dir", axisDirCfg[Axes::pitch], -1.0 );
	n_priv.param( "yaw_axis_dir", axisDirCfg[Axes::yaw], -1.0 );
	
	xyzPid.initPid(pid_i_min, pid_i_max);
	xyzPid.reset();

	rpyPid.initPid(pid_i_min, pid_i_max);
	rpyPid.reset();
	
	for ( int i = 0; i < BeeStem3::NUM_MOTOR_CONTROLLERS; i++ )
	{
		motorCntlMsg.motors[i] = 0;
		motorCntlMsg.mask[i] = 0;
	}

	ros::Publisher motor_cntl_pub = n.advertise<seabee3_driver_base::MotorCntl> ( "/seabee3/motor_cntl", 1 );
	
	SetDesiredPose( geometry_msgs::Twist() ); //set current xyz to 0, desired RPY to current RPY

	while ( ros::ok() )
	{
		pidStep(); //this also grabs and publishes tf frames

		motor_cntl_pub.publish( motorCntlMsg );

		ros::spinOnce();
		ros::Rate( 20 ).sleep();
	}

	ros::spin();
	return 0;
}
