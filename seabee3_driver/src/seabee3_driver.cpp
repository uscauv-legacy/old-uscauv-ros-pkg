/*******************************************************************************
 *
 *      seabee3_driver
 * 
 *      Copyright (c) 2010, Edward T. Kaszubski (ekaszubski@gmail.com),
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

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <control_toolbox/pid.h>

#include <xsens_node/IMUData.h>
#include <seabee3_driver/SetDesiredXYZ.h>
#include <seabee3_driver/SetDesiredRPY.h>
#include <seabee3_driver_base/MotorCntl.h>
#include <seabee3_driver_base/Depth.h>
#include <seabee3_driver_base/KillSwitch.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <xsens_node/SetRPYOffset.h>

#include <seabee3_beestem/BeeStem3.h>

#include <localization_tools/Util.h>

#include <seabee3_driver/ResetPose.h>

xsens_node::IMUData * IMUDataCache;
geometry_msgs::Twist * TwistCache;
//ros::Time * velocity_sample_start;
seabee3_driver_base::MotorCntl * motorCntlMsg;
seabee3_driver_base::Depth * depthCache;
seabee3_driver_base::KillSwitch * killSwitchCache;
geometry_msgs::Vector3 * desiredRPY, * errorInRPY, * desiredChangeInRPYPerSec, * desiredXYZ, * errorInXYZ, * desiredChangeInXYZPerSec;
double maxRollError, maxPitchError, maxHeadingError, maxDepthError, desiredSpeed, desiredStrafe;
ros::Time * lastUpdateTime;
ros::Time * lastPidUpdateTime;
ros::Time lastCmdVelUpdateTime;
tf::Vector3 mImuOriOffset;

double 	speed_m1_dir, speed_m2_dir,
		strafe_m1_dir, strafe_m2_dir,
		depth_m1_dir, depth_m2_dir,
		roll_m1_dir, roll_m2_dir,
		heading_m1_dir, heading_m2_dir;

double speed_axis_dir, strafe_axis_dir, depth_axis_dir, roll_axis_dir, pitch_axis_dir, heading_axis_dir, cmdVelTimeout;

bool depthInitialized;

control_toolbox::Pid * pid_D, * pid_R, * pid_P, * pid_Y;

//tf::Transform estimatedPose;

//tf::Vector3 * vel_est_lin;
//tf::Vector3 * vel_est_ang;

#define axis_speed 0
#define axis_strafe 1
#define axis_depth 2
#define axis_roll 3
#define axis_pitch 4
#define axis_heading 5

#define axis_depth_rel 6
#define axis_strafe_rel 7
#define axis_heading_rel 8

struct PIDConfig
{
	double p;
	double i;
	double d;
};

void updateMotorCntlMsg(seabee3_driver_base::MotorCntl & msg, int axis, int p_value)
{
	int value = p_value;
	
	int motor1, motor2;
	double motor1_scale = 1.0, motor2_scale = 1.0;
	
	switch(axis)
	{
		case axis_speed: //relative to the robot
			motor1 = BeeStem3::MotorControllerIDs::FWD_RIGHT_THRUSTER;
			motor2 = BeeStem3::MotorControllerIDs::FWD_LEFT_THRUSTER;
			motor1_scale = speed_m1_dir;
			motor2_scale = speed_m2_dir;
			break;
		case axis_strafe: //absolute; relative to the world
			updateMotorCntlMsg(msg, axis_strafe_rel, value );
			//updateMotorCntlMsg(msg, axis_strafe_rel, value * cos( LocalizationUtil::degToRad( IMUDataCache->ori.x ) ) );
			//updateMotorCntlMsg(msg, axis_depth_rel, value * -sin( LocalizationUtil::degToRad( IMUDataCache->ori.x ) ) );
			return;
		case axis_strafe_rel: //relative to the robot
			motor1 = BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER;
			motor2 = BeeStem3::MotorControllerIDs::STRAFE_BACK_THRUSTER;
			motor1_scale = strafe_m1_dir;
			motor2_scale = strafe_m2_dir;
			break;
		case axis_depth: //absolute; relative to the world
			updateMotorCntlMsg(msg, axis_depth_rel, value * -1.0 );
			//updateMotorCntlMsg(msg, axis_depth_rel, value * -cos( LocalizationUtil::degToRad( IMUDataCache->ori.x ) ) );
			//updateMotorCntlMsg(msg, axis_strafe_rel, value * -sin( LocalizationUtil::degToRad( IMUDataCache->ori.x ) ) );
			return;
		case axis_depth_rel: //relative to the robot
			motor1 = BeeStem3::MotorControllerIDs::DEPTH_RIGHT_THRUSTER;
			motor2 = BeeStem3::MotorControllerIDs::DEPTH_LEFT_THRUSTER;
			motor1_scale = depth_m1_dir;
			motor2_scale = depth_m2_dir;
			break;
		case axis_roll: //absolute; relative to the world
			motor1 = BeeStem3::MotorControllerIDs::DEPTH_RIGHT_THRUSTER;
			motor2 = BeeStem3::MotorControllerIDs::DEPTH_LEFT_THRUSTER;
			motor1_scale = roll_m1_dir;
			motor2_scale = roll_m2_dir;
			break;
		case axis_pitch: //pitch is only available as seabee starts to roll; make sure that's the case here
			updateMotorCntlMsg(msg, axis_heading_rel, value * -sin( LocalizationUtil::degToRad( IMUDataCache->ori.x ) ) );
			return;
		case axis_heading: //absolute; relative to the world
			//updateMotorCntlMsg(msg, axis_heading_rel, value * cos( LocalizationUtil::degToRad( IMUDataCache->ori.x ) ) );
			updateMotorCntlMsg(msg, axis_heading_rel, value );
			return;
		case axis_heading_rel: //relative to the robot
			motor1 = BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER;
			motor2 = BeeStem3::MotorControllerIDs::STRAFE_BACK_THRUSTER;
			motor1_scale = heading_m1_dir;
			motor2_scale = heading_m2_dir;
			break;
	}
	
	int motor1_val = msg.motors[motor1];
	int motor2_val = msg.motors[motor2];
	
	motor1_val += motor1_scale * value;
	motor2_val += motor2_scale * value;
	
	LocalizationUtil::capValueProp(motor1_val, motor2_val, 100);
	
	msg.motors[motor1] = motor1_val;
	msg.motors[motor2] = motor2_val;
	
	msg.mask[motor1] = 1;
	msg.mask[motor2] = 1;
}

void updateMotorCntlFromTwist(const geometry_msgs::TwistConstPtr & twist)
{
	//ROS_INFO("udpateMotorCntlFromTwist()");
	tf::Vector3 vel_desired_lin (twist->linear.x, twist->linear.y, twist->linear.z);
	tf::Vector3 vel_desired_ang (twist->angular.x, twist->angular.y, twist->angular.z);
	//tf::Vector3 vel_diff_lin (vel_desired_lin - *vel_est_lin);
	//tf::Vector3 vel_diff_ang (vel_desired_ang - *vel_est_ang);
	
	/*ROS_INFO("-----------------------------------------------------");
	
	ROS_INFO("linear x %f y %f z %f", vel_desired_lin.getX(), vel_desired_lin.getY(), vel_desired_lin.getZ());
	ROS_INFO("angular x %f y %f z %f", vel_desired_ang.getX(), vel_desired_ang.getY(), vel_desired_ang.getZ());*/
	
	desiredSpeed = vel_desired_lin.getX() * 100.0;
	desiredStrafe = vel_desired_lin.getY() * 100.0;
	//*desiredDepth = vel_diff_lin.getZ() * 100.0;
	
	//updateMotorCntlMsg(*motorCntlMsg, axis_speed, vel_diff_lin.getX() * 100.0);
	//updateMotorCntlMsg(*motorCntlMsg, axis_strafe, vel_diff_lin.getY() * 50.0);
	//updateMotorCntlMsg(*motorCntlMsg, axis_depth, vel_diff_lin.getZ() * 100.0);
	//updateMotorCntlMsg(*motorCntlMsg, axis_roll, vel_diff_ang.getX() * 100.0);
	//updateMotorCntlMsg(*motorCntlMsg, axis_heading, vel_diff_ang.getZ() * 100.0);

	if(*lastUpdateTime != ros::Time(-1))
	{
		//ros::Duration dt = ros::Time::now() - *lastHeadingUpdateTime;
		
		//desiredChangeInXYZPerSec->x = 100.0 * twist->linear.x;
		//desiredChangeInXYZPerSec->y = 100.0 * twist->linear.y;
		desiredChangeInXYZPerSec->z = 100.0 * twist->linear.z;
		
		//desiredChangeInRPYPerSec->x = 100.0 * twist->angular.x;
		//desiredChangeInRPYPerSec->y = 100.0 * twist->angular.y;
		desiredChangeInRPYPerSec->z = 100.0 * twist->angular.z;

		//ROS_INFO("heading: %f desired heading: %f dt: %f", IMUDataCache->ori.z, *desiredRPY.z, dt.toSec());
		
//		if(!IMUDataCache->empty())
//			ROS_INFO("desired heading: %f actualHeading: %f dt: %f", *desiredRPY.z, IMUDataCache->front().ori.z, dt.toSec());
	}
	
	*lastUpdateTime = ros::Time::now();
}

void headingPidStep()
{
	if(*lastPidUpdateTime != ros::Time(-1))
	{
		for(int i = 0; i < BeeStem3::NUM_MOTOR_CONTROLLERS; i ++)
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
		LocalizationUtil::normalizeAngle(desiredRPY->z);

		//actual - desired; 0 - 40 = -40;
		//desired -> actual; 40 -> 0 = 40 cw = -40
		//double headingError = LocalizationUtil::angleDistRel(desiredRPY->z, IMUDataCache->ori.z);
		
		//errorInXYZ->x = desiredXYZ->y - <some sensor value>;
		//errorInXYZ->y = desiredXYZ->x - <some sensor value>;
		errorInXYZ->z = desiredXYZ->z - depthCache->Value;
		
		//errorInRPY->x = LocalizationUtil::angleDistRel(desiredRPY->x, IMUDataCache->ori.x);
		//errorInRPY->y = LocalizationUtil::angleDistRel(desiredRPY->y, IMUDataCache->ori.y);
		errorInRPY->z = LocalizationUtil::angleDistRel(desiredRPY->z, IMUDataCache->ori.z);
		
		//LocalizationUtil::capValue(errorInDepth->x, maxXError);
		//LocalizationUtil::capValue(errorInDepth->y, maxYError);
		LocalizationUtil::capValue(errorInXYZ->z, maxDepthError);
		
		//LocalizationUtil::capValue(errorInRPY->x, maxRollError);
		//LocalizationUtil::capValue(errorInRPY->y, maxPitchError);
		LocalizationUtil::capValue(errorInRPY->z, maxHeadingError);
		
		//headingError = abs(headingError) > *maxHeadingError ? *maxHeadingError * headingError / abs(headingError) : headingError;
		
		double speedMotorVal = speed_axis_dir * desiredSpeed;
		double strafeMotorVal = strafe_axis_dir * desiredStrafe;
		double depthMotorVal = depth_axis_dir * pid_D->updatePid(errorInXYZ->z, dt);
		//double rollMotorVal = roll_axis_dir * pid_R->updatePid(errorInRPY->x, dt);
		//double pitchMotorVal = pitch_axis_dir * pid_P->updatePid(errorInRPY->y, dt);
		double headingMotorVal = heading_axis_dir * pid_Y->updatePid(errorInRPY->z, dt);

		//ROS_INFO("initial motor val: %f", motorVal);
		
		/*LocalizationUtil::capValue(rollMotorVal, 50.0);
		LocalizationUtil::capValue(pitchMotorVal, 50.0);
		LocalizationUtil::capValue(headingMotorVal, 50.0);
		LocalizationUtil::capValue(depthMotorVal, 50.0);*/
		
		//motorVal = abs(motorVal) > 100 ? 100 * motorVal / abs(motorVal) : motorVal;

		//ROS_INFO("heading: %f desired heading: %f heading error: %f motorValue: %f", IMUDataCache->ori.z, desiredRPY->z, errorInRPY->z, headingMotorVal);
		
		//ROS_INFO("Depth; desired: %d current %d error %d motorVal %f", *desiredDepth, depthCache->Value, *errorInDepth, depthMotorVal);
		
		updateMotorCntlMsg(*motorCntlMsg, axis_speed, speedMotorVal);
		updateMotorCntlMsg(*motorCntlMsg, axis_strafe, strafeMotorVal);
		updateMotorCntlMsg(*motorCntlMsg, axis_depth, depthMotorVal);
		//updateMotorCntlMsg(*motorCntlMsg, axis_roll, rollMotorVal);
		//updateMotorCntlMsg(*motorCntlMsg, axis_pitch, pitchMotorVal);
		updateMotorCntlMsg(*motorCntlMsg, axis_heading, headingMotorVal);

		//ROS_INFO("heading error: %f motorVal: %f", headingError, motorVal);
	}
	*lastPidUpdateTime = ros::Time::now();
}

void IMUDataCallback(const xsens_node::IMUDataConstPtr & data)
{
//	ROS_INFO("IMUDataCallback()");
	*IMUDataCache = *data;
	
	IMUDataCache->ori.x += mImuOriOffset.x();
	IMUDataCache->ori.y += mImuOriOffset.y();
	IMUDataCache->ori.z += mImuOriOffset.z();
	
	while(IMUDataCache->ori.z > 360)
		IMUDataCache->ori.z -= 360;
	while(IMUDataCache->ori.z < 0)
		IMUDataCache->ori.z += 360;
	
	//estimatedPose.setRotation( tf::Quaternion( IMUDataCache->ori.z, IMUDataCache->ori.y, IMUDataCache->ori.x ) );
	//ROS_INFO("x %f y %f z %f", IMUDataCache->ori.x, IMUDataCache->ori.y, IMUDataCache->ori.z);
	//while(IMUDataCache->size() > 5)
	//{
	//	IMUDataCache->pop();
	//}
}

void CmdVelCallback(const geometry_msgs::TwistConstPtr & twist)
{
	//ROS_INFO("CmdVelCallback()");
	*TwistCache = *twist;
	//while(TwistCache->size() > 5)
	//{
	//	TwistCache->pop();
	//}
	
	updateMotorCntlFromTwist(twist);
	
	/*
	lastCmdVelUpdateTime = ros::Time::now();
	static bool timeoutSet = false;
	
	ros::Duration dt = ros::Time::now() - lastCmdVelUpdateTime;
		
	if(dt.toSec() > cmdVelTimeout)
	{
		if(!timeoutSet)
		{
			ROS_INFO("timed out after %f seconds", dt.toSec());
			
			for(int i = 0; i < BeeStem3::NUM_MOTOR_CONTROLLERS; i ++)
			{
				motorCntlMsg->motors[i] = 0;
				motorCntlMsg->mask[i] = 1;
			}
			timeoutSet = true;
		}
	}
	else
	{
		//ROS_INFO("publishing as usual...");
		timeoutSet = false;
		updateMotorCntlFromTwist(twist);
	}
	
	lastCmdVelUpdateTime = ros::Time::now();
	*/
}

bool setDesiredXYZCallback(seabee3_driver::SetDesiredXYZ::Request & req, seabee3_driver::SetDesiredXYZ::Response & resp)
{
	//if(req.Mask.x > 0.0f)
	//	desiredXYZ->x = req.DesiredXYZ.x + (req.Mode.x == 1.0f ? IMUDataCache->ori.x : 0);
			
	//if(req.Mask.y > 0.0f)
	//	desiredXYZ->y = req.DesiredXYZ.y + (req.Mode.y == 1.0f ? IMUDataCache->ori.y : 0);
			
	if(req.Mask.z > 0.0f)
		desiredXYZ->z = req.DesiredXYZ.z + (req.Mode.z == 1.0f ? depthCache->Value : 0);
		
	resp.CurrentDesiredXYZ = *desiredXYZ;
	resp.ErrorInXYZ = *errorInXYZ;
	return true;
}

bool setDesiredRPYCallback(seabee3_driver::SetDesiredRPY::Request & req, seabee3_driver::SetDesiredRPY::Response & resp)
{
	if(req.Mask.x > 0.0f)
		desiredRPY->x = req.DesiredRPY.x + (req.Mode.x == 1.0f ? IMUDataCache->ori.x : 0);
			
	if(req.Mask.y > 0.0f)
		desiredRPY->y = req.DesiredRPY.y + (req.Mode.y == 1.0f ? IMUDataCache->ori.y : 0);
			
	if(req.Mask.z > 0.0f)
		desiredRPY->z = req.DesiredRPY.z + (req.Mode.z == 1.0f ? IMUDataCache->ori.z : 0);
		
	resp.CurrentDesiredRPY = *desiredRPY;
	resp.ErrorInRPY = *errorInRPY;
	return true;
}

void DepthCallback(const seabee3_driver_base::DepthConstPtr & depth)
{
	*depthCache = *depth;
	if(!depthInitialized)
	{
		desiredXYZ->z = depthCache->Value;
		depthInitialized = true;
	}
}

/*void OdomPrimCallback(const geometry_msgs::Vector3ConstPtr & odomPrim)
{
	tf::Vector3 currentPose = estimatedPose.getOrigin();
	//odom points from where we were before we started moving to our current position
	estimatedPose.setOrigin( tf::Vector3( currentPose.x() + odomPrim->x, currentPose.y() + odomPrim->y, currentPose.z() + odomPrim->z ) );
	estimatedPose.setRotation( estimatedPose.getRotation().normalize() );
}*/

bool ResetPoseCallback(seabee3_driver::ResetPose::Request & req, seabee3_driver::ResetPose::Response & resp)
{
	/*tf::Vector3 thePose = estimatedPose.getOrigin();
	if(req.Pos.Mask.x == 1.0)
		thePose.setX(req.Pos.Values.x);
	if(req.Pos.Mask.y == 1.0)
		thePose.setY(req.Pos.Values.y);
	if(req.Pos.Mask.z == 1.0)
		thePose.setZ(req.Pos.Values.z);
	estimatedPose.setOrigin(thePose);*/
	
	seabee3_driver::SetDesiredRPY setRPY;
	setRPY.request.Mode = req.Ori.Mode;
	setRPY.request.Mask = req.Ori.Mask;
	setRPY.request.DesiredRPY = req.Ori.Values;
	
	setDesiredRPYCallback(setRPY.request, setRPY.response);
	
	/*if(req.Ori.Mask.x == 1.0)
		mImuOriOffset.setX(req.Ori.Values.x - mImuOriOffset.x() );
	if(req.Ori.Mask.y == 1.0)
		mImuOriOffset.setY(req.Ori.Values.y - mImuOriOffset.y() );
	if(req.Ori.Mask.z == 1.0)
		mImuOriOffset.setZ(req.Ori.Values.z - mImuOriOffset.z() );
		
	estimatedPose.setRotation( estimatedPose.getRotation().normalize() );*/
	
	desiredXYZ->z = depthCache->Value;
	
	return true;
}

void ResetPose()
{
	seabee3_driver::ResetPose resetPose;
	resetPose.request.Ori.Mask.x = resetPose.request.Ori.Mask.y = resetPose.request.Ori.Mask.z = 1;
	resetPose.request.Ori.Mode.x = resetPose.request.Ori.Mode.y = resetPose.request.Ori.Mode.z = 1;
	ResetPoseCallback(resetPose.request, resetPose.response);
}

void KillSwitchCallback(const seabee3_driver_base::KillSwitchConstPtr & killSwitch)
{
	if(killSwitch->Value == 0 && killSwitchCache->Value == 1)
	{
		//set desired pose to our current pose so we don't move yet
		ResetPose();
	}
	*killSwitchCache = *killSwitch;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "seabee3_driver");
	ros::NodeHandle n("~");
	
	//vel_est_lin = new tf::Vector3(0, 0, 0);
	//vel_est_ang = new tf::Vector3(0, 0, 0);
	//velocity_sample_start = new ros::Time(-1);
	
	IMUDataCache = new xsens_node::IMUData;
	TwistCache = new geometry_msgs::Twist;
	
	ros::Subscriber imu_sub = n.subscribe("/xsens/data_calibrated", 1, IMUDataCallback); //likely necessary to remap this to something real ie. /xsens/data_calibrated
	//ros::Subscriber odom_prim_sub = n.subscribe("/seabee3/odom_prim", 1, OdomPrimCallback);
	ros::Subscriber cmd_vel_sub = n.subscribe("/seabee3/cmd_vel", 1, CmdVelCallback);
	ros::Subscriber extl_depth_sub = n.subscribe("/seabee3/depth", 1, DepthCallback);
	ros::Subscriber kill_switch_sub = n.subscribe("/seabee3/kill_switch", 1, KillSwitchCallback);
	
	desiredRPY = new geometry_msgs::Vector3; desiredRPY->x = 0; desiredRPY->y = 0; desiredRPY->z = 0;
	errorInRPY = new geometry_msgs::Vector3; errorInRPY->x = 0; errorInRPY->y = 0; errorInRPY->z = 0;
	desiredChangeInRPYPerSec = new geometry_msgs::Vector3; desiredChangeInRPYPerSec->x = 0; desiredChangeInRPYPerSec->y = 0; desiredChangeInRPYPerSec->z = 0;
	
	desiredXYZ = new geometry_msgs::Vector3; desiredXYZ->x = 0; desiredXYZ->y = 0; desiredXYZ->z = 0;
	errorInXYZ = new geometry_msgs::Vector3; errorInXYZ->x = 0; errorInXYZ->y = 0; errorInXYZ->z = 0;
	desiredChangeInXYZPerSec = new geometry_msgs::Vector3; desiredChangeInXYZPerSec->x = 0; desiredChangeInXYZPerSec->y = 0; desiredChangeInXYZPerSec->z = 0;
	
	lastUpdateTime = new ros::Time(ros::Time(-1));
	lastPidUpdateTime = new ros::Time(ros::Time(-1));
	depthCache = new seabee3_driver_base::Depth;
	killSwitchCache = new seabee3_driver_base::KillSwitch;
	
	depthInitialized = false;
	
	pid_D = new control_toolbox::Pid;
	pid_R = new control_toolbox::Pid;
	pid_P = new control_toolbox::Pid;
	pid_Y = new control_toolbox::Pid;
	
	PIDConfig pid_D_cfg, pid_R_cfg, pid_P_cfg, pid_Y_cfg;
	
	double pid_i_min, pid_i_max;
	
	n.param("cmd_vel_timeout", cmdVelTimeout, 2.0);
	
	n.param("depth_err_cap", maxDepthError, 100.0);
	n.param("roll_err_cap", maxRollError, 25.0);
	n.param("pitch_err_cap", maxPitchError, 25.0);
	n.param("heading_err_cap", maxHeadingError, 25.0);
	
	n.param("pid/D/p", pid_D_cfg.p, 2.5);
	n.param("pid/D/i", pid_D_cfg.i, 0.05);
	n.param("pid/D/d", pid_D_cfg.d, 0.2);
	
	n.param("pid/R/p", pid_R_cfg.p, 2.5);
	n.param("pid/R/i", pid_R_cfg.i, 0.05);
	n.param("pid/R/d", pid_R_cfg.d, 0.2);
	
	n.param("pid/P/p", pid_P_cfg.p, 2.5);
	n.param("pid/P/i", pid_P_cfg.i, 0.05);
	n.param("pid/P/d", pid_P_cfg.d, 0.2);
	
	n.param("pid/Y/p", pid_Y_cfg.p, 2.5);
	n.param("pid/Y/i", pid_Y_cfg.i, 0.05);
	n.param("pid/Y/d", pid_Y_cfg.d, 0.2);
	
	n.param("pid/i_max", pid_i_max, 1.0);
	n.param("pid/i_min", pid_i_min, -1.0);
	
	n.param("speed_m1_dir", speed_m1_dir, 1.0);
	n.param("speed_m2_dir", speed_m2_dir, 1.0);
	
	n.param("strafe_m1_dir", strafe_m1_dir, 1.0);
	n.param("strafe_m2_dir", strafe_m2_dir, 1.0);
	
	n.param("depth_m1_dir", depth_m1_dir, 1.0);
	n.param("depth_m2_dir", depth_m2_dir, 1.0);
	
	n.param("roll_m1_dir", roll_m1_dir, 1.0);
	n.param("roll_m2_dir", roll_m2_dir, 1.0);
	
	n.param("heading_m1_dir", heading_m1_dir, -1.0);
	n.param("heading_m2_dir", heading_m2_dir, 1.0);
	
	n.param("speed_axis_dir", speed_axis_dir, 1.0);
	n.param("strafe_axis_dir", strafe_axis_dir, 1.0);
	n.param("depth_axis_dir", depth_axis_dir, -1.0);
	n.param("roll_axis_dir", roll_axis_dir, -1.0);
	n.param("pitch_axis_dir", pitch_axis_dir, -1.0);
	n.param("heading_axis_dir", heading_axis_dir, -1.0);
	
	pid_D->initPid(pid_D_cfg.p, pid_D_cfg.i, pid_D_cfg.d, pid_i_max, pid_i_min);
	pid_D->reset();
	pid_R->initPid(pid_R_cfg.p, pid_R_cfg.i, pid_R_cfg.d, pid_i_max, pid_i_min);
	pid_R->reset();
	pid_P->initPid(pid_P_cfg.p, pid_P_cfg.i, pid_P_cfg.d, pid_i_max, pid_i_min);
	pid_P->reset();
	pid_Y->initPid(pid_Y_cfg.p, pid_Y_cfg.i, pid_Y_cfg.d, pid_i_max, pid_i_min);
	pid_Y->reset();
	
	motorCntlMsg = new seabee3_driver_base::MotorCntl;
	
	for(int i = 0; i < BeeStem3::NUM_MOTOR_CONTROLLERS; i ++)
	{
		motorCntlMsg->motors[i] = 0;
		motorCntlMsg->mask[i] = 0;
	}
	
	ros::Publisher motor_cntl_pub = n.advertise<seabee3_driver_base::MotorCntl>("/seabee3/motor_cntl", 1);
	ros::ServiceServer setDesiredXYZ_srv = n.advertiseService("/seabee3/setDesiredXYZ", setDesiredXYZCallback);
	ros::ServiceServer setDesiredRPY_srv = n.advertiseService("/seabee3/setDesiredRPY", setDesiredRPYCallback);
	ros::ServiceServer ResetPose_srv = n.advertiseService("/seabee3/ResetPose", ResetPoseCallback);
	
	//tf::TransformBroadcaster tb;
	//estimatedPose.setRotation( estimatedPose.getRotation().normalize() );
	
	ResetPose(); //set current xyz to 0, desired RPY to current RPY
	
	while(ros::ok())
	{
		headingPidStep();
		
		motor_cntl_pub.publish(*motorCntlMsg);
		
		//tb.sendTransform(tf::StampedTransform(estimatedPose, ros::Time::now(), "/seabee3/odom", "/seabee3/base_link") );
		
		ros::spinOnce();
		ros::Rate(20).sleep();
	}
	
	ros::spin();
	return 0;
}
