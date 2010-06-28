#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <control_toolbox/pid.h>

#include <xsens_node/IMUData.h>
#include <seabee3_driver/SetDesiredDepth.h>
#include <seabee3_driver/SetDesiredRPY.h>
#include <seabee3_driver_base/MotorCntl.h>
#include <seabee3_driver_base/Pressure.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <seabee3_beestem/BeeStem3.h>

#include <seabee3_util/seabee3_util.h>

xsens_node::IMUData * IMUDataCache;
geometry_msgs::Twist * TwistCache;
ros::Time * velocity_sample_start;
seabee3_driver_base::MotorCntl * motorCntlMsg;
seabee3_driver_base::Pressure * extlPressureCache;
geometry_msgs::Vector3 * desiredRPY, * errorInRPY, * desiredChangeInRPYPerSec;
double *maxRollError, *maxPitchError, * maxHeadingError;
int *maxDepthError, * desiredDepth, * errorInDepth, * desiredChangeInDepthPerSec;
ros::Time * lastUpdateTime;
ros::Time * lastPidUpdateTime;

control_toolbox::Pid * pid_D, * pid_R, * pid_P, * pid_Y;

tf::Vector3 * vel_est_lin;
tf::Vector3 * vel_est_ang;

#define axis_speed 0
#define axis_strafe 1
#define axis_depth 2
#define axis_roll 3
#define axis_pitch 4
#define axis_heading 5

void updateMotorCntlMsg(seabee3_driver_base::MotorCntl & msg, int axis, int p_value)
{
	//ROS_INFO("updateMotorCntlMsg()");
	int value = p_value;
	//int value = abs(p_value) > 100 ? 100 * abs(p_value) / p_value : p_value;
	//if(axis == axis_heading)	
	//	ROS_INFO("value: %d", value);
	switch(axis)
	{
		case axis_speed:
			msg.motors[BeeStem3::MotorControllerIDs::FWD_RIGHT_THRUSTER] = -value;
			msg.motors[BeeStem3::MotorControllerIDs::FWD_LEFT_THRUSTER] = value;
			msg.mask[BeeStem3::MotorControllerIDs::FWD_RIGHT_THRUSTER] = 1;
			msg.mask[BeeStem3::MotorControllerIDs::FWD_LEFT_THRUSTER] = 1;
			break;
		case axis_strafe:
			msg.motors[BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER] += value;
			msg.motors[BeeStem3::MotorControllerIDs::STRAFE_BACK_THRUSTER] += value;
			if(msg.mask[BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER] == 1 && msg.mask[BeeStem3::MotorControllerIDs::STRAFE_BACK_THRUSTER] == 1)
			{
				msg.motors[BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER] /= 2;
				msg.motors[BeeStem3::MotorControllerIDs::STRAFE_BACK_THRUSTER] /= 2;
			}
			msg.mask[BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER] = 1;
			msg.mask[BeeStem3::MotorControllerIDs::STRAFE_BACK_THRUSTER] = 1;
			break;
		case axis_depth:
			msg.motors[BeeStem3::MotorControllerIDs::DEPTH_RIGHT_THRUSTER] += value;
			msg.motors[BeeStem3::MotorControllerIDs::DEPTH_LEFT_THRUSTER] += value;
			if(msg.mask[BeeStem3::MotorControllerIDs::DEPTH_RIGHT_THRUSTER] == 1 && msg.mask[BeeStem3::MotorControllerIDs::DEPTH_LEFT_THRUSTER] == 1)
			{
				msg.motors[BeeStem3::MotorControllerIDs::DEPTH_RIGHT_THRUSTER] /= 2;
				msg.motors[BeeStem3::MotorControllerIDs::DEPTH_LEFT_THRUSTER] /= 2;
			}
			msg.mask[BeeStem3::MotorControllerIDs::DEPTH_RIGHT_THRUSTER] = 1;
			msg.mask[BeeStem3::MotorControllerIDs::DEPTH_LEFT_THRUSTER] = 1;
			break;
		case axis_roll:
			msg.motors[BeeStem3::MotorControllerIDs::DEPTH_RIGHT_THRUSTER] += value;
			msg.motors[BeeStem3::MotorControllerIDs::DEPTH_LEFT_THRUSTER] += -value;
			if(msg.mask[BeeStem3::MotorControllerIDs::DEPTH_RIGHT_THRUSTER] == 1 && msg.mask[BeeStem3::MotorControllerIDs::DEPTH_LEFT_THRUSTER] == 1)
			{
				msg.motors[BeeStem3::MotorControllerIDs::DEPTH_RIGHT_THRUSTER] /= 2;
				msg.motors[BeeStem3::MotorControllerIDs::DEPTH_LEFT_THRUSTER] /= 2;
			}
			msg.mask[BeeStem3::MotorControllerIDs::DEPTH_RIGHT_THRUSTER] = 1;
			msg.mask[BeeStem3::MotorControllerIDs::DEPTH_LEFT_THRUSTER] = 1;
			break;
		case axis_heading:
			msg.motors[BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER] += -value;
			msg.motors[BeeStem3::MotorControllerIDs::STRAFE_BACK_THRUSTER] += value;
			if(msg.mask[BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER] == 1 && msg.mask[BeeStem3::MotorControllerIDs::STRAFE_BACK_THRUSTER] == 1)
			{
				msg.motors[BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER] /= 2;
				msg.motors[BeeStem3::MotorControllerIDs::STRAFE_BACK_THRUSTER] /= 2;
			}
			msg.mask[BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER] = 1;
			msg.mask[BeeStem3::MotorControllerIDs::STRAFE_BACK_THRUSTER] = 1;
			break;
	}
	
	/*for(int i = 0; i < BeeStem3::NUM_MOTOR_CONTROLLERS; i ++)
	{
		ROS_INFO("axis %d mask %d value %d", i, msg.mask[i], msg.motors[i]);
	}*/
}

void updateMotorCntlFromTwist(const geometry_msgs::TwistConstPtr & twist)
{
	//ROS_INFO("udpateMotorCntlFromTwist()");
	tf::Vector3 vel_desired_lin (twist->linear.x, twist->linear.y, twist->linear.z);
	tf::Vector3 vel_desired_ang (twist->angular.x, twist->angular.y, twist->angular.z);
	tf::Vector3 vel_diff_lin (vel_desired_lin - *vel_est_lin);
	tf::Vector3 vel_diff_ang (vel_desired_ang - *vel_est_ang);
	
	for(int i = 0; i < BeeStem3::NUM_MOTOR_CONTROLLERS; i ++)
	{
		motorCntlMsg->mask[i] = 0;
		motorCntlMsg->motors[i] = 0;
	}
	
	/*ROS_INFO("-----------------------------------------------------");
	
	ROS_INFO("linear x %f y %f z %f", vel_desired_lin.getX(), vel_desired_lin.getY(), vel_desired_lin.getZ());
	ROS_INFO("angular x %f y %f z %f", vel_desired_ang.getX(), vel_desired_ang.getY(), vel_desired_ang.getZ());*/
	
	updateMotorCntlMsg(*motorCntlMsg, axis_speed, vel_diff_lin.getX() * 100.0);
	updateMotorCntlMsg(*motorCntlMsg, axis_strafe, vel_diff_lin.getY() * 100.0);
	//updateMotorCntlMsg(*motorCntlMsg, axis_depth, vel_diff_lin.getZ() * 100.0);
	//updateMotorCntlMsg(*motorCntlMsg, axis_roll, vel_diff_ang.getX() * 100.0);
	//updateMotorCntlMsg(*motorCntlMsg, axis_heading, vel_diff_ang.getZ() * 100.0);

	if(*lastUpdateTime != ros::Time(-1))
	{
		//ros::Duration dt = ros::Time::now() - *lastHeadingUpdateTime;
		
		*desiredChangeInDepthPerSec = 5.0 * twist->linear.z;
		desiredChangeInRPYPerSec->x = 25.0 * twist->angular.x;
		desiredChangeInRPYPerSec->y = 25.0 * twist->angular.y;
		desiredChangeInRPYPerSec->z = 25.0 * twist->angular.z;

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
		ros::Duration dt = ros::Time::now() - *lastPidUpdateTime;
		
		desiredRPY->z -= desiredChangeInRPYPerSec->z * dt.toSec();

		Seabee3Util::normalizeAngle(desiredRPY->x);
		Seabee3Util::normalizeAngle(desiredRPY->y);
		Seabee3Util::normalizeAngle(desiredRPY->z);

		//actual - desired; 0 - 40 = -40;
		//desired -> actual; 40 -> 0 = 40 cw = -40
		//double headingError = Seabee3Util::angleDistRel(desiredRPY->z, IMUDataCache->ori.z);
		errorInDepth = desiredDepth - extlPressureCache->Value;
		errorInRPY->x = Seabee3Util::angleDistRel(desiredRPY->x, IMUDataCache->ori.x);
		errorInRPY->y = Seabee3Util::angleDistRel(desiredRPY->y, IMUDataCache->ori.y);
		errorInRPY->z = Seabee3Util::angleDistRel(desiredRPY->z, IMUDataCache->ori.z);
		
		Seabee3Util::capValue(*errorInDepth, *maxDepthError);
		Seabee3Util::capValue(errorInRPY->x, *maxRollError);
		Seabee3Util::capValue(errorInRPY->y, *maxPitchError);
		Seabee3Util::capValue(errorInRPY->z, *maxHeadingError);
		
		//headingError = abs(headingError) > *maxHeadingError ? *maxHeadingError * headingError / abs(headingError) : headingError;
		
		double depthMotorVal = 1.0 * pid_D->updatePid((double)(*errorInDepth), dt);
		double rollMotorVal = -1.0 * pid_R->updatePid(errorInRPY->x, dt);
		double pitchMotorVal = -1.0 * pid_P->updatePid(errorInRPY->y, dt);
		double headingMotorVal = -1.0 * pid_Y->updatePid(errorInRPY->z, dt);

		//ROS_INFO("initial motor val: %f", motorVal);
		
		Seabee3Util::capValue(rollMotorVal, 100.0);
		Seabee3Util::capValue(pitchMotorVal, 100.0);
		Seabee3Util::capValue(headingMotorVal, 100.0);
		
		//motorVal = abs(motorVal) > 100 ? 100 * motorVal / abs(motorVal) : motorVal;

		//ROS_INFO("heading: %f desired heading: %f motorValue: %f", IMUDataCache->ori.z, desiredRPY->z, motorVal);
		
		updateMotorCntlMsg(*motorCntlMsg, axis_depth, depthMotorVal);
		updateMotorCntlMsg(*motorCntlMsg, axis_roll, rollMotorVal);
		updateMotorCntlMsg(*motorCntlMsg, axis_pitch, pitchMotorVal);
		updateMotorCntlMsg(*motorCntlMsg, axis_heading, headingMotorVal);

		//ROS_INFO("heading error: %f motorVal: %f", headingError, motorVal);
	}
	*lastPidUpdateTime = ros::Time::now();
}

void IMUDataCallback(const xsens_node::IMUDataConstPtr & data)
{
	//ROS_INFO("IMUDataCallback()");
	*IMUDataCache = *data;
	while(IMUDataCache->ori.z > 360)
		IMUDataCache->ori.z -= 360;
	while(IMUDataCache->ori.z < 0)
		IMUDataCache->ori.z += 360;
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
}

bool setDesiredDepthCallback(seabee3_driver::SetDesiredDepth::Request & req, seabee3_driver::SetDesiredDepth::Response & resp)
{
	if(req.Mask > 0)
		*desiredDepth = req.DesiredDepth + (req.Mode == 1 ? *desiredDepth : 0);
		
	resp.CurrentDesiredDepth = *desiredDepth;
	resp.ErrorInDepth = *errorInDepth;
	return true;
}

bool setDesiredRPYCallback(seabee3_driver::SetDesiredRPY::Request & req, seabee3_driver::SetDesiredRPY::Response & resp)
{
	if(req.Mask.x > 0.0f)
		desiredRPY->x = (req.Mode.x == 1.0f ? desiredRPY->x : 0);
			
	if(req.Mask.y > 0.0f)
		desiredRPY->y = (req.Mode.y == 1.0f ? desiredRPY->y : 0);
			
	if(req.Mask.z > 0.0f)
		desiredRPY->z = (req.Mode.z == 1.0f ? desiredRPY->z : 0);
		
	resp.CurrentDesiredRPY = *desiredRPY;
	resp.ErrorInRPY = *errorInRPY;
	return true;
}

void ExtlPressureCallback(const seabee3_driver_base::PressureConstPtr & extlPressure)
{
	*extlPressureCache = *extlPressure;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "seabee3_driver");
	ros::NodeHandle n;
	
	vel_est_lin = new tf::Vector3(0, 0, 0);
	vel_est_ang = new tf::Vector3(0, 0, 0);
	velocity_sample_start = new ros::Time(-1);
	
	IMUDataCache = new xsens_node::IMUData;
	TwistCache = new geometry_msgs::Twist;
	
	ros::Subscriber imu_sub = n.subscribe("IMUData", 1, IMUDataCallback);
	ros::Subscriber cmd_vel_sub = n.subscribe("seabee3/cmd_vel", 1, CmdVelCallback);
	ros::Subscriber extl_depth_sub = n.subscribe("seabee3/ExtllPressure", 1, ExtlPressureCallback);
	
	desiredRPY = new geometry_msgs::Vector3; desiredRPY->x = 0; desiredRPY->y = 0; desiredRPY->z = 0;
	errorInRPY = new geometry_msgs::Vector3; errorInRPY->x = 0; errorInRPY->y = 0; errorInRPY->z = 0;
	desiredChangeInRPYPerSec = new geometry_msgs::Vector3; desiredChangeInRPYPerSec->x = 0; desiredChangeInRPYPerSec->y = 0; desiredChangeInRPYPerSec->z = 0;
	desiredDepth = new int(0);
	errorInDepth = new int(0);
	desiredChangeInDepthPerSec = new int(0);
	maxDepthError = new int(0);
	maxRollError = new double(0.0);
	maxPitchError = new double(0.0);
	maxHeadingError = new double(0.0);
	lastUpdateTime = new ros::Time(ros::Time(-1));
	lastPidUpdateTime = new ros::Time(ros::Time(-1));
	
	pid_D = new control_toolbox::Pid;
	pid_R = new control_toolbox::Pid;
	pid_P = new control_toolbox::Pid;
	pid_Y = new control_toolbox::Pid;
	
	double pid_i_min, pid_i_max;
	double pid_D_p, pid_D_i, pid_D_d;
	double pid_R_p, pid_R_i, pid_R_d;
	double pid_P_p, pid_P_i, pid_P_d;
	double pid_Y_p, pid_Y_i, pid_Y_d;
	
	n.param("depth_err_cap_thresh", *maxDepthError, 25);
	n.param("roll_err_cap_thresh", *maxRollError, 25.0);
	n.param("pitch_err_cap_thresh", *maxPitchError, 25.0);
	n.param("heading_err_cap_thresh", *maxHeadingError, 25.0);
	
	n.param("pid_D_p", pid_D_p, 10.0);
	n.param("pid_D_i", pid_D_i, 0.2);
	n.param("pid_D_d", pid_D_d, 0.2);
	
	n.param("pid_R_p", pid_R_p, 10.0);
	n.param("pid_R_i", pid_R_i, 0.2);
	n.param("pid_R_d", pid_R_d, 0.2);
	
	n.param("pid_P_p", pid_P_p, 10.0);
	n.param("pid_P_i", pid_P_i, 0.2);
	n.param("pid_P_d", pid_P_d, 0.2);
	
	n.param("pid_Y_p", pid_Y_p, 10.0);
	n.param("pid_Y_i", pid_Y_i, 0.2);
	n.param("pid_Y_d", pid_Y_d, 0.2);
	
	n.param("pid_i_max", pid_i_max, 1.0);
	n.param("pid_i_min", pid_i_min, -1.0);
	
	pid_D->initPid(pid_D_p, pid_D_i, pid_D_d, pid_i_max, pid_i_min);
	pid_D->reset();
	pid_R->initPid(pid_R_p, pid_R_i, pid_R_d, pid_i_max, pid_i_min);
	pid_R->reset();
	pid_P->initPid(pid_P_p, pid_P_i, pid_P_d, pid_i_max, pid_i_min);
	pid_P->reset();
	pid_Y->initPid(pid_Y_p, pid_Y_i, pid_Y_d, pid_i_max, pid_i_min);
	pid_Y->reset();
	
	motorCntlMsg = new seabee3_driver_base::MotorCntl;
	
	for(int i = 0; i < BeeStem3::NUM_MOTOR_CONTROLLERS; i ++)
	{
		motorCntlMsg->motors[i] = 0;
		motorCntlMsg->mask[i] = 0;
	}
	
	ros::Publisher motor_cntl_pub = n.advertise<seabee3_driver_base::MotorCntl>("seabee3/MotorCntl", 1);
	ros::ServiceServer setDesiredDepth_srv = n.advertiseService("seabee3/setDesiredDepth", setDesiredDepthCallback);
	ros::ServiceServer setDesiredRPY_srv = n.advertiseService("seabee3/setDesiredRPY", setDesiredRPYCallback);
	
	while(ros::ok())
	{
		headingPidStep();
		motor_cntl_pub.publish(*motorCntlMsg);
		ros::spinOnce();
		ros::Rate(20).sleep();
	}
	
	ros::spin();
	return 0;
}
