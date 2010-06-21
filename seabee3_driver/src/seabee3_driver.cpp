#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <control_toolbox/pid.h>

#include <xsens_node/IMUData.h>
#include <seabee3_driver_base/MotorCntl.h>
#include <geometry_msgs/Twist.h>

#include <seabee3_beestem/BeeStem3.h>

#include <seabee3_util/seabee3_util.h>

xsens_node::IMUData * IMUDataCache;
geometry_msgs::Twist * TwistCache;
ros::Time * velocity_sample_start;
seabee3_driver_base::MotorCntl * motorCntlMsg;
double * desiredHeading, *maxHeadingError, *desiredChangeInHeadingPerSec;
ros::Time * lastHeadingUpdateTime;
ros::Time * lastHeadingPidUpdateTime;

control_toolbox::Pid * pid_controller;

tf::Vector3 * vel_est_lin;
tf::Vector3 * vel_est_ang;

#define axis_speed 0
#define axis_strafe 1
#define axis_depth 2
#define axis_roll 3
#define axis_heading 4

void updateMotorCntlMsg(seabee3_driver_base::MotorCntl & msg, int axis, int p_value)
{
	//ROS_INFO("updateMotorCntlMsg()");
	int value = p_value;
	//int value = abs(p_value) > 100 ? 100 * abs(p_value) / p_value : p_value;
	if(axis == axis_heading)	
		ROS_INFO("value: %d", value);
	switch(axis)
	{
		case axis_speed:
			msg.motors[BeeStem3::MotorControllerIDs::FWD_RIGHT_THRUSTER] = -value;
			msg.motors[BeeStem3::MotorControllerIDs::FWD_LEFT_THRUSTER] = value;
			msg.mask[BeeStem3::MotorControllerIDs::FWD_RIGHT_THRUSTER] = 1;
			msg.mask[BeeStem3::MotorControllerIDs::FWD_LEFT_THRUSTER] = 1;
			break;
		case axis_strafe:
			msg.motors[BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER] = value;
			msg.motors[BeeStem3::MotorControllerIDs::STRAFE_BACK_THRUSTER] = value;
			msg.mask[BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER] = 1;
			msg.mask[BeeStem3::MotorControllerIDs::STRAFE_BACK_THRUSTER] = 1;
			break;
		case axis_depth:
			msg.motors[BeeStem3::MotorControllerIDs::DEPTH_RIGHT_THRUSTER] = value;
			msg.motors[BeeStem3::MotorControllerIDs::DEPTH_LEFT_THRUSTER] = value;
			msg.mask[BeeStem3::MotorControllerIDs::DEPTH_RIGHT_THRUSTER] = 1;
			msg.mask[BeeStem3::MotorControllerIDs::DEPTH_LEFT_THRUSTER] = 1;
			break;
		/*case axis_roll:
			msg.motors[BeeStem3::MotorControllerIDs::DEPTH_RIGHT_THRUSTER] = value;
			msg.motors[BeeStem3::MotorControllerIDs::DEPTH_LEFT_THRUSTER] = -value;
			msg.mask[BeeStem3::MotorControllerIDs::DEPTH_RIGHT_THRUSTER] = 1;
			msg.mask[BeeStem3::MotorControllerIDs::DEPTH_LEFT_THRUSTER] = 1;
			break;*/
		//!combine heading and axis_strafe later
		case axis_heading:
			msg.motors[BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER] = -value;
			msg.motors[BeeStem3::MotorControllerIDs::STRAFE_BACK_THRUSTER] = value;
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
	
	/*for(int i = 0; i < BeeStem3::NUM_MOTOR_CONTROLLERS; i ++)
	{
		motorCntlMsg->mask[i] = 0;
		motorCntlMsg->motors[i] = 0;
	}*/
	
	/*ROS_INFO("-----------------------------------------------------");
	
	ROS_INFO("linear x %f y %f z %f", vel_desired_lin.getX(), vel_desired_lin.getY(), vel_desired_lin.getZ());
	ROS_INFO("angular x %f y %f z %f", vel_desired_ang.getX(), vel_desired_ang.getY(), vel_desired_ang.getZ());*/
	
	updateMotorCntlMsg(*motorCntlMsg, axis_speed, vel_diff_lin.getX() * 100.0);
	//updateMotorCntlMsg(*motorCntlMsg, axis_strafe, vel_diff_lin.getY() * 100.0);
	updateMotorCntlMsg(*motorCntlMsg, axis_depth, vel_diff_lin.getZ() * 100.0);
	updateMotorCntlMsg(*motorCntlMsg, axis_roll, vel_diff_ang.getX() * 100.0);
	//updateMotorCntlMsg(*motorCntlMsg, axis_heading, vel_diff_ang.getZ() * 100.0);

	if(*lastHeadingUpdateTime != ros::Time(-1))
	{
		ros::Duration dt = ros::Time::now() - *lastHeadingUpdateTime;

		*desiredChangeInHeadingPerSec = 100.0 * twist->angular.z;

		//ROS_INFO("heading: %f desired heading: %f dt: %f", IMUDataCache->ori.z, *desiredHeading, dt.toSec());
		
//		if(!IMUDataCache->empty())
//			ROS_INFO("desired heading: %f actualHeading: %f dt: %f", *desiredHeading, IMUDataCache->front().ori.z, dt.toSec());
	}
	
	*lastHeadingUpdateTime = ros::Time::now();
}

void headingPidStep()
{
	if(*lastHeadingPidUpdateTime != ros::Time(-1))
	{
		ros::Duration dt = ros::Time::now() - *lastHeadingPidUpdateTime;
		
		*desiredHeading -= *desiredChangeInHeadingPerSec * dt.toSec();

		Seabee3Util::normalizeAngle(*desiredHeading);

		//actual - desired; 0 - 40 = -40;
		//desired -> actual; 40 -> 0 = 40 cw = -40
		double headingError = Seabee3Util::angleDistRel(*desiredHeading, IMUDataCache->ori.z);
		
		headingError = abs(headingError) > *maxHeadingError ? *maxHeadingError * headingError / abs(headingError) : headingError;
		
		double motorVal = -1.0 * pid_controller->updatePid(headingError, dt);

		//ROS_INFO("initial motor val: %f", motorVal);
		
		motorVal = abs(motorVal) > 100 ? 100 * motorVal / abs(motorVal) : motorVal;

		ROS_INFO("heading: %f desired heading: %f motorValue: %f", IMUDataCache->ori.z, *desiredHeading, motorVal);
		
		updateMotorCntlMsg(*motorCntlMsg, axis_heading, motorVal);

		//ROS_INFO("heading error: %f motorVal: %f", headingError, motorVal);
	}
	*lastHeadingPidUpdateTime = ros::Time::now();
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
	
	desiredHeading = new double(0.0);
	desiredChangeInHeadingPerSec = new double(0.0);
	maxHeadingError = new double(0.0);
	lastHeadingUpdateTime = new ros::Time(ros::Time(-1));
	lastHeadingPidUpdateTime = new ros::Time(ros::Time(-1));
	
	pid_controller = new control_toolbox::Pid;
	
	double pid_p, pid_i, pid_i_min, pid_i_max, pid_d;
	
	n.param("heading_err_cap_thresh", *maxHeadingError, 25.0);
	n.param("pid_p", pid_p, 10.0);
	n.param("pid_i", pid_i, 0.2);
	n.param("pid_d", pid_d, 0.2);
	n.param("pid_i_max", pid_i_max, 1.0);
	n.param("pid_i_min", pid_i_min, -1.0);
	
	pid_controller->initPid(pid_p, pid_i, pid_d, pid_i_max, pid_i_min);
	
	pid_controller->reset();
	
	motorCntlMsg = new seabee3_driver_base::MotorCntl;
	motorCntlMsg->motors[0] = 0;
	motorCntlMsg->motors[1] = 0;
	motorCntlMsg->motors[2] = 0;
	motorCntlMsg->motors[3] = 0;
	motorCntlMsg->motors[4] = 0;
	motorCntlMsg->motors[5] = 0;
	motorCntlMsg->motors[5] = 0;
	motorCntlMsg->motors[7] = 0;
	motorCntlMsg->motors[8] = 0;
	
	motorCntlMsg->mask[0] = 0;
	motorCntlMsg->mask[1] = 0;
	motorCntlMsg->mask[2] = 0;
	motorCntlMsg->mask[3] = 0;
	motorCntlMsg->mask[4] = 0;
	motorCntlMsg->mask[5] = 0;
	motorCntlMsg->mask[6] = 0;
	motorCntlMsg->mask[7] = 0;
	motorCntlMsg->mask[8] = 0;
	
	//motor_cntl_pub = new ros::Publisher;
	ros::Publisher motor_cntl_pub = n.advertise<seabee3_driver_base::MotorCntl>("seabee3/MotorCntl", 1);
	
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
