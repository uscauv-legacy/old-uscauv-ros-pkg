#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <xsens_node/IMUData.h>
#include <seabee3_driver_base/MotorCntl.h>
#include <geometry_msgs/Twist.h>

#include <queue>

#include "libseabee3/BeeStem3.h"

ros::Publisher * motor_cntl_pub;
std::queue<xsens_node::IMUData> * IMUDataCache;
std::queue<geometry_msgs::Twist> * TwistCache;
ros::Time * velocity_sample_start;

tf::Vector3 * vel_est_lin;
tf::Vector3 * vel_est_ang;

namespace Axes
{
	const static int speed = 0;
	const static int strafe = 1;
	const static int depth = 2;
	const static int roll = 3;
	const static int heading = 4;
}

void updateMotorCntlMsg(seabee3_driver_base::MotorCntl & msg, const int & axis, const int & p_value)
{
	int value = abs(p_value) > 100 ? 100 * abs(p_value) / p_value : p_value;
	for(int i = 0; i < BeeStem3::NUM_MOTOR_CONTROLLERS; i ++)
	{
		msg.mask[i] = 0;
		msg.motors[i] = 0;
	}
	switch(axis)
	{
		case Axes::speed:
			msg.motors[BeeStem3::MotorControllerIDs::FWD_RIGHT_THRUSTER] = value;
			msg.motors[BeeStem3::MotorControllerIDs::FWD_LEFT_THRUSTER] = value;
			msg.mask[BeeStem3::MotorControllerIDs::FWD_RIGHT_THRUSTER] = 1;
			msg.mask[BeeStem3::MotorControllerIDs::FWD_LEFT_THRUSTER] = 1;
			break;
		case Axes::strafe:
			msg.motors[BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER] = -value;
			msg.motors[BeeStem3::MotorControllerIDs::STRAFE_BACK_THRUSTER] = value;
			msg.mask[BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER] = 1;
			msg.mask[BeeStem3::MotorControllerIDs::STRAFE_BACK_THRUSTER] = 1;
			break;
		case Axes::depth:
			msg.motors[BeeStem3::MotorControllerIDs::DEPTH_RIGHT_THRUSTER] = value;
			msg.motors[BeeStem3::MotorControllerIDs::DEPTH_LEFT_THRUSTER] = value;
			msg.mask[BeeStem3::MotorControllerIDs::DEPTH_RIGHT_THRUSTER] = 1;
			msg.mask[BeeStem3::MotorControllerIDs::DEPTH_LEFT_THRUSTER] = 1;
			break;
		case Axes::roll:
			msg.motors[BeeStem3::MotorControllerIDs::DEPTH_RIGHT_THRUSTER] = value;
			msg.motors[BeeStem3::MotorControllerIDs::DEPTH_LEFT_THRUSTER] = -value;
			msg.mask[BeeStem3::MotorControllerIDs::DEPTH_RIGHT_THRUSTER] = 1;
			msg.mask[BeeStem3::MotorControllerIDs::DEPTH_LEFT_THRUSTER] = 1;
			break;
		case Axes::heading:
			msg.motors[BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER] = -value;
			msg.motors[BeeStem3::MotorControllerIDs::STRAFE_BACK_THRUSTER] = value;
			msg.mask[BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER] = 1;
			msg.mask[BeeStem3::MotorControllerIDs::STRAFE_BACK_THRUSTER] = 1;
			break;
	}
}

void updateMotorCntlFromTwist(const geometry_msgs::TwistConstPtr & twist)
{
	tf::Vector3 vel_desired_lin (twist->linear.x, twist->linear.y, twist->linear.z);
	tf::Vector3 vel_desired_ang (twist->angular.x, twist->angular.y, twist->angular.z);
	//tf::Vector3 vel_diff_lin (vel_desired_lin - *vel_est_lin);
	//tf::Vector3 vel_diff_ang (vel_desired_ang - *vel_est_ang);
	
	seabee3_driver_base::MotorCntl motorCntlMsg;
	updateMotorCntlMsg(motorCntlMsg, Axes::speed, vel_desired_lin.getX() * 50);
	updateMotorCntlMsg(motorCntlMsg, Axes::strafe, vel_desired_lin.getY() * 50);
	updateMotorCntlMsg(motorCntlMsg, Axes::depth, vel_desired_lin.getZ() * 50);
	updateMotorCntlMsg(motorCntlMsg, Axes::roll, vel_desired_ang.getX() * 25);
	updateMotorCntlMsg(motorCntlMsg, Axes::heading, vel_desired_ang.getX() * 25);
}

void IMUDataCallback(const xsens_node::IMUDataConstPtr & data)
{
	IMUDataCache->push(*data);
	while(IMUDataCache->size() > 5)
	{
		IMUDataCache->pop();
	}
}

void CmdVelCallback(const geometry_msgs::TwistConstPtr & twist)
{
	TwistCache->push(*twist);
	while(TwistCache->size() > 5)
	{
		TwistCache->pop();
	}
	
	updateMotorCntlFromTwist(twist);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "seabee3_driver_imu");
	ros::NodeHandle n;
	
	vel_est_lin = new tf::Vector3(0, 0, 0);
	vel_est_ang = new tf::Vector3(0, 0, 0);
	velocity_sample_start = new ros::Time(-1);
	
	IMUDataCache = new std::queue<xsens_node::IMUData>;
	TwistCache = new std::queue<geometry_msgs::Twist>;
	
	ros::Subscriber imu_sub = n.subscribe("IMUData", 100, IMUDataCallback);
	ros::Subscriber cmd_vel_sub = n.subscribe("/seabee3/cmd_vel", 100, CmdVelCallback);
	
	motor_cntl_pub = new ros::Publisher;
	*motor_cntl_pub = n.advertise<seabee3_driver_base::MotorCntl>("MotorCntl", 1);
	
	/*while(ros::ok()
	{
		updateTwist()
		ros::spinOnce();
		ros::Rate(20).sleep();
	}*/
	
	ros::spin();
	return 0;
}
