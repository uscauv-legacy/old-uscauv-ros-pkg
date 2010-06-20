#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <xsens_node/IMUData.h>
#include <seabee3_driver_base/MotorCntl.h>
#include <geometry_msgs/Twist.h>

#include <queue>

#include "libseabee3/BeeStem3.h"

std::queue<xsens_node::IMUData> * IMUDataCache;
std::queue<geometry_msgs::Twist> * TwistCache;
ros::Time * velocity_sample_start;
seabee3_driver_base::MotorCntl * motorCntlMsg;

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
	ROS_INFO("axis %d", axis);
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
		case axis_heading:
			msg.motors[BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER] += -value;
			msg.motors[BeeStem3::MotorControllerIDs::STRAFE_BACK_THRUSTER] += value;
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
	updateMotorCntlMsg(*motorCntlMsg, axis_strafe, vel_diff_lin.getY() * 100.0);
	updateMotorCntlMsg(*motorCntlMsg, axis_depth, vel_diff_lin.getZ() * 100.0);
	updateMotorCntlMsg(*motorCntlMsg, axis_roll, vel_diff_ang.getX() * 100.0);
	updateMotorCntlMsg(*motorCntlMsg, axis_heading, vel_diff_ang.getZ() * 100.0);
}

void IMUDataCallback(const xsens_node::IMUDataConstPtr & data)
{
	//ROS_INFO("IMUDataCallback()");
	IMUDataCache->push(*data);
	while(IMUDataCache->size() > 5)
	{
		IMUDataCache->pop();
	}
}

void CmdVelCallback(const geometry_msgs::TwistConstPtr & twist)
{
	//ROS_INFO("CmdVelCallback()");
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
	ros::Subscriber cmd_vel_sub = n.subscribe("seabee3/cmd_vel", 100, CmdVelCallback);
	
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
	ros::Publisher motor_cntl_pub = n.advertise<seabee3_driver_base::MotorCntl>("MotorCntl", 1);
	
	while(ros::ok())
	{
		motor_cntl_pub.publish(*motorCntlMsg);
		ros::spinOnce();
		ros::Rate(20).sleep();
	}
	
	ros::spin();
	return 0;
}
