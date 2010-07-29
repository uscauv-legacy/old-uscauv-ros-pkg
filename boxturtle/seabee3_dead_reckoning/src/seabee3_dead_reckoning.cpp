/*
Code taken from: http://www.ros.org/wiki/navigation/Tutorials/RobotSetup/Odom
*/


#include <ros/ros.h>
#include <seabee3_driver_base/MotorCntl.h>
#include <seabee3_driver_base/Depth.h>
#include <tf/transform_broadcaster.h>
#include <xsens_node/IMUData.h>
#include <seabee3_dead_reckoning/ResetOdom.h>

//This is terrible, and needs to be put somewhere more logical and global
#define FWD_RIGHT_THRUSTER 	   3
#define FWD_LEFT_THRUSTER 	   1
#define DEPTH_RIGHT_THRUSTER 	 4
#define DEPTH_LEFT_THRUSTER 	 2
#define STRAFE_FRONT_THRUSTER	 0
#define STRAFE_BACK_THRUSTER 	 5
#define SHOOTER 	             6
#define DROPPER_STAGE1 	       7
#define DROPPER_STAGE2 	       8

#define POS_TRANSLATE_COEFF   0.004
#define NEG_TRANSLATE_COEFF   0.001
#define POS_STRAFE_COEFF      0.002
#define NEG_STRAFE_COEFF      0.002 

geometry_msgs::Vector3 ori, linear;
double depth;
double x, y, z;
tf::Transform odom_trans;

float getSimpleSpeed(int pwr, int translate)
{
	if(pwr > 0.0 && translate)
		return pwr * POS_TRANSLATE_COEFF;
	else if(pwr <= 0.0 && translate)
		return pwr * NEG_TRANSLATE_COEFF;
	else if(pwr > 0.0 && !translate)
		return pwr * POS_STRAFE_COEFF;
	else if(pwr <= 0.0 && !translate)
		return pwr * NEG_STRAFE_COEFF;
	else
		return 0.0;
}

tf::TransformBroadcaster * odom_broadcaster;

void imuCallback(const xsens_node::IMUDataConstPtr & msg)
{
	ori = msg->ori;
}

//######################################################################
void depthCallback(const seabee3_driver_base::DepthConstPtr & msg)
{
  depth = msg->Value;
  ROS_INFO("Got depth: %f",depth);
}

//######################################################################
void cmdVelocityCallback(const geometry_msgs::TwistConstPtr & msg)
{
	linear = msg->linear;
}

//######################################################################
void motorCntlCallback(const seabee3_driver_base::MotorCntlConstPtr & msg)
{
	ROS_INFO("%d %d %d %d %d %d %d %d %d", msg->motors[0], msg->motors[1], msg->motors[2], msg->motors[3], msg->motors[4], msg->motors[5], msg->motors[6], msg->motors[7], msg->motors[8] );
	//ROS_INFO("motorCntlCallback");
	//Create some persistent variables for this callback
	static double th           =  0.0;
	static double last_depth   =  0.0;
	static double last_th      =  0.0;
	static ros::Time last_time =  ros::Time::now();
	ros::Time current_time     =  ros::Time::now();

	//Find the amount of elapsed time
	double dt = (current_time - last_time).toSec();
	ROS_INFO("dt %f", dt);

	//Get the orientation from the imu 
	th = ori.z * M_PI/180.0;

	//Get the average forward speed
	int right_thrust_speed = msg->motors[FWD_RIGHT_THRUSTER];
	int left_thrust_speed = msg->motors[FWD_LEFT_THRUSTER];

	double trans_speed = getSimpleSpeed( right_thrust_speed, 1);

	//Compute our predicted velocity from the motor commands
	double vx  = cos(th) * trans_speed;
	double vy  = sin(th) * trans_speed;

	// if we are strafing, add that component to our velocities
	ROS_INFO("lin y %f", linear.y);
	if(linear.y != 0.0)
	{
		int front_strafe_speed = msg->motors[STRAFE_FRONT_THRUSTER];
		int back_strafe_speed = msg->motors[STRAFE_BACK_THRUSTER];

		double strafe_speed = getSimpleSpeed( back_strafe_speed, 0);
		ROS_INFO("strafe speed %f; %d %d", strafe_speed, front_strafe_speed, back_strafe_speed);
		vx += cos(th+(M_PI/2)) * strafe_speed;
		vy += sin(th+(M_PI/2)) * strafe_speed;
	}

	double vth = (th - last_th) / dt;
	double vz  = (depth - last_depth)/dt;

	//compute the change in planar position from the speed and the elapsed time
	double delta_x = vx * dt;
	double delta_y = vy * dt;

	//Integrate!
	x  += delta_x;
	y  += delta_y;
	z  = depth;

	odom_trans = tf::Transform( tf::Quaternion( ori.z, ori.y, ori.x), tf::Vector3( x, y, z) );

	//Update our last_* variables
	last_time  = current_time;
	last_depth = depth;
	last_th = th;
}

bool ResetOdomCallback( seabee3_dead_reckoning::ResetOdom::Request & req, seabee3_dead_reckoning::ResetOdom::Response & resp)
{
	x = req.Pos.x;
	y = req.Pos.y;
	z = req.Pos.z; //this is just depth
	odom_trans = tf::Transform( tf::Quaternion( ori.z, ori.y, ori.x), tf::Vector3( x, y, z) );
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "seabee3_dead_reckoning");
	ros::NodeHandle n("~");

	ros::Subscriber motor_cntl_sub = n.subscribe ("/seabee3/motor_cntl", 1, motorCntlCallback);

	ros::Subscriber imu_sub = n.subscribe ("/xsens/data_calibrated", 1, imuCallback);

	ros::Subscriber depth_sub = n.subscribe ("/seabee3/depth", 1, depthCallback);

	ros::Subscriber cmd_vel_sub = n.subscribe ("/seabee3/cmd_vel", 1, cmdVelocityCallback);

	ros::ServiceServer reset_odom = n.advertiseService ("/seabee3/resetOdom", ResetOdomCallback);

	odom_broadcaster = new tf::TransformBroadcaster;
		
	odom_trans = tf::Transform( tf::Quaternion( ori.z, ori.y, ori.x), tf::Vector3( x, y, z) );

	while( ros::ok() )
	{
		odom_broadcaster->sendTransform( tf::StampedTransform( odom_trans, ros::Time::now(), "/seabee3/odom", "/seabee3/base_link" ) );
		ros::spinOnce();
		ros::Rate(20).sleep();
	}

	return 0;
}
