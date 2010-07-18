#include <ros/ros.h>
//#include <seabee3_driver/SetDesiredRPY.h>
#include <seabee3_driver/SetDesiredXYZ.h>
#include <geometry_msgs/Twist.h>
#include <seabee3_driver_base/FiringDeviceAction.h>
#include <seabee3_driver_base/KillSwitch.h>

bool killSwitchEnabled = true; //killed

void KillSwitchCallback(const seabee3_driver_base::KillSwitchConstPtr & killSwitch)
{
	killSwitchEnabled = killSwitch->Value == 1 ? true : false;
}

int main(int argc, char * argv[])
{
	ros::init(argc, argv, "ultra_basic_fsm");
	ros::NodeHandle n("~");
	
	int sleep1, dive_sleep, dive_depth, forward_speed, shooter_delay;
	
	n.param("sleep1", sleep1, 5);
	n.param("dive_sleep", dive_sleep, 5);
	n.param("shooter_delay", shooter_delay, 15);
	
	n.param("dive_depth", dive_depth, 950);
	n.param("forward_speed", forward_speed, 80);
	
	ros::ServiceClient depth_srv = n.serviceClient<seabee3_driver::SetDesiredXYZ>("/seabee3/setDesiredXYZ");
	ros::ServiceClient shooter_srv = n.serviceClient<seabee3_driver_base::FiringDeviceAction>("/seabee3/ShooterAction");
	ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/seabee3/cmd_vel", 1);
	ros::Subscriber kill_switch_sub = n.subscribe("/seabee3/kill_switch", 1, KillSwitchCallback);
	seabee3_driver_base::FiringDeviceAction fireShooter;
	
	seabee3_driver::SetDesiredXYZ setDesiredXYZ;
	setDesiredXYZ.request.Mask.z = 1;
	setDesiredXYZ.request.DesiredXYZ.z = dive_depth;
	
	geometry_msgs::Twist cmd_vel;
	
	int mState = -1;
	ros::Time lastTime = ros::Time::now();
	
	while( ros::ok() )
	{
		ros::Duration dt = ros::Time::now() - lastTime;
		
		if(killSwitchEnabled)
		{
			mState = -1;
			cmd_vel = geometry_msgs::Twist();
			cmd_vel_pub.publish(cmd_vel);
		}
		
		if(mState == -1)
		{
			if(!killSwitchEnabled)
			{
				mState = 0;
				lastTime = ros::Time::now();
			}
		}
		else if(mState == 0)
		{
			if(dt.toSec() > sleep1)
			{
				mState = 1;
				lastTime = ros::Time::now();
				//set depth
				depth_srv.call(setDesiredXYZ);
				cmd_vel.linear.x = forward_speed;
			}
			else
			{
				cmd_vel_pub.publish(cmd_vel);
			}
		}
		else if(mState == 1)
		{
			if(dt.toSec() > dive_sleep)
			{
				lastTime = ros::Time::now();
				mState = 2;
			}
		}
		else if(mState == 2)
		{
			//drive forward
			cmd_vel_pub.publish(cmd_vel);
			if(dt.toSec() > shooter_delay)
			{
				lastTime = ros::Time::now();
				mState = 3;
				shooter_srv.call(fireShooter);
			}
		}
		else if(mState == 3)
		{
			cmd_vel_pub.publish(cmd_vel);
		}
		
		ros::spinOnce();
		ros::Rate(15).sleep();
	}
}
