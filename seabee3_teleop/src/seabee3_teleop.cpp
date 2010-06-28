#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <joy/Joy.h>

ros::Publisher * cmd_vel_pub;
int speed, strafe, surface, dive, heading, roll;
double speed_s, strafe_s, surface_s, dive_s, heading_s, roll_s;

double applyDeadZone (double value)
{
	return fabs(value) < 0.15 ? 0.0 : value;
}

void joyCallback(const joy::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist cmd_vel;
	
	double joy_speed = 		applyDeadZone( (double)(joy->axes[speed]) );
	double joy_strafe = 	applyDeadZone( (double)(joy->axes[strafe]) );
	double joy_dive = 		applyDeadZone( (double)(joy->axes[dive]) );
	double joy_surface = 	applyDeadZone( (double)(joy->axes[surface]) );
	double joy_heading = 	applyDeadZone( (double)(joy->axes[heading]) );
	double joy_roll = 		applyDeadZone( (double)(joy->axes[roll]) );
	
	cmd_vel.linear.x = speed_s * joy_speed; //speed
	cmd_vel.linear.y = strafe_s * joy_strafe; //strafe
	cmd_vel.linear.z = 0.5 * (float)(dive_s * joy_dive - surface_s * joy_surface); //dive - surface
	
	cmd_vel.angular.x = roll_s * joy_roll; //roll
	cmd_vel.angular.y = 0;
	cmd_vel.angular.z = heading_s * joy_heading; //heading
	
	cmd_vel_pub->publish(cmd_vel);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "seabee3_teleop");
	ros::NodeHandle n;
	
	n.param("speed", speed, 0);
	n.param("strafe", strafe, 1);
	n.param("surface", surface, 2);
	n.param("dive", dive, 5);
	n.param("heading", heading, 4);
	n.param("roll", roll, 3);
	
	n.param("speed_scale", speed_s, 1.0);
	n.param("strafe_scale", strafe_s, 1.0);
	n.param("surface_scale", surface_s, 1.0);
	n.param("dive_scale", dive_s, 1.0);
	n.param("heading_scale", heading_s, 1.0);
	n.param("roll_scale", roll_s, 1.0);
	
	ros::Subscriber joy_sub = n.subscribe("joy", 1, joyCallback);
	
	cmd_vel_pub = new ros::Publisher;
	*cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/seabee3/cmd_vel", 1);
	
	ros::spin();
	return 0;
}


