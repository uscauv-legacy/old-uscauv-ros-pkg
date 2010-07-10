#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <seabee3_driver/SetDesiredRPY.h>
#include <seabee3_driver/SetDesiredXYZ.h>
#include <landmark_map/LandmarkMap.h>
#include <localization_defs/LandmarkMapMsg.h>
#include <color_segmenter/FindBlobs.h>
#include <string>
#include <tf/transform_listener.h>
#include <control_toolbox/pid.h>

std::string robot_frame, map_frame;

struct PIDConfig
{
	double p;
	double i;
	double d;
};

control_toolbox::Pid * pid_X, * pid_Y;

geometry_msgs::Twist updateCmdVel()
{
	geometry_msgs::Twist cmd_vel;
	return cmd_vel;
}

int main( int argc, char * argv[] )
{
	ros::init(argc, argv, "waypoint_controller");
	ros::NodeHandle n("~");
	
	n.param("robot_frame", robot_frame, std::string("/base_link") );
	n.param("map_frame", map_frame, std::string("/landmark_map") );
	
	ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	
	ros::ServiceClient set_desired_rpy_srv = n.serviceClient<seabee3_driver::SetDesiredRPY>("setRPY");
	
	ros::ServiceClient set_desired_xyz_srv = n.serviceClient<seabee3_driver::SetDesiredXYZ>("setXYZ");
	
	pid_X = new control_toolbox::Pid;
	pid_Y = new control_toolbox::Pid;
	
	PIDConfig pid_X_cfg, pid_Y_cfg;
	
	double pid_i_min, pid_i_max;
	
	n.param("pid/X/p", pid_X_cfg.p, 2.5);
	n.param("pid/X/i", pid_X_cfg.i, 0.05);
	n.param("pid/X/d", pid_X_cfg.d, 0.2);
	
	n.param("pid/Y/p", pid_Y_cfg.p, 2.5);
	n.param("pid/Y/i", pid_Y_cfg.i, 0.05);
	n.param("pid/Y/d", pid_Y_cfg.d, 0.2);
	
	n.param("pid/i_max", pid_i_max, 1.0);
	n.param("pid/i_min", pid_i_min, -1.0);
	
	pid_X->initPid(pid_X_cfg.p, pid_X_cfg.i, pid_X_cfg.d, pid_i_max, pid_i_min);
	pid_X->reset();
	pid_Y->initPid(pid_Y_cfg.p, pid_Y_cfg.i, pid_Y_cfg.d, pid_i_max, pid_i_min);
	pid_Y->reset();
	
	while( ros::ok() )
	{	
		cmd_vel_pub.publish( updateCmdVel() );
		
		ros::spinOnce();
		ros::Rate(20).sleep();
	}
	
	return 0;
}
