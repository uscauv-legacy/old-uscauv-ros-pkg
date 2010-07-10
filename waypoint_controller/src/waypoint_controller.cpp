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

void MapCallback(const localization_defs::LandmarkMapMsgConstPtr & map)
{
	
}

void updateCmdVel()
{
	
}

int main( int argc, char * argv[] )
{
	ros::init(argc, argv, "waypoint_controller");
	ros::NodeHandle n("~");
	
	n.param("robot_frame", robot_frame, std::string("base_link") );
	
	n.param("map_frame", map_frame, std::string("map") );
	
	ros::Subscriber map_sub = n.subscribe("map", 1, MapCallback);
	
	ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	
	ros::ServiceClient set_desired_rpy_srv = n.serviceClient<seabee3_driver::SetDesiredRPY>("setRPY");
	
	ros::ServiceClient set_desired_xyz_srv = n.serviceClient<seabee3_driver::SetDesiredXYZ>("setXYZ");
	
	pid_D = new control_toolbox::Pid;
	pid_R = new control_toolbox::Pid;
	pid_P = new control_toolbox::Pid;
	pid_Y = new control_toolbox::Pid;
	
	PIDConfig pid_D_cfg, pid_R_cfg, pid_P_cfg, pid_Y_cfg;
	
	double pid_i_min, pid_i_max;
	
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
	
	pid_D->initPid(pid_D_cfg.p, pid_D_cfg.i, pid_D_cfg.d, pid_i_max, pid_i_min);
	pid_D->reset();
	pid_R->initPid(pid_R_cfg.p, pid_R_cfg.i, pid_R_cfg.d, pid_i_max, pid_i_min);
	pid_R->reset();
	pid_P->initPid(pid_P_cfg.p, pid_P_cfg.i, pid_P_cfg.d, pid_i_max, pid_i_min);
	pid_P->reset();
	pid_Y->initPid(pid_Y_cfg.p, pid_Y_cfg.i, pid_Y_cfg.d, pid_i_max, pid_i_min);
	pid_Y->reset();
	
	while( ros::ok() )
	{	
		cmd_vel_pub( updateCmdVel() );
		
		ros::spinOnce();
		ros::Rate(20).sleep();
	}
	
	return 0;
}
